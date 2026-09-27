#include "logManager/logManager.h"

#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace logManager {
namespace {

// --------------------------------------------------------------------------
// LOG_EVERY_T 用到的按调用点节流
// --------------------------------------------------------------------------
struct LogSite {
  const char* file;
  int line;
  bool operator==(const LogSite& other) const {
    return file == other.file && line == other.line;
  }
};

struct LogSiteHash {
  std::size_t operator()(const LogSite& site) const {
    return std::hash<const void*>()(site.file) ^
           static_cast<std::size_t>(site.line);
  }
};

std::mutex g_throttle_mutex;
std::unordered_map<LogSite, std::chrono::steady_clock::time_point, LogSiteHash>
    g_last_log_time;

// --------------------------------------------------------------------------
// 初始化 / 清理线程
// --------------------------------------------------------------------------
std::once_flag g_init_once;
std::thread g_cleanup_thread;
std::mutex g_cleanup_mutex;
std::condition_variable g_cleanup_cv;
bool g_stop_cleanup = false;

// 日志文件名前缀，例如 occupancy_grid_to_cost_map。
// 注意：glog 只是保存 InitGoogleLogging() 传进去的指针（不拷贝字符串），
// 所以这个字符串必须一直活到进程结束，不能传临时变量。
std::string g_program_name = "algorithm";

// 程序名（日志文件名前缀），例如 occupancy_grid_to_cost_map
std::string GetProgramName() {
  char buffer[1024];
  const ssize_t length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (length <= 0) {
    return "algorithm";
  }
  buffer[length] = '\0';
  const char* name = ::strrchr(buffer, '/');
  return name != nullptr ? std::string(name + 1) : std::string(buffer);
}

// 递归创建目录，相对路径、绝对路径都可以
void EnsureDirectory(const std::string& dir) {
  if (dir.empty()) {
    return;
  }
  std::size_t pos = 0;
  while (pos <= dir.size()) {
    const std::size_t slash = dir.find('/', pos);
    const std::size_t end = (slash == std::string::npos) ? dir.size() : slash;
    const std::string path = dir.substr(0, end);
    if (!path.empty() && path != "." && path != "..") {
      // 已经存在（EEXIST）或者创建失败都交给 glog 去报错，这里不打断程序
      ::mkdir(path.c_str(), 0755);
    }
    if (slash == std::string::npos) {
      break;
    }
    pos = slash + 1;
  }
}

// 删除 dir 下超过 keep_days 天没更新的日志文件
void CleanupExpiredFiles(const std::string& dir, int keep_days) {
  if (keep_days <= 0) {
    return;
  }
  DIR* dirp = ::opendir(dir.c_str());
  if (dirp == nullptr) {
    return;
  }

  const std::time_t now = std::time(nullptr);
  const double expire_sec = static_cast<double>(keep_days) * 24 * 3600;
  std::vector<std::string> expired;
  struct dirent* entry = nullptr;
  while ((entry = ::readdir(dirp)) != nullptr) {
    const std::string name = entry->d_name;
    if (name == "." || name == "..") {
      continue;
    }
    const std::string path = dir + "/" + name;
    struct stat info;
    if (::lstat(path.c_str(), &info) != 0) {
      continue;
    }
    if (S_ISLNK(info.st_mode)) {
      // glog 生成的 INFO / WARNING ... 软链接，指向的文件已经删掉就一起清掉
      struct stat target_info;
      if (::stat(path.c_str(), &target_info) != 0) {
        expired.push_back(path);
      }
      continue;
    }
    if (!S_ISREG(info.st_mode)) {
      continue;
    }
    if (std::difftime(now, info.st_mtime) > expire_sec) {
      expired.push_back(path);
    }
  }
  ::closedir(dirp);

  for (const std::string& path : expired) {
    ::unlink(path.c_str());
  }
}

// 先清理一次，再按周期后台清理
void StartCleanup(const std::string& dir, int keep_days, int interval_sec) {
  CleanupExpiredFiles(dir, keep_days);
  if (interval_sec <= 0) {
    return;
  }
  g_cleanup_thread = std::thread([dir, keep_days, interval_sec]() {
    std::unique_lock<std::mutex> lock(g_cleanup_mutex);
    while (!g_stop_cleanup) {
      if (g_cleanup_cv.wait_for(lock, std::chrono::seconds(interval_sec),
                                []() { return g_stop_cleanup; })) {
        break;
      }
      lock.unlock();
      CleanupExpiredFiles(dir, keep_days);
      lock.lock();
    }
  });
}

}  // namespace

void InitLogging(const std::string& log_dir, int keep_days,
                 int cleanup_interval_sec) {
  // 只在第一次调用时生效
  std::call_once(g_init_once, [log_dir, keep_days, cleanup_interval_sec]() {
    EnsureDirectory(log_dir);

    // 相对路径，落在程序的工作目录下（工程根目录启动时就是 <工程根>/log）
    FLAGS_log_dir = log_dir;
    FLAGS_logtostderr = false;     // 离线保存到文件
    FLAGS_alsologtostderr = true;  // 同时输出到终端，方便调试
    FLAGS_logbufsecs = 0;          // 不缓存，立刻写盘，崩溃也不丢日志

    g_program_name = GetProgramName();
    ::google::InitGoogleLogging(g_program_name.c_str());

    StartCleanup(log_dir, keep_days, cleanup_interval_sec);
    std::atexit(&ShutdownLogging);
  });
}

void CleanupOldLogs(const std::string& log_dir, int keep_days) {
  CleanupExpiredFiles(log_dir, keep_days);
}

void ShutdownLogging() {
  static std::atomic<bool> shutdown_done(false);
  bool expected = false;
  if (!shutdown_done.compare_exchange_strong(expected, true)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(g_cleanup_mutex);
    g_stop_cleanup = true;
  }
  g_cleanup_cv.notify_all();
  if (g_cleanup_thread.joinable()) {
    g_cleanup_thread.join();
  }

  ::google::ShutdownGoogleLogging();
}

bool ShouldLogEveryT(const char* file, int line, double interval_sec) {
  if (interval_sec <= 0.0) {
    return true;  // 不做节流
  }

  const auto now = std::chrono::steady_clock::now();
  const LogSite site{file, line};

  std::lock_guard<std::mutex> lock(g_throttle_mutex);
  const auto iter = g_last_log_time.find(site);
  if (iter != g_last_log_time.end() &&
      std::chrono::duration<double>(now - iter->second).count() < interval_sec) {
    return false;
  }
  g_last_log_time[site] = now;
  return true;
}

}  // namespace logManager
