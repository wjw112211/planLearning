// logManager: 在 glog 基础上增加日志离线保存、定时清理、按时间节流打印的功能。
//
// 用法（别的 package 里只要三步）：
//   1) CMakeLists.txt: find_package(catkin REQUIRED COMPONENTS logManager)
//   2) #include "logManager/logManager.h"
//   3) main() 开头调用一次 LOG_INIT(); 之后 LOG(INFO)、LOG_EVERY_T(INFO, 1.0) 随便用。
#ifndef LOG_MANAGER_LOG_MANAGER_H_
#define LOG_MANAGER_LOG_MANAGER_H_

#include <string>

// glog 的原生宏（LOG / VLOG / LOG_IF / CHECK ...）全部保留，这里直接转出去
#include <glog/logging.h>

namespace logManager {

// 初始化日志，进程里调用一次即可（多线程同时调用也安全，只有第一次生效）：
//   log_dir  日志目录，用相对路径，默认 "./log"，也就是和 src/build/devel 同级的那一层
//            （所以请在工程根目录下启动程序，例如 cd /home/wjw/Algorithm 后再 roslaunch/rosrun）
//   keep_days            日志保留天数，超过天数的日志文件会被删掉，<=0 表示不按时间清理
//                        （启动时先清理一次，之后由清理线程按周期清理）
//   cleanup_interval_sec 清理线程的运行周期，单位秒，<=0 表示只清理一次、不起线程
void InitLogging(const std::string& log_dir = "./log",
                 int keep_days = 7,
                 int cleanup_interval_sec = 6 * 3600);

// 立刻清理一次过期日志（一般不用手动调）
void CleanupOldLogs(const std::string& log_dir = "./log", int keep_days = 7);

// 停止清理线程并关闭 glog，程序正常退出时会自动调用
void ShutdownLogging();

// LOG_EVERY_T 内部的节流判断：同一个调用点距离上次打印超过 interval_sec 秒才返回 true
bool ShouldLogEveryT(const char* file, int line, double interval_sec);

}  // namespace logManager

// 一行初始化，等价于 logManager::InitLogging()，也可以写成 LOG_INIT("./log", 3);
#define LOG_INIT(...) ::logManager::InitLogging(__VA_ARGS__)

// 按时间节流：同一个调用点最多每 interval_sec 秒打印一次（单位秒，可以是小数），
// 用法和 LOG(severity) 完全一样：
//   LOG_EVERY_T(INFO, 1.0) << "1 秒最多打一条，第一次一定会打";
#define LOG_EVERY_T(severity, interval_sec)                     \
  LOG_IF(severity,                                              \
         ::logManager::ShouldLogEveryT(__FILE__, __LINE__,      \
                                       static_cast<double>(interval_sec)))

#endif  // LOG_MANAGER_LOG_MANAGER_H_
