// logManager 的简单测试：自己就是个 main，不依赖 gtest。
//
// 编译运行（在工程根目录）：
//   source /opt/ros/noetic/setup.bash && source devel/setup.bash
//   g++ -std=c++11 -Iinclude test/test_logManager.cpp -o /tmp/test_logManager
//       -Ldevel/lib -llogManager -lglog -lgflags -lpthread
//   cd /home/wjw/Algorithm && /tmp/test_logManager
//
// 或者挂到 CMakeLists.txt 里：
//   add_executable(test_logManager test/test_logManager.cpp)
//   target_link_libraries(test_logManager ${catkin_LIBRARIES})

#include <cstdio>

#include "logManager/logManager.h"

static int g_failed = 0;

static void Check(bool ok, const char* name) {
  if (ok) {
    LOG(INFO) << "[PASS] " << name;
  } else {
    LOG(ERROR) << "[FAIL] " << name;
    ++g_failed;
  }
}

int main() {
  // 测试日志写到 ./log_test，不按时间清理（keep_days=0），也不起清理线程（interval=0）
  LOG_INIT("./log_test", 0, 0);

  // 1. 原生 glog 用法还能用，日志会落到 ./log_test
  LOG(INFO) << "logManager test start";

  // 下面这些调用都写在 lambda 里，所以 __FILE__/__LINE__ 固定为同一行，
  // 也就是同一个调用点，会共享同一条节流记录。
  const auto throttle_same_site = [](double interval_sec) {
    return logManager::ShouldLogEveryT(__FILE__, __LINE__, interval_sec);
  };

  // 2. 同一个调用点：第一次 true，1 秒内第二次 false
  Check(throttle_same_site(1.0) == true,
        "first call at a site returns true");
  Check(throttle_same_site(1.0) == false,
        "second call at the same site within interval returns false");

  // 3. interval <= 0 表示不节流，同一个调用点连着问两次都是 true
  Check(throttle_same_site(0.0) == true,
        "interval <= 0 disables throttling (1st)");
  Check(throttle_same_site(0.0) == true,
        "interval <= 0 disables throttling (2nd)");

  // 4. LOG_EVERY_T：同一个调用点循环打 3 次，只有第一次会真的输出
  for (int i = 0; i < 3; ++i) {
    LOG_EVERY_T(INFO, 1.0) << "LOG_EVERY_T 只应该打一次, i = " << i;
  }

  // 5. 节流是按调用点分开记的：换一个新调用点不受刚才那条记录影响
  Check(logManager::ShouldLogEveryT(__FILE__, __LINE__, 1.0) == true,
        "a different call site is not throttled");

  // 6. 手动清理 + 关闭，能正常退出（不崩溃、不卡住）
  LOG(INFO) << "logManager test done";
  logManager::CleanupOldLogs("./log_test", 0);
  logManager::ShutdownLogging();

  if (g_failed == 0) {
    printf("ALL PASSED\n");
    return 0;
  }
  printf("FAILED: %d check(s)\n", g_failed);
  return 1;
}
