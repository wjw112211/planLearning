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
//   log_dir  日志目录，默认 "./log"，也就是和 src/build/devel 同级的那一层。
//            相对路径会先挂到 catkin 工作空间根目录下（从可执行文件位置往上一层层找出
//            同时有 devel 和 src 的那层），所以 roslaunch/rosrun 从哪个目录启动都不影响；
//            只有找不到工作空间（例如装到 /opt/ros 下）时才退回原来相对 cwd 的语义。
//            想写到别的地方直接给绝对路径。
//   keep_days            日志保留天数，超过天数的日志文件会被删掉，<=0 表示不按时间清理
//                        （启动时先清理一次，之后由清理线程按周期清理）
//   cleanup_interval_sec 清理线程的运行周期，单位秒，<=0 表示只清理一次、不起线程
//   log_name             日志文件名前缀，例如 "mapping" 会生成 log/mapping.INFO、
//                        log/mapping.WARNING 这些文件；空字符串表示用可执行文件名
//                        （例如 cmd_vel_watchdog）
void InitLogging(const std::string& log_dir = "./log",
                 int keep_days = 7,
                 int cleanup_interval_sec = 6 * 3600,
                 const std::string& log_name = "");

// 立刻清理一次过期日志（一般不用手动调），log_dir 的解析规则和 InitLogging 一样
void CleanupOldLogs(const std::string& log_dir = "./log", int keep_days = 7);

// 停止清理线程并关闭 glog，程序正常退出时会自动调用
void ShutdownLogging();

// LOG_EVERY_T 内部的节流判断：同一个调用点距离上次打印超过 interval_sec 秒才返回 true
bool ShouldLogEveryT(const char* file, int line, double interval_sec);

}  // namespace logManager

// 一行初始化，等价于 logManager::InitLogging()，也可以写成 LOG_INIT("./log", 3)；
// 想固定日志文件名（例如落盘成 log/mapping.INFO）就再传一个文件名前缀：
//   LOG_INIT("./log", 7, 6 * 3600, "mapping");
#define LOG_INIT(...) ::logManager::InitLogging(__VA_ARGS__)

// 按时间节流：同一个调用点最多每 interval_sec 秒打印一次（单位秒，可以是小数），
// 用法和 LOG(severity) 完全一样：
//   LOG_EVERY_T(INFO, 1.0) << "1 秒最多打一条，第一次一定会打";
#define LOG_EVERY_T(severity, interval_sec)                     \
  LOG_IF(severity,                                              \
         ::logManager::ShouldLogEveryT(__FILE__, __LINE__,      \
                                       static_cast<double>(interval_sec)))

#endif  // LOG_MANAGER_LOG_MANAGER_H_
