# logManager

在 glog 外面包了一层，保留 glog 的全部原生用法（`LOG(INFO)`、`VLOG()`、`CHECK()` ...），另外加了三个功能：

1. **日志离线保存**：`LOG_INIT()` 之后所有日志都会写到工程最外层目录（和 `src`、`build`、`devel` 同级）的 `log/` 里，用相对路径 `./log`。
2. **定时删除**：日志默认保留 7 天，启动时先清理一次，之后后台线程按周期再清理。
3. **按时间节流**：`LOG_EVERY_T(INFO, 1.0)`，同一个调用点 1 秒最多打印一次（第一次一定打印）。

## 依赖

```bash
sudo apt install libgoogle-glog-dev   # 会一起装上 libgflags-dev
```

## 在自己的 package 里使用

`CMakeLists.txt` 里只要把 `logManager` 加到 `find_package` 的 COMPONENTS 里（glog、pthread 会被自动带上）：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  logManager
)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(my_node ${catkin_LIBRARIES})
```

`package.xml` 里加上：

```xml
<depend>logManager</depend>
```

代码里：

```cpp
#include "logManager/logManager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_node");
  LOG_INIT();                 // 默认写到 ./log，保留 7 天
  // LOG_INIT("./log", 3);    // 也可以：保留 3 天
  // LOG_INIT("./log", 3, 600); // 再指定清理周期（秒）

  LOG(INFO) << "原生 glog 用法不变";
  LOG_EVERY_T(INFO, 1.0) << "这条 1 秒最多打一次";

  while (ros::ok()) {
    // 或者不用宏，自己判断：
    if (logManager::ShouldLogEveryT(__FILE__, __LINE__, 5.0)) {
      LOG(INFO) << "5 秒最多打一次";
    }
  }
}
```

## glog 常用功能速查

`logManager.h` 里已经把 `<glog/logging.h>` 转出来了，下面这些都是 glog 原生功能，用法完全不变。

### 日志级别

```cpp
LOG(INFO) << "普通信息";     // 写到 *.INFO 文件
LOG(WARNING) << "警告";      // *.WARNING
LOG(ERROR) << "错误";        // *.ERROR
LOG(FATAL) << "致命错误";    // *.FATAL，打完直接 abort（会被 InstallFailureSignalHandler 捕获）
LOG(DFATAL) << "...";        // debug 构建里等于 FATAL，NDEBUG（Release）里降级成 ERROR
```

日志行格式：`I0927 15:00:05.922688 40709 my_node.cpp:67] 消息`，即
`级别+月日 时分秒.微秒 线程号 文件:行号] 消息`，多线程时靠线程号区分。

### 流式输出

```cpp
LOG(INFO) << "x = " << x << ", name = " << std::string("abc");
LOG(INFO) << "custom = " << my_type;   // 只要 my_type 有 operator<< 就行
```

想直接打印 `vector` / `map` / `pair` 等容器，额外 include（注意顺序，必须放在 logging.h 之后）：

```cpp
#include <glog/stl_logging.h>
LOG(INFO) << "v = " << std::vector<int>{1, 2, 3};   // 输出：v = 1 2 3
```

`<<` 后面的东西只有在达到级别门槛时才会求值，不会白算。

### 条件 / 次数 / 时间节流

```cpp
LOG_IF(INFO, i % 2 == 0) << "i 是偶数才打";
LOG_EVERY_N(INFO, 100) << "每 100 次打一条";              // glog 原生，按次数
LOG_FIRST_N(INFO, 3) << "只打前 3 次";
LOG_IF_EVERY_N(INFO, cond, 100) << "满足 cond 时每 100 次一条";
LOG_EVERY_T(INFO, 1.0) << "1 秒最多打一条";               // 本包新增，按时间
LOG_ASSERT(ptr != nullptr) << "条件不成立就 FATAL";
```

### VLOG：分级调试日志

```cpp
VLOG(1) << "默认不输出，要打开才看得到";
VLOG_IF(1, cond) << "满足条件才打";
VLOG_EVERY_N(1, 100) << "每 100 次一条";
if (VLOG_IS_ON(2)) { /* 昂贵的调试信息，只有需要时才计算 */ }
```

打开方式（推荐在代码里设，见下面的命令行说明）：

```cpp
FLAGS_v = 2;                          // 全局打开 1、2 级
google::SetVLOGLevel("my_node", 2);   // 只打开某个文件/模块
// 命令行：--v=2 / --vmodule=my_node=2 需要程序自己调 gflags::ParseCommandLineFlags 才生效，
// 直接传给 ROS 节点是没用的（实测 --v=2、--minloglevel=1 都不起作用），所以代码里赋值最靠谱。
```

### CHECK 检查 / 断言

```cpp
CHECK(ptr != nullptr);
CHECK_EQ(a, b) << "可以自己补一句提示";
CHECK_NE(a, b);  CHECK_LT(a, b);  CHECK_LE(a, b);  CHECK_GT(a, b);  CHECK_GE(a, b);
CHECK_NOTNULL(ptr);
CHECK_STREQ(s1, s2);  CHECK_STRNE(s1, s2);  CHECK_STRCASEEQ(s1, s2);
CHECK_NEAR(1.0, 1.0009, 1e-3);  CHECK_DOUBLE_EQ(a, b);
CHECK_ERR(::close(fd));        // 返回值 == -1 就报错，适合检查系统调用

DCHECK_EQ(a, b);               // DCHECK 系列只在没有 NDEBUG 的构建里生效
```

> catkin_make 默认不加 `-DNDEBUG`，所以 `DCHECK` 是生效的；`-DCMAKE_BUILD_TYPE=Release` 后会变成空操作。

### PLOG：自动带上 errno

```cpp
const int fd = ::open("/not/exist", O_RDONLY);
PLOG_IF(ERROR, fd < 0) << "打开文件失败";
// 输出：打开文件失败: No such file or directory [2]
PLOG(ERROR) << "同样会带上 errno";
```

### RAW_LOG：初始化之前、信号处理里也能用

```cpp
#include <glog/raw_logging.h>
RAW_LOG(INFO, "printf 风格 %d %s", 42, "abc");
RAW_CHECK(cond, "msg");
```

不做缓冲、直接写，所以日志格式里没有正常时间戳（`I0000 00:00:00.000000`）。


### 落盘的文件长什么样

```
log/
├── my_node.INFO                      # 软链接，指向最新的 INFO 文件
├── my_node.WARNING  my_node.ERROR  my_node.FATAL
└── my_node.wjw.wjw.log.INFO.20260927-150005.40709   # 程序名.主机名.用户名.log.级别.日期-进程号
```

每个级别一个文件 + 一个软链接，`tail -f log/my_node.INFO` 就能实时看日志；超过保留天数的文件由本包的清理线程删掉。

### 线程安全

`LOG(...)`、`CHECK(...)`、`VLOG(...)` 都是线程安全的，`<<` 到同一条消息也是原子的；本包新增的 `LOG_EVERY_T` 同样线程安全（同一个调用点共用一把锁保护时间戳）。

## 测试

`test/test_logManager.cpp` 是个自带 `main` 的简单测试，不依赖 gtest，`CATKIN_ENABLE_TESTING=ON`（默认）时
`catkin_make` 会一起编出来：

```bash
cd /home/wjw/Algorithm
catkin_make                                        # 编出 devel/lib/logManager/test_logManager
./devel/lib/logManager/test_logManager             # 手动跑，日志落在 ./log_test
# 或者用 ctest（工作目录是 build/common/logManager，日志落在那里）
cd build && ctest -R test_logManager -V
```

测试内容：`LOG_INIT` + 原生 `LOG(INFO)` 可用；`ShouldLogEveryT` 同一个调用点首次为 true、间隔内为 false；
`interval <= 0` 不节流；`LOG_EVERY_T` 循环 3 次只输出 1 条；不同调用点互不影响；`ShutdownLogging` 能正常退出。
全过打印 `ALL PASSED` 并返回 0。

> 注意：`ShouldLogEveryT` 是按 `__FILE__ + __LINE__` 区分调用点的，所以测"同一个调用点"时必须让多次调用
> 落在同一行（测试里用一个 lambda 把调用点固定住），否则每次都是新调用点、永远不会被节流。

## 注意

* `LOG_INIT()` 请在 `main()` 里调用一次；因为用的是 `./log` 这样的相对路径，所以程序请在工程根目录下启动
  （例如 `cd ~/Algorithm && roslaunch bringup ...`），日志才会落在 `~/Algorithm/log` 里。
* `keep_days <= 0` 表示不按时间清理（`cleanup_interval_sec <= 0` 表示只清理一次、不起后台线程）。
* 一个进程只初始化一次（多线程/多次调用都安全，只有第一次生效）。
* 退出时（或者手动调用 `logManager::ShutdownLogging()`）会停掉清理线程并 flush。
