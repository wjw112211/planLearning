# 一、Gazebo Grid Map 2D 

这个工作区用于启动 TurtleBot3 的 Gazebo 仿真，并在 RViz 中显示对应的二维栅格地图。
```bash
roslaunch turtlebot3_gazebo gazebo_grid_map.launch
```

## 1.1常用参数
```bash
# 指定 TurtleBot3 模型
model:=burger

# 使用 SLAM 模式
 use_static_map:=false run_slam:=true

# 速度指令
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 1.2代价地图
`grid_map_gazebo_bridge` 会把占据栅格地图转换成代价地图，代价取值沿用 ROS costmap_2d 的约定：
0 为自由空间，1~252 为膨胀代价，253 为紧贴障碍的内切膨胀栅格，254 为致命障碍，未知区域为 -1。

* `/map`：输入的占据栅格地图，来自静态地图或 gmapping。
* `/grid_map`：grid_map 形式的代价地图，包含 `occupancy`（原始占据）和 `cost`（代价）两个图层。
* `/costmap`：OccupancyGrid 形式的代价地图，`cost` 由 [0, 254] 归一化到 [0, 100]，供 RViz 与规划器使用。

RViz 只显示代价地图本身：白为自由空间，黑为致命障碍，灰色梯度为膨胀代价。


## 1.3速度指令与自动停车
Gazebo 的差速驱动插件（libgazebo_ros_diff_drive.so）没有指令超时机制：只要收到过一次 `/cmd_vel`，
就会一直按这个速度驱动轮子，直到收到新的指令。所以 `rostopic pub` 结束之后机器人不会自己停下。

`cmd_vel_watchdog` 节点订阅 `/cmd_vel`，超过 `cmd_vel_timeout` 秒没有收到新指令，就发布全 0 的
Twist 让机器人停下。

```bash
# 停止发指令 1 秒后自动停车：修改 src/bringup/config/gazebo_grid_map.yaml 中的 timeout

# 关掉看门狗（机器人会保持最后一次速度指令）
roslaunch bringup gazebo_grid_map.launch launch_cmd_vel_watchdog:=false
```


# 二、编译和启动bringup

grid map 相关运行参数集中放在 `src/bringup/config/gazebo_grid_map.yaml` 中，启动文件在
`src/bringup/launch/gazebo_grid_map.launch` 中直接加载该 YAML。简单说，bringup中重新实现了启动文件，目的是把左右启动文件集中到一块儿。

## 2.1编译

```bash
cd /home/wjw/Algorithm
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

> 如果要用 `src/common/logManager`（日志模块），编译前先装 glog：
> `sudo apt install libgoogle-glog-dev`（会一起装上 libgflags-dev）。

## 2.2启动

启动gazebo和rviz,打开地图，可以看到机器人仿真环境以及生成的二维栅格地图，以及相应的代价地图。
```bash
# 建图
roslaunch bringup manual_mapping.launch

# 定位，默认使用最新的地图
roslaunch bringup grid_map.launch

#全局规划
roslaunch bringup dijkstra.launch
```

# 三、日志模块 logManager

`src/common/logManager` 在 glog 外面包了一层，glog 的原生用法（`LOG(INFO)`、`VLOG()`、`CHECK()` ...）全部保留，
另外加了三个功能：日志离线保存、定时清理、按时间节流打印。详细说明见
`src/common/logManager/README.md`。

## 3.1需要的环境

```bash
sudo apt install libgoogle-glog-dev
```

## 3.2使用

别的 package 里只要在 CMakeLists.txt 的 `find_package(catkin ... COMPONENTS ...)` 中加上 `logManager`，
再 `#include "logManager/logManager.h"`，然后：

```cpp
LOG_INIT();                        // 日志写到 <工程根>/log 下，保留 7 天，定时清理

LOG(INFO) << "原生 glog 用法不变";
LOG_EVERY_T(INFO, 1.0) << "同一个调用点 1 秒最多打一次（第一次一定打）";
```

mapping 相关的节点（`occupancy_grid_to_cost_map`、`static_turtlebot3_world_map`、`cmd_vel_watchdog`、
`turtlebot3_drive`）用第 4 个参数把日志文件名固定成 `mapping`，落盘就是 `log/mapping.INFO`、
`log/mapping.WARNING` 这些文件：

```cpp
LOG_INIT("./log", 7, 6 * 3600, "mapping");
```

日志文件默认落在工程最外层目录（和 `src`、`build`、`devel` 同级）的 `log/` 里。
相对路径（`./log`）不是相对进程 cwd，而是相对 catkin 工作空间根目录：`logManager` 会从可执行文件
位置往上一层层找"同时有 `devel` 和 `src`"的那层，所以 roslaunch 在哪个目录下启动都一样：

```bash
roslaunch bringup gazebo_grid_map.launch      # 日志照样落在 /home/wjw/Algorithm/log
```

> 之所以要这样处理：roslaunch 默认把节点的 cwd 设成 `ROS_HOME`（`~/.ros`），直接用 cwd 的话
> 日志会写到 `~/.ros/log`，工程目录里看不到 `log/`。想换位置就在 `LOG_INIT` 里传绝对路径。

每次启动的第一条 INFO 日志会写明实际落盘位置，例如
`I0927 16:08:47.132158 58478 logManager.cpp:242] logging to /home/wjw/Algorithm/log (keep 7 days)`。


