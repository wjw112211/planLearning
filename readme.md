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

#轨迹平滑（订阅 /global_path，发布 /global_path_smooth）
roslaunch bringup minijerk.launch

#路径跟踪（订阅 /global_path_smooth，发布 /cmd_vel）
roslaunch bringup pursuit.launch
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


# 四、路径跟踪 pursuit（纯追踪）

`src/followPath/pursuit` 用纯追踪（pure pursuit）算法让机器人沿全局路径走。它只订阅路径、发速度，
不做规划也不发新路径，所以要和 dijkstra + minijerk 一起用：

```
RViz 点 2D Nav Goal → dijkstra → /global_path → minijerk → /global_path_smooth → pursuit → /cmd_vel → Gazebo
```

分层的写法和 `trajgenerate/minijerk` 一致：`pursuit_node`（主循环，按控制频率调 `processOnce`）
→ `pursuit_interface`（订阅、发布、TF 取位姿）→ `pursuit_core`（纯算法，只吃位姿吐指令）。

## 4.1参数

参数全部放在 `src/followPath/pursuit/config/pursuit.yaml` 里，改完重新 `roslaunch` 就生效，不用重新编译。
命令行也能临时覆盖（不传就和 YAML 一致），例如
`roslaunch bringup pursuit.launch linear_speed:=0.15 lookahead_distance:=0.5`。

| 参数 | 含义 |
| --- | --- |
| `path_topic` | 待跟踪的路径，默认 `/global_path_smooth`（想跟原始路径就改成 `/global_path`） |
| `cmd_vel_topic` | 速度指令话题，默认 `/cmd_vel` |
| `control_frequency` | 控制频率，默认 20 Hz |
| `lookahead_distance` | 基准前视距离，默认 0.4 m |
| `lookahead_gain` | 自适应前视增益，默认 0.6 s：`ld = 基准 + 增益 * 上一拍线速度` |
| `min_lookahead` / `max_lookahead` | 前视距离上下限，默认 0.2 / 1.2 m |
| `linear_speed` | 最高线速度，默认 0.22 m/s（burger 上限） |
| `min_linear_speed` | 最低线速度，默认 0.05 m/s，太小会被电机死区吃掉 |
| `max_angular_speed` | 角速度限幅，默认 1.5 rad/s，相当于给最小转弯半径兜底 |
| `max_angular_acceleration` | 指令角速度的变化率上限，默认 4 rad/s²：位姿、路径点的抖动不会被放大成乱打方向，设 0 关闭 |
| `deceleration_distance` | 离终点多远开始减速，默认 0.6 m |
| `goal_tolerance` | 到点判据，默认 0.05 m：离路径最后一个点小于它就停车。**这个值就是最终到点误差的上限**，判停还带迟滞（偏出 3 倍才会重新起步），不会停在边界上反复蹭 |
| `publish_zero_when_idle` | 空闲（无路径/已到终点）时是否每拍都发零速度，默认 true |

调参经验：`lookahead_distance` 是"贴线"和"稳"之间的权衡——调小跟得更贴、过弯切内弯更少，但对定位噪声更
敏感；调大更稳，代价是过弯切内弯更多（仿真数据，0.5 m 半径弯道：0.4 → 横向偏差 0.05 m，0.5 → 0.08 m）。
`lookahead_gain` 让车跑得快时看得更远。**如果车一直在左右摆，先别急着调前视距离，按 4.5 那节查一遍。**

## 4.2运行

```bash
roslaunch bringup grid_map.launch      # 定位（提供 map -> base_link 的 TF）
roslaunch bringup dijkstra.launch      # RViz 里点 2D Nav Goal，出 /global_path
roslaunch bringup minijerk.launch      # 平滑成 /global_path_smooth
roslaunch bringup pursuit.launch       # 跟 /global_path_smooth 走，发 /cmd_vel
```

机器人位姿来自 TF 的 `map -> base_link`（和 dijkstra 一致），拿不到 TF（定位没起来、坐标系名写错）
就会打一条限流日志并停车，不会乱跑。

调试时在 RViz 里加一个 `PointStamped` 显示 `/pursuit/lookahead_point`，就能看到算法当前盯的是路径上的哪个点，
跟踪不正常时先看这个点是不是跑飞了。

## 4.3目标点在身后会自动掉头

纯追踪的公式 `kappa = 2 * sin(alpha) / ld` 在 alpha 接近 ±180°（前视点正好在车身后方）时
会退化成 0：算出来的角速度几乎是 0，机器人不转弯、直着往前冲，离路径越来越远，看起来就是
"车往反方向跑了"。日志里表现为 `alpha≈±3.07、kappa≈0.07、w≈0.02`，`dist_to_goal` 一直不降。

所以算法里加了一条退化处理：只要前视点落在身后超过 90°，就**先原地掉头**（线速度归零，
只往需要的方向给角速度，被 `max_angular_speed` 限幅），等前视点转到身前再恢复正常跟踪，
日志会打一条 `[PURSUIT TURNING] ... turn in place`。

也就是说：在 RViz 里点目标点时，目标点在车头后方也没关系，车会先掉头再走。如果你不希望它掉头，
先用键盘遥控把车头摆到大致朝向路径的方向即可。

## 4.4停车行为

跟踪中每拍都发 `/cmd_vel`；到终点、没有路径、取不到位姿这三种情况都发零速度停车。
到点判停之后路径还留着（不会自动清空），在 RViz 里重新点一次 2D Nav Goal 发新路径就会重新跟上。

到点判据是 `goal_tolerance`（默认 0.05 m），**停稳后的误差就是这个量级**，想让车停得更准就把它调小
（代价：最后一段爬得更久，也更容易受定位噪声影响）。判停带迟滞：进入容差就锁存"已到达"，
只有被推、被撞出去超过 3 倍容差（默认 0.15 m）才恢复跟踪，避免车停在容差边界上"起步-停车"地反复蹭。

日志上：**没收到路径时只在启动后提示一条** `[PURSUIT WAIT] no path received yet ...`
（这是正常状态，不会一直刷警告），到达终点时打一条 `[PURSUIT STOP] arrived ...`；
只有真的收到路径却用不了（点太少、点全部重合）才按 1 Hz 报 `[PURSUIT FAILED]`，
TF 取不到位姿（定位没起来、坐标系名写错）也按 1 Hz 报 `[PURSUIT STOP] can not get robot pose`。

要注意 Gazebo 的差速插件没有指令超时（见 1.3），`publish_zero_when_idle: false` 时节点只在刚停下那一拍
发一次零速度，之后 `/cmd_vel` 安静下来，方便键盘遥控接管，但此时 `cmd_vel_watchdog` 可能会在一段时间后
再补一脚零速度——两者发的内容一样，不冲突。


## 4.5跟不准怎么排查（左右晃荡、到点不准）

先说结论：**纯追踪这一层的控制律本身在被控对象上是稳的**。用闭环仿真（车体运动学 + 底盘一阶延迟
0.15 s + 20 Hz 控制）验证过下面这些情况，横向偏差都不会发散、也不会来回摆：

| 情况 | 横向偏差（巡航段） | 偏差变号次数 |
| --- | --- | --- |
| 起步就贴在路径上 | 0.000 m | 0 |
| 起步横向偏 0.3 m | 收敛回路径 | 0 |
| 起步朝向差 0.5 rad | 0.083 m（起步段） | 0 |
| 走廊 90° 弯（半径 0.5 m） | 0.049 m | 0 |
| 定位噪声 1 cm / 0.5°、10 Hz 更新 | 0.049 m | 0 |
| 定位延迟 0.3 s | 0.033 m | 0 |
| 底盘实际角速度是指令的 3 倍（轮距标定错） | 0.129 m | 0 |

所以真车上一旦"一直左右晃荡、根本不在路径上"，原因基本在**输入**（路径本身、位姿估计）而不在控制律。
按下面顺序查，每一条都有对应的日志：

1. **日志里的 `cross_track`（单位 m，左正右负）**
   跟踪日志每次都会打：`[PURSUIT TRACKING] dist_to_goal=..., cross_track=..., ...`。
   它就是机器人离路径的真实横向偏差：
   - 数值在 ±0.05 上下跳 → 跟得很正常，你在 RViz 里看到的"晃"多半是两条不同的路径叠在一起；
   - 数值**一直很大且正负交替** → 真的在画龙，看第 2、3 条；
   - 数值**一直是同一个符号、变化不大** → 不是控制问题，而是定位/坐标系有一个固定偏差。
2. **你对比的是哪条路径**
   pursuit 默认跟的是 minijerk 平滑后的 `/global_path_smooth`，不是 dijkstra 的 `/global_path`。
   平滑会抹掉栅格台阶，也会在拐角处切内弯，两条线在弯道处相差几十厘米很正常。
   排查时把 `/global_path`（原始）、`/global_path_smooth`（平滑）、`/pursuit/lookahead_point`（当前盯的点）
   同时显示出来对比，**别拿原始折线当参照**。
3. **路径本身是不是锯齿/波浪**
   收到路径时会自动检查一次：如果相邻两段的转向方向反复变号（典型的四连通栅格路径，或者平滑参数没调好、
   拐角处甩出去的路径），会打一条警告
   `[PURSUIT PATH] received path looks jagged: N/M vertices flip the turning direction...`。
   这种路径纯追踪会"忠实地"跟着它左右摆，先确认 `path_topic` 是不是平滑后的路径、平滑的参数合不合适。
4. **位姿从哪来**
   `map -> base_link` 的 TF 是建图/定位估计出来的，不是真值。如果建图跑得慢、激光匹配有跳变，
   控制器看到的就是一个"在跳的"位姿，车自然跟着晃。先用 `rosrun tf tf_echo map base_link` 看看
   位姿是不是连续、平滑；定位不稳时先把车开慢（`linear_speed:=0.1`）再调。
5. **底盘是否按指令动**
   键盘遥控发 `cmd_vel` 里 `angular.z = 0.5`，量一下车是不是约 0.5 rad/s（转 2 s 约 57°）；
   转多了说明轮距/轮径标定不准或打滑，等效于把控制增益放大，会让跟踪偏差变大。

另外两个可用的旋钮：
- `goal_tolerance` 默认 0.05 m，**它就是最终到点误差的上限**（老配置是 0.15 m，仿真里停稳后离终点 0.14 m，
  调到 0.05 之后是 0.039 m）；调大更容易"到点"，但停得更远。
- `max_angular_acceleration` 默认 4 rad/s²。位姿或路径点突然跳变时，指令角速度会被限住按拍爬上去
  （20 Hz 下 0.55 s 爬到目标值），避免车头猛摆一下。设 0 关闭，改成 1~2 会让转向更柔和也更迟钝。
