# Gazebo Grid Map 2D 示例

这个工作区用于启动 TurtleBot3 的 Gazebo 仿真，并在 RViz 中显示对应的二维栅格地图。

## 编译

```bash
cd /home/wjw/Algorithm
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 启动

```bash
cd /home/wjw/Algorithm
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch turtlebot3_gazebo gazebo_grid_map.launch
```

启动后会打开 Gazebo 和 RViz，可以看到机器人仿真环境以及生成的二维栅格地图。

## 常用参数

```bash
# 指定 TurtleBot3 模型
roslaunch turtlebot3_gazebo gazebo_grid_map.launch model:=burger

# 使用 SLAM 模式
roslaunch turtlebot3_gazebo gazebo_grid_map.launch use_static_map:=false run_slam:=true

# 速度指令
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 代价地图

`grid_map_gazebo_bridge` 会把占据栅格地图转换成代价地图，代价取值沿用 ROS costmap_2d 的约定：
0 为自由空间，1~252 为膨胀代价，253 为紧贴障碍的内切膨胀栅格，254 为致命障碍，未知区域为 -1。

* `/map`：输入的占据栅格地图，来自静态地图或 gmapping。
* `/grid_map`：grid_map 形式的代价地图，包含 `occupancy`（原始占据）和 `cost`（代价）两个图层。
* `/costmap`：OccupancyGrid 形式的代价地图，`cost` 由 [0, 254] 归一化到 [0, 100]，供 RViz 与规划器使用。

RViz 只显示代价地图本身：白为自由空间，黑为致命障碍，灰色梯度为膨胀代价。

代价地图参数的默认值在 `grid_map_gazebo_bridge/launch/grid_map_gazebo_bridge.launch` 中，也可以在启动时覆盖：

```bash
# 占据值达到多少算致命障碍
roslaunch turtlebot3_gazebo gazebo_grid_map.launch lethal_threshold:=65
# 膨胀半径，超出该距离的栅格代价为 0
roslaunch turtlebot3_gazebo gazebo_grid_map.launch inflation_radius:=0.8
```


