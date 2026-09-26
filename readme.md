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


