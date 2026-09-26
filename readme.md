# Gazebo + Grid Map 2D Integration

This workspace now provides a single launch entry that starts TurtleBot3 in
Gazebo, publishes a static 2D map for the Gazebo world, converts it into a
planar grid map, and shows both in RViz.

## Dependencies

Install the ROS packages used by the combined launch:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-turtlebot3-description
```

Check that ROS can find them:

```bash
source /opt/ros/noetic/setup.bash
rospack find gazebo_ros
rospack find turtlebot3_description
```

## Workspace Layout

The workspace uses the standard catkin layout:

```text
/home/wjw/Algorithm
├── src
│   ├── common
│   ├── mapping
│   │   ├── grid_map
│   │   └── turtlebot3_simulations
│   └── pathFinding
├── build
└── devel
```

`src/mapping` contains the real source code, not symlinks.

Only the packages needed by this 2D Gazebo grid-map workflow are enabled by
default:

- `grid_map_core`
- `grid_map_sdf`
- `grid_map_cv`
- `grid_map_msgs`
- `grid_map_ros`
- `grid_map_visualization`
- `grid_map_rviz_plugin`
- `grid_map_gazebo_bridge`
- `turtlebot3_gazebo`

Other optional/demo packages remain in the source tree but contain
`CATKIN_IGNORE`, so `catkin_make` will not try to build unused PCL, OctoMap,
costmap, demo, or fake-TurtleBot packages.

## Build

```bash
cd /home/wjw/Algorithm
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Source `/home/wjw/Algorithm/devel/setup.bash` before launching so all packages
are visible.

## Run

```bash
cd /home/wjw/Algorithm
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch turtlebot3_gazebo gazebo_grid_map.launch
```

Useful options:

```bash
roslaunch turtlebot3_gazebo gazebo_grid_map.launch model:=burger
roslaunch turtlebot3_gazebo gazebo_grid_map.launch run_autonomous_drive:=false
roslaunch turtlebot3_gazebo gazebo_grid_map.launch static_map_type:=turtlebot3_square
roslaunch turtlebot3_gazebo gazebo_grid_map.launch use_static_map:=false run_slam:=true
```

`run_autonomous_drive` defaults to `false` because `turtlebot3_drive` is optional
and must be compiled from `turtlebot3_gazebo` before use.

Topics:

- `/map`: static 2D `nav_msgs/OccupancyGrid` for the Gazebo world.
- `/grid_map`: planar `grid_map_msgs/GridMap` with `occupancy` and `flat` layers.
- `/grid_map_visualization/occupancy_grid`: RViz-friendly occupancy grid rebuilt
  from the grid map.

If `Resource not found: gazebo_ros` appears, the Gazebo ROS packages are not
installed in the current ROS environment. Install the dependency package:

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs
```

If you switch to SLAM mode with `use_static_map:=false run_slam:=true`, install:

```bash
sudo apt install ros-noetic-slam-gmapping ros-noetic-gmapping
```
