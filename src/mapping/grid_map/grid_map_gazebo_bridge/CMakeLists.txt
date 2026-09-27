cmake_minimum_required(VERSION 3.0.2)
project(grid_map_gazebo_bridge)

add_compile_options(-std=c++11 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  grid_map_sdf
  logManager
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    grid_map_core
    grid_map_msgs
    grid_map_ros
    grid_map_sdf
    logManager
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(occupancy_grid_to_cost_map
  src/occupancy_grid_to_cost_map.cpp
)

add_executable(static_turtlebot3_world_map
  src/static_turtlebot3_world_map.cpp
)

add_dependencies(occupancy_grid_to_cost_map
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(static_turtlebot3_world_map
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(occupancy_grid_to_cost_map
  ${catkin_LIBRARIES}
)

target_link_libraries(static_turtlebot3_world_map
  ${catkin_LIBRARIES}
)

install(TARGETS
    occupancy_grid_to_cost_map
    static_turtlebot3_world_map
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
