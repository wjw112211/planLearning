cmake_minimum_required(VERSION 3.0.2)
project(pursuit)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  logManager
  nav_msgs
  roscpp
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    logManager
    nav_msgs
    roscpp
    tf2
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pursuit_node
  src/pursuit_node.cpp
  src/pursuit_interface.cpp
  src/pursuit_core.cpp
)

add_dependencies(pursuit_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(pursuit_node
  ${catkin_LIBRARIES}
)

install(TARGETS pursuit_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
