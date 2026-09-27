cmake_minimum_required(VERSION 3.0.2)
project(grid_map_manual_mapping)

add_compile_options(-std=c++11 -Wall -Wextra -Wpedantic)

################################################################################
# 依赖
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  logManager
  rviz
  pluginlib
)

# 只为了读写 .pgm 的元信息 .yaml（map_server 那套格式）
find_package(yaml-cpp REQUIRED)

# rviz 面板是 Qt 控件，用 rviz 自己用的 Qt 版本，保证 ABI 一致
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  find_package(Qt5 REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    logManager
    rviz
    pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

################################################################################
# 节点：手动建图 / 定位
################################################################################
add_executable(manual_mapping_node src/manual_mapping_node.cpp)
add_dependencies(manual_mapping_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(manual_mapping_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 节点：键盘 wsad 遥控（发布 /cmd_vel）
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
add_dependencies(keyboard_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})

################################################################################
# RViz 面板插件：三个按钮（定位模式 / 建图模式 / 保存地图）
################################################################################
# 面板是 Qt 控件，要用 rviz 自己的 Qt 版本做 moc（和仓库里 grid_map_rviz_plugin 一个套路）
add_definitions(-DQT_NO_KEYWORDS)

set(PANEL_HEADERS
  include/grid_map_manual_mapping/manual_mapping_panel.h
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(PANEL_MOC_FILES ${PANEL_HEADERS})
else()
  qt5_wrap_cpp(PANEL_MOC_FILES ${PANEL_HEADERS})
endif()

add_library(manual_mapping_panel
  src/manual_mapping_panel.cpp
  ${PANEL_MOC_FILES}
)
add_dependencies(manual_mapping_panel ${catkin_EXPORTED_TARGETS})
target_link_libraries(manual_mapping_panel
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS manual_mapping_node keyboard_teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS manual_mapping_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch rviz config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
