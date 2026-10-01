cmake_minimum_required(VERSION 3.0.2)
project(minijerk)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  logManager
  nav_msgs
  roscpp
)

# QP求解器：OSQP的C库和CMake包（由 ros-noetic-osqp-vendor 提供）
# 首选系统安装：sudo apt install ros-noetic-osqp-vendor
# 没有root权限时，把vendor包解包到 $HOME/.local/osqp_vendor，
# 或者用环境变量 OSQP_PREFIX 指向解包出来的前缀，下面会自动兜底
find_package(osqp QUIET)
if(NOT osqp_FOUND)
  foreach(osqp_candidate "$ENV{OSQP_PREFIX}" "$ENV{HOME}/.local/osqp_vendor")
    if(osqp_candidate AND EXISTS "${osqp_candidate}/lib/cmake/osqp/osqp-config.cmake")
      list(APPEND CMAKE_PREFIX_PATH "${osqp_candidate}")
      set(OSQP_LOCAL_PREFIX "${osqp_candidate}")
      message(STATUS "osqp 未系统安装，使用本地前缀：${osqp_candidate}")
      break()
    endif()
  endforeach()
  find_package(osqp REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    logManager
    nav_msgs
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(minijerk_node
  src/minijerk_node.cpp
  src/minijerk_interface.cpp
  src/minijerk_core.cpp
)

add_dependencies(minijerk_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(minijerk_node
  ${catkin_LIBRARIES}
  osqp::osqp
)

# 本地前缀不在系统库搜索路径里，给可执行文件写rpath，省得再配LD_LIBRARY_PATH
if(OSQP_LOCAL_PREFIX)
  set_target_properties(minijerk_node PROPERTIES BUILD_RPATH "${OSQP_LOCAL_PREFIX}/lib")
endif()

install(TARGETS minijerk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
