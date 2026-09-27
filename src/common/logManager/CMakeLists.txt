cmake_minimum_required(VERSION 3.0.2)
project(logManager)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED)
find_package(Threads REQUIRED)

# Ubuntu 自带的 glog 只提供 pkg-config 文件（libglog.pc），没有 cmake config 文件
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(GLOG QUIET libglog)
endif()
if(NOT GLOG_FOUND)
  message(FATAL_ERROR "logManager 需要 glog，请先安装：sudo apt install libgoogle-glog-dev")
endif()

# 把 glog / pthread 一起导出去，下游 package 只要
# find_package(catkin REQUIRED COMPONENTS logManager) + link ${catkin_LIBRARIES}
# 就能直接用 LOG(INFO)、LOG_EVERY_T 这些宏
catkin_package(
  INCLUDE_DIRS
    include
    ${GLOG_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    glog
    ${CMAKE_THREAD_LIBS_INIT}
)

# glog 头文件（glog 装在非标准前缀时也能编译）
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/logManager.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}
  ${GLOG_LDFLAGS}
  ${CMAKE_THREAD_LIBS_INIT}
)

# ---------------------------------------------------------------------------
# 测试：test/test_logManager.cpp，自己带 main，不依赖 gtest。
#   catkin_make                                   # 会一起编出来
#   cd <工程根目录> && ./devel/lib/logManager/test_logManager
# 指定了 -DCATKIN_ENABLE_TESTING=OFF 时不编译。
# ---------------------------------------------------------------------------

# 需要使用的时候，需要把这个下面的内容注释掉，然后在 cmake 时指定 -DCATKIN_ENABLE_TESTING=ON
set(CATKIN_ENABLE_TESTING OFF)

if(CATKIN_ENABLE_TESTING)
  add_executable(test_logManager test/test_logManager.cpp)
  add_dependencies(test_logManager ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_logManager
    ${PROJECT_NAME}
    ${GLOG_LDFLAGS}
    ${CMAKE_THREAD_LIBS_INIT}
  )

  # 测试里 LOG_INIT("./log_test") 是相对路径，指定到 build 目录，
  # 免得 ctest 把日志写到源码树里；手动跑时还是落在当前工作目录。
  add_test(NAME test_logManager COMMAND test_logManager)
  set_tests_properties(test_logManager PROPERTIES
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  )
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
