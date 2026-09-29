#include <ros/ros.h>
#include "dijkstra_interface.hpp"
#include "dijkstra_core.hpp"


int main(int argc, char** argv)
{
    // 创建ROS节点
    ros::init(argc, argv, "dijkstra_node");
    // 初始化日志管理器
    LOG_INIT("./log", 7, 6 * 3600, "dijkstra");

    dijkstra::DijkstraInterface dijkstra_interface;

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();    // 处理回调函数
        // 系统自动分配线程，调用processGlobalPlanRequest()函数进行全局规划
        dijkstra_interface.processOnce();
        rate.sleep();
    }

    return 0;    
}