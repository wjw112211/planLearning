#include <ros/ros.h>
#include "minijerk_interface.hpp"


int main(int argc, char** argv)
{
    // 创建ROS节点
    ros::init(argc, argv, "minijerk_node");
    // 初始化日志管理器
    LOG_INIT("./log", 7, 6 * 3600, "minijerk");

    minijerk::MiniJerk mini_jerk;

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();    // 处理回调函数
        // 收到新的粗糙轨迹就平滑并发布
        mini_jerk.processOnce();
        rate.sleep();
    }

    return 0;
}
