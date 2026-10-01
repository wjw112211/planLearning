#include <ros/ros.h>
#include "pursuit_interface.hpp"


int main(int argc, char** argv)
{
    // 创建ROS节点
    ros::init(argc, argv, "pursuit_node");
    // 初始化日志管理器
    LOG_INIT("./log", 7, 6 * 3600, "pursuit");

    pursuit::Pursuit pursuit;

    // 控制频率从 config/pursuit.yaml 读，参数异常时退回 20 Hz，避免除零或者空转
    const double control_frequency = pursuit.getControlFrequency();
    if(control_frequency <= 0.0)
    {
        LOG(WARNING) << "invalid control frequency: " << control_frequency << ", use 20 Hz instead.";
    }

    ros::Rate rate(control_frequency > 0.0 ? control_frequency : 20.0);
    while (ros::ok()) {
        ros::spinOnce();    // 处理回调函数
        // 每拍算一次跟踪指令：有路径就跟着走，没有路径就发零速度停车
        pursuit.processOnce();
        rate.sleep();
    }

    return 0;
}
