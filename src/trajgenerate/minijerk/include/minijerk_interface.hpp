#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <shared_mutex>
#include "logManager/logManger.h"
#include "minijerk_core.hpp"


namespace minijerk
{
class MiniJerk
{
public:
    // 使用编辑器默认的构造和析构
    MiniJerk();
    ~MiniJerk() = default;

    // 没有经过平滑的轨迹
    nav_msgs::Path rough_path_;


private:
    // 定义节点句柄
    ros::NodeHandle nh_;

    //创建读写互斥锁
    std::shared_mutex mutex_;


    // 定义初始化函数
    void init();

    // 接收全局轨迹（未平滑的）,及其回调函数
    ros::Subscriber<nav_msgs::Path> path_sub_;
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);


    // 触发轨迹平滑的标志位
    bool need_smooth_flag_{false};

    // 发布经过多项式平滑后的轨迹,及其发布函数
    nav_msgs::Path smooth_path_;
    ros::Publisher path_pub_;
    void pathPub();

    // 创建MiniJerk算法类
    minijerk::MinijerkCore mini_jerk_;


    // 轨迹平滑处理，调用mini-jerk算法
    // 内部将实例化minijerk_core::MiniJerk类
    void smoothTrajectory(const nav_msgs::Path &rough_path, nav_msgs::Path &smooth_path);

};    
} // namespace name
