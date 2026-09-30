#include "minijerk_interface.h"


namespace minijerk
{
    // 构造函数 
    MiniJerkInterface::MiniJerkInterface()
    {
        // 初始化函数
        init();
    }


    // 初始化函数
    MiniJerkInterface::init()
    {
        // 创建订阅器，订阅没有经过平滑的轨迹
        path_sub_ = nh_.subscribe("/global_path", 1, &MiniJerkInterface::pathCallback, this);

        // 创建发布器，发布经过平滑的轨迹
        path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path_smooth", 1);
        
    }


    // 订阅粗糙轨迹的回调函数
    void MiniJerkInterface::pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        // 加入读写保护锁
        std::unique_lock<std::mutex> lock(mutex_);

        // 将粗糙轨迹软拷贝给rough_path_
        rough_path_ = *msg;
        
        // 触发轨迹平滑的标志
        path_smooth_flag_ = true;

        // 解锁, 打印订阅状态
        lock.unlock();
        LOG(INFO) << "receive rough path, size: " << rough_path_.poses.size();
    }


    // 轨迹平滑处理函数
    void smoothTrajectory(const nav_msgs::Path &rough_path, nav_msgs::Path &smooth_path)
    {
        /**
         * TODO： 实现轨迹平滑函数,并发布平滑轨迹。
         */

    }


    // 发布平滑轨迹
    void pathPub()
    {
        path_pub_.publish(smooth_path_);
    }


}