#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <shared_mutex>
#include <string>
#include "logManager/logManager.h"
#include "minijerk_core.hpp"

namespace minijerk
{
class MiniJerk
{
public:
    // 使用编辑器默认的析构函数
    MiniJerk();
    ~MiniJerk() = default;

    // 处理一次平滑请求：有新的粗糙轨迹就平滑并发布。
    // 由节点主循环按固定频率调用，耗时操作放在锁外，不阻塞订阅回调
    void processOnce();

    // 没有经过平滑的轨迹
    nav_msgs::Path rough_path_;

private:
    // 定义节点句柄
    ros::NodeHandle nh_;

    // 创建读写互斥锁，保护 interface 和 core 共享的轨迹数据
    std::shared_mutex mutex_;

    // 定义初始化函数
    void init();

    // 接收全局轨迹（未平滑的），及其回调函数
    ros::Subscriber path_sub_;
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);

    // 触发轨迹平滑的标志位
    bool need_smooth_flag_{false};

    // 经过平滑后的轨迹，及其发布
    nav_msgs::Path smooth_path_;
    ros::Publisher path_pub_;
    void pathPub();

    // 创建MiniJerk算法类，构造函数里把本对象的读写锁交给它
    MinijerkCore mini_jerk_;

    // 轨迹平滑处理，调用mini-jerk算法
    // 内部先取出待平滑的轨迹，再实例化minijerk_core::MinijerkCore做采样和平滑
    bool smoothTrajectory(const nav_msgs::Path& rough_path, nav_msgs::Path& smooth_path);

    // 话题和坐标系配置
    std::string rough_path_topic_{"/global_path"};
    std::string smooth_path_topic_{"/global_path_smooth"};
    std::string map_frame_{"map"};

    // 算法参数：采样间隔（m）、特征点阈值（m）、保真项权重
    double sample_interval_{0.1};
    double feature_epsilon_{0.05};
    double fidelity_weight_{1.0};
};
} // namespace minijerk
