#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "logManager/logManager.h"
#include "pursuit_core.hpp"

namespace pursuit
{
class Pursuit
{
public:
    // 构造函数：参数从参数服务器读（launch 里加载的是 config/pursuit.yaml）
    Pursuit();
    ~Pursuit() = default;

    // 处理一次跟踪：按控制频率由节点主循环调用。
    // 拿不到位姿或者没有路径就发零速度停车（Gazebo 差速插件没有指令超时，
    // 不发指令机器人会一直按上一次的速度跑）
    void processOnce();

    // 控制频率，单位 Hz：节点主循环用它决定循环周期
    double getControlFrequency() const;

    // 最近一次收到的待跟踪路径
    nav_msgs::Path path_;

private:
    // 定义节点句柄
    ros::NodeHandle nh_;

    // 创建读写互斥锁，保护 interface 和 core 共享的路径数据
    std::shared_mutex mutex_;

    // 定义初始化函数
    void init();

    // 接收待跟踪的路径，及其回调函数
    ros::Subscriber path_sub_;
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);

    // 收到路径时检查一下路径本身平滑不平滑，不平滑就提示一次（只打日志，不改数据）
    void checkPathQuality(const nav_msgs::Path& path);

    // 速度指令发布
    ros::Publisher cmd_vel_pub_;
    void cmdVelPub(const geometry_msgs::Twist& cmd);

    // 前视点发布，方便在 RViz 里看纯追踪当前盯的是路径上的哪个点
    ros::Publisher lookahead_pub_;
    void lookaheadPub();

    // 跟踪算法类，构造函数里把本对象的读写锁交给它
    PursuitCore pursuit_core_;

    // TF 监听器：拿 map(世界系) -> base_link(车体) 的位姿，位姿里的朝向就是车头方向
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool getRobotPose(geometry_msgs::Pose2D& robot_pose);

    // 最近一次 TF 查询失败的原因，和停车日志一起打出来，方便排查
    std::string tf_error_;

    // 话题和坐标系配置
    std::string path_topic_{"/global_path_smooth"};
    std::string cmd_vel_topic_{"/cmd_vel"};
    std::string lookahead_point_topic_{"/pursuit/lookahead_point"};
    std::string map_frame_{"map"};
    std::string base_frame_{"base_link"};

    // 控制频率，单位 Hz
    double control_frequency_{20.0};

    // 前视距离参数：基准(m)、速度增益(s)、上下限(m)。
    // 基准别设太小：定位有噪声、路径点有抖动时，盯太近会把噪声放大成左右画龙；
    // 也别为了"更贴线"把它调大，前视距离越大切内弯越厉害（仿真里 0.4→0.5 让 0.5 m 半径
    // 弯道的横向偏差从 0.05 m 涨到 0.08 m）
    double lookahead_distance_{0.4};
    double lookahead_gain_{0.6};
    double min_lookahead_{0.2};
    double max_lookahead_{1.2};

    // 速度参数：最高线速度(m/s)、最低线速度(m/s)、最大角速度(rad/s)、
    // 最大角加速度(rad/s²，0 表示不做变化率限幅)
    double linear_speed_{0.22};
    double min_linear_speed_{0.05};
    double max_angular_speed_{1.5};
    double max_angular_acceleration_{4.0};

    // 终点减速距离(m)、到点判据(m)
    double deceleration_distance_{0.6};
    double goal_tolerance_{0.05};

    // 空闲（没有路径/已到达终点）时是否每拍都发零速度指令。
    // true 适合仿真，false 只在刚停下的那一拍发一次，之后不再抢 /cmd_vel，方便手动遥控接管
    bool publish_zero_when_idle_{true};

    // 上一拍是否在跟踪，用来判断"刚停下"的那一刻
    bool was_tracking_{false};

    // 是否收到过路径：没有路径时只提示一次"正在等路径"，不按控制频率刷警告
    bool path_received_{false};

    // 这一拍为什么没有在跟踪（正常跟踪时是 TRACKING）
    enum class StopReason
    {
        TRACKING,
        NO_POSE,
        NO_PATH,
        ARRIVED
    };
    StopReason stop_reason_{StopReason::TRACKING};

    // 停车状态只在"刚进入"时打一条日志，避免 20 Hz 重复打同样的内容
    void logStopOnce(StopReason reason);
};
} // namespace pursuit
