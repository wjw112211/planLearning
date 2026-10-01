#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include "logManager/logManager.h"

namespace pursuit
{
// 纯追踪（Pure Pursuit）算法核心：只做"给定路径 + 机器人位姿 -> 速度指令"这件事，
// 不碰 ROS 话题、TF 和参数服务器，分层方式沿用 trajgenerate/minijerk
class PursuitCore
{
public:
    // 构造函数：引用接口层的读写锁，core 和 interface 共用同一份路径数据
    explicit PursuitCore(std::shared_mutex& mutex);
    ~PursuitCore() = default;

    // ---------- 前视距离参数（单位见 config/pursuit.yaml 注释） ----------

    // 基准前视距离，单位 m：机器人速度接近 0 时盯多远的点
    void setLookaheadDistance(double lookahead_distance);

    // 前视距离随速度的增益，单位 s：ld = lookahead_distance + gain * 上一次线速度
    void setLookaheadGain(double lookahead_gain);

    // 前视距离下限、上限，单位 m：算出来的 ld 夹在这个区间里
    void setMinLookahead(double min_lookahead);
    void setMaxLookahead(double max_lookahead);

    // ---------- 速度参数 ----------

    // 最高线速度，单位 m/s
    void setLinearSpeed(double linear_speed);

    // 最低线速度，单位 m/s：转弯和快到终点时都不会低于它，0 表示允许完全停下
    void setMinLinearSpeed(double min_linear_speed);

    // 最大角速度，单位 rad/s：纯追踪算出来的角速度超过它就截断，相当于限制最小转弯半径
    void setMaxAngularSpeed(double max_angular_speed);

    // 角速度变化率上限，单位 rad/s²：指令角速度每拍最多变化这么多，用来滤掉位姿噪声、
    // 路径点抖动引起的左右画龙；0 表示不做限制
    void setMaxAngularAcceleration(double max_angular_acceleration);

    // 控制周期，单位 s：角速度变化率限幅要用它换算成"每拍允许变化多少"
    void setControlPeriod(double control_period);

    // 终点减速距离，单位 m：离终点小于这个距离就开始按比例减速
    void setDecelerationDistance(double deceleration_distance);

    // 到点判据，单位 m：机器人离路径最后一个点小于这个距离就算到达
    void setGoalTolerance(double goal_tolerance);

    // 接收待跟踪的路径（加写锁保存，线程安全）
    void setPath(const nav_msgs::Path& path);

    // 丢弃当前路径，之后 computeCommand 会返回 false（调用方发零速度）
    void resetPath();

    // 计算一次跟踪指令：找前视点 -> 算曲率 -> 算线速度/角速度。
    // 返回 false 表示没有可用路径，调用方应该发零速度停车；
    // 返回 true 时 cmd 是这一拍的指令，arrived() 表示机器人已经到达路径终点
    bool computeCommand(const geometry_msgs::Pose2D& robot_pose, geometry_msgs::Twist& cmd);

    // 上一次计算用的前视点，用于在 RViz 里看算法当前盯的是路径上的哪个点
    const geometry_msgs::PointStamped& getLookaheadPoint() const;

    // 上一次计算时机器人到路径终点的距离，单位 m（停车日志里会打出来）
    double getDistanceToGoal() const;

    // 上一次计算时机器人到路径的横向偏差（离路径最近点的距离），单位 m
    double getCrossTrackError() const;

    // 上一次 computeCommand 是否判定为"已到达终点"
    bool arrived() const;

private:
    // 平面点，跟踪只关心 x、y
    struct Point2d
    {
        double x{0.0};
        double y{0.0};
    };

    // 接口层的读写锁
    std::shared_mutex& mutex_;

    // 待跟踪的路径
    nav_msgs::Path path_;

    // 基准前视距离，单位 m
    double lookahead_distance_{0.4};

    // 前视距离随速度的增益，单位 s
    double lookahead_gain_{0.6};

    // 前视距离上下限，单位 m
    double min_lookahead_{0.2};
    double max_lookahead_{1.2};

    // 最高线速度、最低线速度，单位 m/s
    double linear_speed_{0.22};
    double min_linear_speed_{0.05};

    // 最大角速度，单位 rad/s
    double max_angular_speed_{1.5};

    // 角速度变化率上限（rad/s²）和控制周期（s），0 表示不限幅
    double max_angular_acceleration_{4.0};
    double control_period_{0.05};

    // 终点减速距离、到点判据，单位 m
    double deceleration_distance_{0.6};
    double goal_tolerance_{0.05};

    // 上一次下发的线速度，用来算自适应前视距离。
    // 这里没有订阅里程计，用上一次指令做近似，速度指令本身就是连续变化的，够用
    double last_linear_speed_{0.0};

    // 上一次计算时机器人到路径终点的距离，单位 m
    double distance_to_goal_{0.0};

    // 上一次计算时机器人到路径最近点的距离，单位 m，也就是横向偏差（机器人偏离路径多远）
    double cross_track_error_{0.0};

    // 上一次下发的角速度，用来做角速度变化率限幅
    double last_angular_command_{0.0};

    // 到点判据的迟滞：一旦判定到达就保持住，只有偏出容差的 kArrivalHysteresis 倍才恢复跟踪
    bool arrived_latched_{false};

    // 上一次计算出来的前视点、是否到达终点
    geometry_msgs::PointStamped lookahead_point_;
    bool arrived_{false};

    // 1、把 nav_msgs::Path 里的点取出来，顺便丢掉重复点
    static std::vector<Point2d> convertPath(const nav_msgs::Path& path);

    // 2、找前视点：从离机器人最近的路径点出发沿路径向前累计弧长，到前视距离处取点。
    //    同时给出机器人到路径的横向偏差（离最近路径点的距离），日志和抗画龙减速都要用
    struct LookaheadResult
    {
        Point2d point;                // 前视点
        double closest_distance{0.0}; // 机器人到路径最近点的距离，即横向偏差
    };

    static LookaheadResult findLookaheadPoint(const std::vector<Point2d>& points,
                                              const Point2d& robot,
                                              double lookahead);

    // 3、把点转换到机器人坐标系：x 向前（车头方向），y 向左
    static void toRobotFrame(const Point2d& point,
                             const Point2d& robot,
                             double yaw,
                             double& local_x,
                             double& local_y);

    // 4、终点附近按剩余距离线性减速，结果夹在 [min_linear_speed_, linear_speed_] 内
    double computeLinearSpeed(double distance_to_goal) const;

    // 5、记住这一拍的前视点，给 RViz 调试用（跟踪和原地掉头两个分支都要记）
    void storeLookaheadPoint(const std_msgs::Header& header, const Point2d& point);
};
} // namespace pursuit
