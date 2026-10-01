#include "pursuit_interface.hpp"

#include <cmath>

namespace pursuit
{
// 构造函数：先把本对象的读写锁交给 core，再读参数、建订阅和发布。
// 初始化列表的顺序和成员声明顺序保持一致（core 在 TF 前面）
Pursuit::Pursuit()
    : pursuit_core_(mutex_),
      tf_listener_(tf_buffer_)
{
    init();
}

// 初始化函数
void Pursuit::init()
{
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("path_topic", path_topic_, path_topic_);
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
    private_nh.param<std::string>("lookahead_point_topic", lookahead_point_topic_, lookahead_point_topic_);
    private_nh.param<std::string>("map_frame", map_frame_, map_frame_);
    private_nh.param<std::string>("base_frame", base_frame_, base_frame_);
    private_nh.param<double>("control_frequency", control_frequency_, control_frequency_);
    private_nh.param<double>("lookahead_distance", lookahead_distance_, lookahead_distance_);
    private_nh.param<double>("lookahead_gain", lookahead_gain_, lookahead_gain_);
    private_nh.param<double>("min_lookahead", min_lookahead_, min_lookahead_);
    private_nh.param<double>("max_lookahead", max_lookahead_, max_lookahead_);
    private_nh.param<double>("linear_speed", linear_speed_, linear_speed_);
    private_nh.param<double>("min_linear_speed", min_linear_speed_, min_linear_speed_);
    private_nh.param<double>("max_angular_speed", max_angular_speed_, max_angular_speed_);
    private_nh.param<double>("max_angular_acceleration", max_angular_acceleration_, max_angular_acceleration_);
    private_nh.param<double>("deceleration_distance", deceleration_distance_, deceleration_distance_);
    private_nh.param<double>("goal_tolerance", goal_tolerance_, goal_tolerance_);
    private_nh.param<bool>("publish_zero_when_idle", publish_zero_when_idle_, publish_zero_when_idle_);

    // 前视距离参数传给算法层。先设上下限再设基准值：core 里上限要和下限比较，
    // yaml 里 min 配得比默认值大时也不会被误判成非法参数
    pursuit_core_.setMinLookahead(min_lookahead_);
    pursuit_core_.setMaxLookahead(max_lookahead_);
    pursuit_core_.setLookaheadDistance(lookahead_distance_);
    pursuit_core_.setLookaheadGain(lookahead_gain_);

    // 速度、减速和到点参数传给算法层
    pursuit_core_.setLinearSpeed(linear_speed_);
    pursuit_core_.setMinLinearSpeed(min_linear_speed_);
    pursuit_core_.setMaxAngularSpeed(max_angular_speed_);
    pursuit_core_.setMaxAngularAcceleration(max_angular_acceleration_);
    pursuit_core_.setDecelerationDistance(deceleration_distance_);
    pursuit_core_.setGoalTolerance(goal_tolerance_);

    // 控制周期（角速度变化率限幅要用它），控制频率非法时退回默认值，避免除零
    if(control_frequency_ > 0.0)
    {
        pursuit_core_.setControlPeriod(1.0 / control_frequency_);
    }
    else
    {
        LOG(WARNING) << "invalid control frequency: " << control_frequency_
                     << " (must be positive), use 20 Hz";
        control_frequency_ = 20.0;
        pursuit_core_.setControlPeriod(1.0 / control_frequency_);
    }

    // 创建订阅器，订阅待跟踪的路径，队列长度为1，消息无滞后
    path_sub_ = nh_.subscribe(path_topic_, 1, &Pursuit::pathCallback, this);

    // 创建发布器，发布速度指令；队列长度为1，过期的速度指令没有意义
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

    // 前视点 latching 住，RViz 后打开也能看到
    lookahead_pub_ = nh_.advertise<geometry_msgs::PointStamped>(lookahead_point_topic_, 1, true);

    LOG(INFO) << "pursuit init, path topic=" << path_topic_
              << ", cmd vel topic=" << cmd_vel_topic_
              << ", frame " << map_frame_ << " -> " << base_frame_
              << ", control frequency=" << control_frequency_
              << ", lookahead distance=" << lookahead_distance_
              << ", lookahead gain=" << lookahead_gain_
              << ", lookahead range=[" << min_lookahead_ << ", " << max_lookahead_ << "]"
              << ", linear speed=" << linear_speed_
              << ", min linear speed=" << min_linear_speed_
              << ", max angular speed=" << max_angular_speed_
              << ", max angular acceleration=" << max_angular_acceleration_
              << ", deceleration distance=" << deceleration_distance_
              << ", goal tolerance=" << goal_tolerance_
              << ", publish zero when idle=" << publish_zero_when_idle_;
}

// 订阅待跟踪路径的回调函数
void Pursuit::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    {
        std::unique_lock<std::shared_mutex> lock(mutex_); // 加锁，保护资源
        path_ = *msg;                                     // 把待跟踪的路径软拷贝给 path_
    }

    // 路径直接交给算法层（core 自己加锁保存），接口层这份拷贝只用于日志和排查
    pursuit_core_.setPath(*msg);

    // 之后就不再是"还没收到路径"的状态了
    path_received_ = true;

    LOG(INFO) << "receive path to follow, size: " << msg->poses.size();

    checkPathQuality(*msg);
}

// 收到路径时做一次质量检查（只打日志，不改数据）。
// 判据：相邻两段的转向方向反复变号的比例。平滑过的路径几乎不变号；
// 四连通栅格路径（dijkstra 没平滑的输出）几乎是每点一变号，车跟着走就一定左右摆。
// 这条日志能直接区分"车控制不好"和"路径本身就在摆"
void Pursuit::checkPathQuality(const nav_msgs::Path& path)
{
    const size_t size = path.poses.size();
    if(size < 3) return;

    int flips = 0;   // 转向方向变号的次数
    int turns = 0;   // 参与统计的转向次数
    int last_sign = 0;

    for(size_t i = 1; i + 1 < size; ++i)
    {
        const double ax = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
        const double ay = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
        const double bx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
        const double by = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;

        const double scale = std::hypot(ax, ay) * std::hypot(bx, by);
        if(scale < 1e-12) continue;

        // 叉乘的符号就是转向方向；转角小于 1° 当直行，不参与统计
        const double cross = ax * by - ay * bx;
        if(std::fabs(cross) / scale < 0.017) continue;

        const int sign = (cross > 0.0) ? 1 : -1;
        if(last_sign != 0)
        {
            ++turns;
            if(sign != last_sign) ++flips;
        }
        last_sign = sign;
    }

    if(turns >= 10 && flips > turns / 3)
    {
        LOG(WARNING) << "[PURSUIT PATH] received path looks jagged: " << flips << "/" << turns
                     << " vertices flip the turning direction, the robot will weave when "
                     << "following it. Check that path_topic is the smoothed path.";
    }
}

// 处理一次跟踪
void Pursuit::processOnce()
{
    geometry_msgs::Pose2D robot_pose;
    geometry_msgs::Twist cmd; // 默认全 0，也就是停车指令
    bool tracking = false;
    StopReason reason = StopReason::TRACKING;

    if(!getRobotPose(robot_pose))
    {
        // 拿不到位姿（TF 还没建好、定位丢了、坐标系名写错）是持续故障，按 1 Hz 提示
        LOG_EVERY_T(WARNING, 1.0) << "[PURSUIT STOP] can not get robot pose from tf ("
                                  << map_frame_ << " -> " << base_frame_ << "), stop. reason: "
                                  << tf_error_;
        reason = StopReason::NO_POSE;
    }
    else if(!pursuit_core_.computeCommand(robot_pose, cmd))
    {
        // 没有路径（还没收到 / 收到的路径点太少），core 里已经按 1 Hz 打了具体原因
        reason = StopReason::NO_PATH;
    }
    else if(pursuit_core_.arrived())
    {
        // 到终点了，cmd 仍然是零速度，机器人停在终点附近
        reason = StopReason::ARRIVED;
    }
    else
    {
        tracking = true;
        lookaheadPub();
    }

    logStopOnce(reason);

    // 跟踪中每拍都发指令；停车指令默认每拍都发（Gazebo 差速插件没有指令超时，
    // 不发就会一直按上一次的速度跑）。配成只在刚停下那一拍发一次时，之后不再抢
    // /cmd_vel，方便手动遥控接管
    if(tracking || publish_zero_when_idle_ || was_tracking_)
    {
        cmdVelPub(cmd);
    }

    was_tracking_ = tracking;
}

// 停车状态只在"刚进入"时打一条日志。
// 没收到路径和到达终点都是正常状态，按控制频率（默认 20 Hz）重复打会把日志刷爆，
// 这里只在状态发生变化时提示一次；取不到位姿是故障，已经由 1 Hz 限流日志负责，不重复打
void Pursuit::logStopOnce(StopReason reason)
{
    const bool just_entered = (reason != stop_reason_);
    stop_reason_ = reason;

    if(!just_entered) return;

    switch(reason)
    {
        case StopReason::NO_PATH:
            if(!path_received_)
            {
                LOG(INFO) << "[PURSUIT WAIT] no path received yet, waiting for path on topic "
                          << path_topic_ << ", stop.";
            }
            else
            {
                LOG(WARNING) << "[PURSUIT STOP] path is not usable, stop.";
            }
            break;

        case StopReason::ARRIVED:
            LOG(INFO) << "[PURSUIT STOP] arrived at the end of path, distance to goal="
                      << pursuit_core_.getDistanceToGoal() << " m, stop.";
            break;

        case StopReason::NO_POSE: // 已由 1 Hz 限流日志提示
        case StopReason::TRACKING: // 正常跟踪不打日志
        default:
            break;
    }
}

// 控制频率
double Pursuit::getControlFrequency() const
{
    return control_frequency_;
}

// 从 TF 里取机器人在 map 系下的位姿
bool Pursuit::getRobotPose(geometry_msgs::Pose2D& robot_pose)
{
    geometry_msgs::TransformStamped transform;

    try
    {
        // 取最新时刻的变换：map -> base_link
        transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0));
    }
    catch(const tf2::TransformException& ex)
    {
        // 原因存下来，由调用方和停车日志一起打，避免同一个故障打两遍日志
        tf_error_ = ex.what();
        return false;
    }

    // 纯追踪在平面上跟踪，只需要平移 x、y 和绕 z 轴的偏航角（车头方向）
    robot_pose.x = transform.transform.translation.x;
    robot_pose.y = transform.transform.translation.y;
    robot_pose.theta = tf2::getYaw(transform.transform.rotation);

    return true;
}

// 发布速度指令
void Pursuit::cmdVelPub(const geometry_msgs::Twist& cmd)
{
    cmd_vel_pub_.publish(cmd);
}

// 发布前视点，方便 RViz 调试
void Pursuit::lookaheadPub()
{
    lookahead_pub_.publish(pursuit_core_.getLookaheadPoint());
}

} // namespace pursuit
