#include "pursuit_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pursuit
{
namespace
{
// 判断两个点是否重合、以及路径段是否可用的阈值，避免零长度线段带来除零
constexpr double kMinSegmentLength = 1e-6;

// 前视点落在身后超过这个角度（90°）就认为是"要掉头"。纯追踪的圆弧解在这个区间退化：
// alpha 接近 ±180° 时 sin(alpha) 接近 0，算出来的角速度也接近 0，机器人不转弯、
// 直着往前冲，越走越偏
constexpr double kTurnAroundAngle = M_PI_2;

// 掉头时的比例增益：omega = 增益 * alpha，再被最大角速度截断
constexpr double kTurnAroundGain = 1.0;

// 到点判据的迟滞倍数：进入容差就算到达，偏出 容差 * 这个倍数 才恢复跟踪
constexpr double kArrivalHysteresis = 3.0;
} // namespace

// 构造函数：引用接口层的读写锁
PursuitCore::PursuitCore(std::shared_mutex& mutex)
    : mutex_(mutex)
{
}

// 设置基准前视距离
void PursuitCore::setLookaheadDistance(double lookahead_distance)
{
    if(lookahead_distance <= kMinSegmentLength)
    {
        LOG(WARNING) << "invalid lookahead distance: " << lookahead_distance
                     << " (must be positive), keep " << lookahead_distance_;
        return;
    }

    lookahead_distance_ = lookahead_distance;
}

// 设置前视距离速度增益
void PursuitCore::setLookaheadGain(double lookahead_gain)
{
    if(lookahead_gain < 0.0)
    {
        LOG(WARNING) << "invalid lookahead gain: " << lookahead_gain
                     << " (must not be negative), keep " << lookahead_gain_;
        return;
    }

    lookahead_gain_ = lookahead_gain;
}

// 设置前视距离下限
void PursuitCore::setMinLookahead(double min_lookahead)
{
    if(min_lookahead <= kMinSegmentLength)
    {
        LOG(WARNING) << "invalid min lookahead: " << min_lookahead
                     << " (must be positive), keep " << min_lookahead_;
        return;
    }

    min_lookahead_ = min_lookahead;
}

// 设置前视距离上限
void PursuitCore::setMaxLookahead(double max_lookahead)
{
    if(max_lookahead < min_lookahead_)
    {
        LOG(WARNING) << "invalid max lookahead: " << max_lookahead
                     << " (should not be smaller than min lookahead " << min_lookahead_
                     << "), keep " << max_lookahead_;
        return;
    }

    max_lookahead_ = max_lookahead;
}

// 设置最高线速度
void PursuitCore::setLinearSpeed(double linear_speed)
{
    if(linear_speed < 0.0)
    {
        LOG(WARNING) << "invalid linear speed: " << linear_speed
                     << " (must not be negative), keep " << linear_speed_;
        return;
    }

    linear_speed_ = linear_speed;
}

// 设置最低线速度
void PursuitCore::setMinLinearSpeed(double min_linear_speed)
{
    if(min_linear_speed < 0.0)
    {
        LOG(WARNING) << "invalid min linear speed: " << min_linear_speed
                     << " (must not be negative), keep " << min_linear_speed_;
        return;
    }

    min_linear_speed_ = min_linear_speed;
}

// 设置最大角速度
void PursuitCore::setMaxAngularSpeed(double max_angular_speed)
{
    if(max_angular_speed <= 0.0)
    {
        LOG(WARNING) << "invalid max angular speed: " << max_angular_speed
                     << " (must be positive), keep " << max_angular_speed_;
        return;
    }

    max_angular_speed_ = max_angular_speed;
}

// 设置角速度变化率上限
void PursuitCore::setMaxAngularAcceleration(double max_angular_acceleration)
{
    if(max_angular_acceleration < 0.0)
    {
        LOG(WARNING) << "invalid max angular acceleration: " << max_angular_acceleration
                     << " (must not be negative), keep " << max_angular_acceleration_;
        return;
    }

    max_angular_acceleration_ = max_angular_acceleration;
}

// 设置控制周期
void PursuitCore::setControlPeriod(double control_period)
{
    if(control_period <= 0.0)
    {
        LOG(WARNING) << "invalid control period: " << control_period
                     << " (must be positive), keep " << control_period_;
        return;
    }

    control_period_ = control_period;
}

// 设置终点减速距离
void PursuitCore::setDecelerationDistance(double deceleration_distance)
{
    if(deceleration_distance < 0.0)
    {
        LOG(WARNING) << "invalid deceleration distance: " << deceleration_distance
                     << " (must not be negative), keep " << deceleration_distance_;
        return;
    }

    deceleration_distance_ = deceleration_distance;
}

// 设置到点判据
void PursuitCore::setGoalTolerance(double goal_tolerance)
{
    if(goal_tolerance < 0.0)
    {
        LOG(WARNING) << "invalid goal tolerance: " << goal_tolerance
                     << " (must not be negative), keep " << goal_tolerance_;
        return;
    }

    goal_tolerance_ = goal_tolerance;
}

// 接收待跟踪的路径
void PursuitCore::setPath(const nav_msgs::Path& path)
{
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        path_ = path;
    }

    // 新路径意味着重新开始跟：清掉上一段的到点标记和角速度限幅基准，
    // 否则新路径的终点如果正好落在上一段的到点容差里，会漏掉一次起步
    last_linear_speed_ = 0.0;
    last_angular_command_ = 0.0;
    arrived_latched_ = false;

    // 收到路径的日志由接口层打，算法层只管存数据
}

// 丢弃当前路径
void PursuitCore::resetPath()
{
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        path_.poses.clear();
        path_.header = std_msgs::Header();
    }

    last_linear_speed_ = 0.0;
    last_angular_command_ = 0.0;
    arrived_ = false;
    arrived_latched_ = false;
    distance_to_goal_ = 0.0;
    cross_track_error_ = 0.0;

    LOG(INFO) << "Reset path to follow.";
}

// 计算一次跟踪指令
bool PursuitCore::computeCommand(const geometry_msgs::Pose2D& robot_pose, geometry_msgs::Twist& cmd)
{
    // 先清零，后面任何一个"该停车"的分支返回时，指令都是零速度
    cmd = geometry_msgs::Twist();
    arrived_ = false;

    // 加锁拷贝一份路径，后面的计算都在锁外做，不长时间占用锁
    nav_msgs::Path path;
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        path = path_;
    }

    // 还没收到路径（节点刚起来、上游还没发）是正常状态：静默返回 false，
    // 由接口层提示一次"正在等路径"，不要按控制频率刷警告
    if(path.poses.empty())
    {
        distance_to_goal_ = 0.0;
        cross_track_error_ = 0.0;
        last_angular_command_ = 0.0; // 这一拍发的是零速度，限幅基准也归零
        return false;
    }

    // 1、把路径点取出来（平滑后的路径点间距很小，重复点已经在 convertPath 里丢掉）
    const std::vector<Point2d> points = convertPath(path);
    if(points.size() < 2)
    {
        // 收到路径了但用不了（只有一个点、或者所有点重合），这是上游的问题，按 1 Hz 提示
        LOG_EVERY_T(WARNING, 1.0) << "[PURSUIT FAILED] path is too short to follow, size="
                                  << points.size();
        last_angular_command_ = 0.0;
        return false;
    }

    const Point2d robot{robot_pose.x, robot_pose.y};
    const Point2d goal = points.back();
    const double distance_to_goal = std::hypot(goal.x - robot.x, goal.y - robot.y);

    // 记下来，停车日志里会打出来
    distance_to_goal_ = distance_to_goal;

    // 2、到达终点：进入容差就打 arrived 并锁定，偏出 3 倍容差才恢复跟踪。
    //    迟滞是必要的：没有它车会停在容差边界上，稍微蹭出去一点又开始起步，
    //    来回"起步-停车"，看起来就是终点附近反复晃
    if(!arrived_latched_ && distance_to_goal <= goal_tolerance_) arrived_latched_ = true;
    if(arrived_latched_ && distance_to_goal > kArrivalHysteresis * goal_tolerance_)
    {
        arrived_latched_ = false;
    }

    if(arrived_latched_)
    {
        arrived_ = true;
        last_linear_speed_ = 0.0;
        last_angular_command_ = 0.0; // 停住了，下一拍重新从 0 起算角速度

        // 这里不打日志：接口层在进入"已到达"状态时打一条就够了，1 Hz 重复打会刷屏
        return true;
    }

    // 3、自适应前视距离：速度越快看得越远（ld = 基准 + 增益 * 上一次线速度），
    //    再夹到 [min, max] 里。高速时盯太近会切内弯，低速时盯太远会贴着外圈跑
    const double lookahead = std::clamp(lookahead_distance_ + lookahead_gain_ * last_linear_speed_,
                                        min_lookahead_,
                                        max_lookahead_);
    const LookaheadResult lookahead_result = findLookaheadPoint(points, robot, lookahead);
    const Point2d lookahead_point = lookahead_result.point;

    // 机器人到路径的横向偏差：它在日志里能直接量化"跟得准不准"，
    // 也是判断左右画龙（偏差反复变号）还是稳态偏差（一直偏同一侧）的依据
    cross_track_error_ = lookahead_result.closest_distance;

    // 4、前视点在机器人坐标系下的位置，alpha 是它相对车头的夹角
    double local_x = 0.0;
    double local_y = 0.0;
    toRobotFrame(lookahead_point, robot, robot_pose.theta, local_x, local_y);

    const double lookahead_actual = std::hypot(local_x, local_y);
    if(lookahead_actual < kMinSegmentLength)
    {
        LOG_EVERY_T(WARNING, 1.0) << "[PURSUIT FAILED] lookahead point overlaps robot pose.";
        last_angular_command_ = 0.0;
        return false;
    }

    const double alpha = std::atan2(local_y, local_x);

    // 退化处理：前视点落在身后（|alpha| > 90°）时纯追踪的圆弧解不能用 —— alpha 接近
    // ±180° 时 sin(alpha) 接近 0，算出来的角速度也接近 0，机器人会不转弯、直着往前冲，
    // 离路径越来越远（日志里就是 alpha≈±3.07、w≈0.01、dist_to_goal 一直不降）。
    // 这种情况先原地掉头：线速度归零（不能边冲边转），只往 alpha 的符号方向给角速度，
    // 等前视点转到身前再恢复正常跟踪。转到 |alpha| = 90° 附近时角速度本身就是最大的，
    // 不会在两个分支之间来回跳
    if(std::fabs(alpha) > kTurnAroundAngle)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = std::clamp(kTurnAroundGain * alpha, -max_angular_speed_, max_angular_speed_);
        last_linear_speed_ = 0.0;
        last_angular_command_ = cmd.angular.z;

        storeLookaheadPoint(path.header, lookahead_point);

        LOG_EVERY_T(INFO, 1.0) << "[PURSUIT TURNING] lookahead point is behind the robot, turn in place: "
                               << "dist_to_goal=" << distance_to_goal
                               << ", alpha=" << alpha
                               << ", cmd=(v=0, w=" << cmd.angular.z << ")";
        return true;
    }

    // 5、纯追踪几何：曲率 kappa = 2 * sin(alpha) / ld，也就是让机器人走一段同时过当前
    //    位置和前视点的圆弧（半径 ld / (2 * sin(alpha))）。差速底盘用 omega = v * kappa
    //    就能走出这段圆弧，alpha 为正是左转
    const double curvature = 2.0 * std::sin(alpha) / lookahead_actual;

    const double linear = computeLinearSpeed(distance_to_goal);
    double angular = curvature * linear;

    // 6、角速度限幅：先按最大角速度截断（过急的弯不再提高角速度，相当于给最小转弯半径
    //    兜底），再按最大角加速度限制每拍的变化量。位姿（TF）本身有噪声、路径点也有抖动，
    //    不限变化率的话角速度会跟着噪声抖，车就在路径左右画龙
    angular = std::clamp(angular, -max_angular_speed_, max_angular_speed_);

    if(max_angular_acceleration_ > 0.0)
    {
        const double max_step = max_angular_acceleration_ * control_period_;
        angular = std::clamp(angular, last_angular_command_ - max_step, last_angular_command_ + max_step);
    }

    cmd.linear.x = linear;
    cmd.angular.z = angular;
    last_linear_speed_ = linear;
    last_angular_command_ = angular;

    storeLookaheadPoint(path.header, lookahead_point);

    // 跟踪过程按 1 Hz 打日志，控制频率下的每一拍都打会把日志刷爆。
    // cross_track 是机器人离路径多远（越小越好），反复变号说明在画龙；
    // lookahead_cmd 是这一拍用的前视距离，lookahead_actual 是机器人和前视点的实际距离
    LOG_EVERY_T(INFO, 1.0) << "[PURSUIT TRACKING] dist_to_goal=" << distance_to_goal
                           << ", cross_track=" << cross_track_error_
                           << ", lookahead_cmd=" << lookahead
                           << ", lookahead_actual=" << lookahead_actual
                           << ", alpha=" << alpha
                           << ", kappa=" << curvature
                           << ", cmd=(v=" << linear << ", w=" << angular << ")";

    return true;
}

// 记住这一拍的前视点，给 RViz 调试用；时间戳取当前时刻，避免 TF 过期显示不出来
void PursuitCore::storeLookaheadPoint(const std_msgs::Header& header, const Point2d& point)
{
    lookahead_point_.header = header;
    lookahead_point_.header.stamp = ros::Time::now();
    lookahead_point_.point.x = point.x;
    lookahead_point_.point.y = point.y;
    lookahead_point_.point.z = 0.0;
}

// 取上一次计算时机器人到路径终点的距离
double PursuitCore::getDistanceToGoal() const
{
    return distance_to_goal_;
}

// 取上一次计算时机器人到路径的横向偏差
double PursuitCore::getCrossTrackError() const
{
    return cross_track_error_;
}

// 取上一次计算的前视点
const geometry_msgs::PointStamped& PursuitCore::getLookaheadPoint() const
{
    return lookahead_point_;
}

// 上一次是否判定为到达终点
bool PursuitCore::arrived() const
{
    return arrived_;
}

// 把 nav_msgs::Path 里的点取出来，顺便丢掉重复点
std::vector<PursuitCore::Point2d> PursuitCore::convertPath(const nav_msgs::Path& path)
{
    std::vector<Point2d> points;
    points.reserve(path.poses.size());

    for(const geometry_msgs::PoseStamped& pose : path.poses)
    {
        Point2d point;
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;

        // 和上一个点重合的点直接丢掉：弧长累计和曲率都要求每段长度大于 0
        if(!points.empty())
        {
            const double step = std::hypot(point.x - points.back().x, point.y - points.back().y);
            if(step < kMinSegmentLength) continue;
        }

        points.push_back(point);
    }

    return points;
}

// 找前视点：返回前视点和机器人到路径的横向偏差
PursuitCore::LookaheadResult PursuitCore::findLookaheadPoint(const std::vector<Point2d>& points,
                                                             const Point2d& robot,
                                                             double lookahead)
{
    LookaheadResult result;
    const size_t last = points.size() - 1;

    // 先找离机器人最近的路径点。路径只有几百个点，线性搜索的耗时可以忽略
    size_t closest_index = 0;
    double min_distance = std::numeric_limits<double>::max();

    for(size_t i = 0; i <= last; ++i)
    {
        const double distance = std::hypot(points[i].x - robot.x, points[i].y - robot.y);
        if(distance < min_distance)
        {
            min_distance = distance;
            closest_index = i;
        }
    }

    // 机器人到最近路径点的距离，就是横向偏差（跟得准不准看它）。
    // 符号以最近点所在段的方向为基准：机器人在路径左侧为正、右侧为负，
    // 日志里一连串正负交替就是左右画龙，一直是同一个符号则更像标定/定位的固定偏差
    result.closest_distance = min_distance;

    const size_t sign_index = (closest_index < last) ? closest_index : last - 1;
    const double dir_x = points[sign_index + 1].x - points[sign_index].x;
    const double dir_y = points[sign_index + 1].y - points[sign_index].y;
    const double dir_length = std::hypot(dir_x, dir_y);

    if(dir_length > kMinSegmentLength)
    {
        const double cross = (dir_x * (robot.y - points[closest_index].y) -
                              dir_y * (robot.x - points[closest_index].x)) / dir_length;
        if(cross < 0.0) result.closest_distance = -min_distance;
    }

    // 机器人已经在最后一个点附近，直接以终点为前视点
    if(closest_index >= last)
    {
        result.point = points[last];
        return result;
    }

    // 从最近点沿路径向前累计弧长，到前视距离处就是前视点（近似从机器人在路径上的投影
    // 点算起）。只向前找：机器人横向偏差比较大时也不会把目标点选到身后来回摆头
    double remaining = lookahead;

    for(size_t i = closest_index; i < last; ++i)
    {
        const double segment = std::hypot(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y);
        if(segment < kMinSegmentLength) continue;

        if(remaining <= segment)
        {
            // 前视点落在这一段里，按比例插值，前视距离不会被离散点间距量化
            const double ratio = remaining / segment;

            result.point.x = points[i].x + ratio * (points[i + 1].x - points[i].x);
            result.point.y = points[i].y + ratio * (points[i + 1].y - points[i].y);
            return result;
        }

        remaining -= segment;
    }

    // 剩余路径比前视距离还短，说明前面已经是终点，直接盯终点
    result.point = points[last];
    return result;
}

// 把点转换到机器人坐标系：x 向前（车头方向），y 向左
void PursuitCore::toRobotFrame(const Point2d& point,
                               const Point2d& robot,
                               double yaw,
                               double& local_x,
                               double& local_y)
{
    const double dx = point.x - robot.x;
    const double dy = point.y - robot.y;

    // 二维旋转矩阵的转置（世界系 -> 机器人系）：yaw 是机器人在世界系里的朝向
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    local_x = cos_yaw * dx + sin_yaw * dy;
    local_y = -sin_yaw * dx + cos_yaw * dy;
}

// 终点附近减速
double PursuitCore::computeLinearSpeed(double distance_to_goal) const
{
    double speed = linear_speed_;

    // 快到终点时按剩余距离线性降速：v = v_max * d / d_decel。
    // 这样接近终点时速度足够小，配合到点判据能停下来，而不是以满速冲过去
    if(deceleration_distance_ > kMinSegmentLength && distance_to_goal < deceleration_distance_)
    {
        speed = linear_speed_ * distance_to_goal / deceleration_distance_;
    }

    // 先兜底最低速度（太低会被电机死区吃掉：指令在发但轮子不转），再夹住最高速度。
    // 参数配错（min > max）时以最高速度为准，至少不会冲出去
    speed = std::max(speed, min_linear_speed_);
    speed = std::min(speed, linear_speed_);

    return speed;
}

} // namespace pursuit

