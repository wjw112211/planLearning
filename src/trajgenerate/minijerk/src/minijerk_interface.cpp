#include "minijerk_interface.hpp"

namespace minijerk
{
// 构造函数：先把本对象的读写锁交给 core，再读参数、建订阅和发布
MiniJerk::MiniJerk()
    : mini_jerk_(mutex_)
{
    init();
}

// 初始化函数
void MiniJerk::init()
{
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("rough_path_topic", rough_path_topic_, rough_path_topic_);
    private_nh.param<std::string>("smooth_path_topic", smooth_path_topic_, smooth_path_topic_);
    private_nh.param<std::string>("map_frame", map_frame_, map_frame_);
    private_nh.param<double>("sample_interval", sample_interval_, sample_interval_);
    private_nh.param<double>("feature_epsilon", feature_epsilon_, feature_epsilon_);
    private_nh.param<double>("fidelity_weight", fidelity_weight_, fidelity_weight_);

    // 采样间隔、特征点阈值和保真项权重传给算法层
    mini_jerk_.setSampleInterval(sample_interval_);
    mini_jerk_.setFeatureEpsilon(feature_epsilon_);
    mini_jerk_.setFidelityWeight(fidelity_weight_);

    // 创建订阅器，订阅没有经过平滑的轨迹，队列长度为1，消息无滞后
    path_sub_ = nh_.subscribe(rough_path_topic_, 1, &MiniJerk::pathCallback, this);

    // 创建发布器，发布经过平滑的轨迹，latch住最后一帧方便RViz随时显示
    path_pub_ = nh_.advertise<nav_msgs::Path>(smooth_path_topic_, 1, true);

    LOG(INFO) << "minijerk init, rough path topic=" << rough_path_topic_
              << ", smooth path topic=" << smooth_path_topic_
              << ", sample interval=" << sample_interval_
              << ", feature epsilon=" << feature_epsilon_
              << ", fidelity weight=" << fidelity_weight_;
}

// 订阅粗糙轨迹的回调函数
void MiniJerk::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    {
        std::unique_lock<std::shared_mutex> lock(mutex_); // 加锁，保护资源
        rough_path_ = *msg;                               // 将粗糙轨迹软拷贝给rough_path_
        need_smooth_flag_ = true;                         // 触发轨迹平滑的标志
    }

    LOG(INFO) << "receive rough path, size: " << msg->poses.size();
}

// 处理一次平滑请求
void MiniJerk::processOnce()
{
    nav_msgs::Path rough_path;

    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        // 没有新轨迹直接返回；有的话清掉标志并把轨迹拷贝出来。
        // 平滑本身比较耗时，放到锁外做，避免阻塞订阅回调
        if(!need_smooth_flag_) return;

        need_smooth_flag_ = false;
        rough_path = rough_path_;
    }

    if(!smoothTrajectory(rough_path, smooth_path_))
    {
        LOG(WARNING) << "[MINIJERK FAILED] smooth trajectory failed, rough point size="
                     << rough_path.poses.size();
        return;
    }

    pathPub();

    LOG(INFO) << "[MINIJERK SUCCESS] rough point size=" << rough_path.poses.size()
              << ", smooth point size=" << smooth_path_.poses.size();
}

// 轨迹平滑处理函数
bool MiniJerk::smoothTrajectory(const nav_msgs::Path& rough_path, nav_msgs::Path& smooth_path)
{
    // 把待平滑的轨迹交给算法层（core和本对象共用一把读写锁）
    mini_jerk_.getTrajectory(rough_path);

    // 采样 -> 选特征点 -> mini-jerk 求解
    if(!mini_jerk_.smooth(smooth_path)) return false;

    // 粗糙轨迹没带坐标系时用参数里的坐标系，保证RViz能显示
    if(smooth_path.header.frame_id.empty()) smooth_path.header.frame_id = map_frame_;

    return true;
}

// 发布平滑轨迹
void MiniJerk::pathPub()
{
    path_pub_.publish(smooth_path_);
}

} // namespace minijerk
