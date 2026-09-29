#include "dijkstra_interface.hpp"

namespace dijkstra
{
DijkstraInterface::DijkstraInterface()
{
    //刚创建实例后，进行代价地图的的订阅，队列长度为1，订阅的消息无滞后
    costmap_sub_ = nh_.subscribe("/costmap", 1, &DijkstraInterface::costmapCallback, this);
    // 订阅导航目标点，队列长度为1
    goal_sub_ = nh_.subscribe("/goal", 1, &DijkstraInterface::goalCallback, this);
    // 订阅TF变换树，获取机器人在map系下的位姿，队列长度为1
    tf_sub_ = nh_.subscribe("/tf", 1, &DijkstraInterface::tfCallback, this);
}


// 代价地图的回调函数
DijkstraInterface::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::unique_lock<std::shared_mutex> lock(mutex_); // 加锁，保护资源
    width_ = static_cast<int>(msg->info.width);
    height_ = static_cast<int>(msg->info.height); // 代价地图的长度和宽度是索引形式
    resolution_ = msg->info.resolution;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    cost_map_ = msg->data; // 拷贝代价地图的数据

    lock.unlock(); // 解锁进行日志打印
    LOG(INFO) << "Received costmap: width=" << width_ << ", height=" << height_ 
              << ", resolution=" << resolution_ << ", origin=(" << origin_x_ << "," << origin_y_ << ")"
              << ", cost_map size=" << cost_map_.size();
}

// 导航目标点的回调函数
DijkstraInterface::goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    std::unique_lock<std::shared_mutex> lock(mutex_); // 加锁，保护资源
    double goal_x = msg->x;
    double goal_y = msg->y;

    // 将目标点转换为代价地图的索引
    goal_index_x_ = static_cast<int>((goal_x - origin_x_) / resolution_);
    goal_index_y_ = static_cast<int>((goal_y - origin_y_) / resolution_);

    // 接受到导航目标点后，设置请求全局规划的标志位为true
    request_global_plan_ = true;

    lock.unlock(); // 解锁进行日志打印

    LOG(INFO) << "Received goal: (" << goal_x << ", " << goal_y << ")"
              << ", converted to index: (" << goal_index_x << ", " << goal_index_y << ")";
}

// tf回调函数
DijkstraInterface::tfCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;

    try 
    {
        transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));

        // 机器人位置在map系下的坐标
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;

        // 机器人在代价地图中的索引坐标
        start_index_x_ = static_cast<int>((robot_x - origin_x_) / resolution_);
        start_index_y_ = static_cast<int>((robot_y - origin_y_) / resolution_);
    }
    catch (const tf2::TransformException& ex) 
    {
        LOG(WARNING) << "Failed to get robot pose from tf: " << ex.what();
        return;
    }
}


// 受到导航目标点之后，调用dijkstra_core开始搜路
void DijkstraInterface::processGlobalPlanRequest()
{
    bool need_plan = false;

    // 代码块，加锁保护对request_global_plan_的访问，但是不影响别处的代码
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        if(request_global_plan_) 
        {
            request_global_plan_ = false;
            need_plan = true;
        }
    }

    if(!need_plan) return;
    LOG(INFO) << "[DIJKSTRA START]" << " , Start global planning.";

    //dijkstra_core搜路

}

}
