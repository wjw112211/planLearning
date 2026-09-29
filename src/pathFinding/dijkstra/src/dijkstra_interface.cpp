#include "dijkstra_interface.hpp"

namespace dijkstra
{
DijkstraInterface::DijkstraInterface()
    : tf_listener_(tf_buffer_)
{
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("costmap_topic", costmap_topic_, costmap_topic_);
    private_nh.param<std::string>("goal_topic", goal_topic_, goal_topic_);
    private_nh.param<std::string>("rviz_goal_topic", rviz_goal_topic_, rviz_goal_topic_);
    private_nh.param<std::string>("path_topic", path_topic_, path_topic_);
    private_nh.param<std::string>("path_polygon_topic", path_polygon_topic_, path_polygon_topic_);
    private_nh.param<std::string>("map_frame", map_frame_, map_frame_);
    private_nh.param<std::string>("base_frame", base_frame_, base_frame_);

    //刚创建实例后，进行代价地图的的订阅，队列长度为1，订阅的消息无滞后
    costmap_sub_ = nh_.subscribe(costmap_topic_, 1, &DijkstraInterface::costmapCallback, this);
    // 订阅导航目标点，队列长度为1
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &DijkstraInterface::goalCallback, this);
    // 订阅RViz 2D Nav Goal，队列长度为1
    rviz_goal_sub_ = nh_.subscribe(rviz_goal_topic_, 1, &DijkstraInterface::rvizGoalCallback, this);
    // 发布规划出来的全局路径，nav_msgs::Path用于RViz直接显示
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
    // 保留发布Polygon格式路径，便于算法层调试
    path_polygon_pub_ = nh_.advertise<geometry_msgs::Polygon>(path_polygon_topic_, 1, true);
}


// 代价地图的回调函数
void DijkstraInterface::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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
void DijkstraInterface::goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    setGoalFromMapPosition(msg->x, msg->y);
}

// RViz 2D Nav Goal回调函数
void DijkstraInterface::rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    setGoalFromMapPosition(msg->pose.position.x, msg->pose.position.y);
}

void DijkstraInterface::setGoalFromMapPosition(double goal_x, double goal_y)
{
    int goal_index_x = 0;
    int goal_index_y = 0;

    {
        std::unique_lock<std::shared_mutex> lock(mutex_); // 加锁，保护资源
        if(resolution_ <= 0.0 || width_ <= 0 || height_ <= 0)
        {
            LOG(WARNING) << "Costmap is not ready, ignore goal.";
            return;
        }

        // 将目标点转换为代价地图的索引
        goal_index_x_ = static_cast<int>((goal_x - origin_x_) / resolution_);
        goal_index_y_ = static_cast<int>((goal_y - origin_y_) / resolution_);
        goal_index_x = goal_index_x_;
        goal_index_y = goal_index_y_;

        // 接受到导航目标点后，设置请求全局规划的标志位为true
        request_global_plan_ = true;
    }

    LOG(INFO) << "Received goal: (" << goal_x << ", " << goal_y << ")"
              << ", converted to index: (" << goal_index_x << ", " << goal_index_y << ")";
}

bool DijkstraInterface::getStartIndexFromTf(double origin_x,
                                            double origin_y,
                                            double resolution,
                                            int& start_index_x,
                                            int& start_index_y)
{
    geometry_msgs::TransformStamped transform;

    try 
    {
        transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0));

        // 机器人位置在map系下的坐标
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;

        // 机器人在代价地图中的索引坐标
        start_index_x = static_cast<int>((robot_x - origin_x) / resolution);
        start_index_y = static_cast<int>((robot_y - origin_y) / resolution);
        return true;
    }
    catch (const tf2::TransformException& ex) 
    {
        LOG(WARNING) << "Failed to get robot pose from tf: " << ex.what();
        return false;
    }
}


// 受到导航目标点之后，调用dijkstra_core开始搜路
void DijkstraInterface::processGlobalPlanRequest()
{
    bool need_plan = false;
    std::vector<int8_t> cost_map;
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    int start_index_x = 0;
    int start_index_y = 0;
    int goal_index_x = 0;
    int goal_index_y = 0;

    // 代码块，加锁保护对request_global_plan_的访问，但是不影响别处的代码
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        if(request_global_plan_) 
        {
            request_global_plan_ = false;
            need_plan = true;
        }

        if(need_plan)
        {
            cost_map = cost_map_;
            width = width_;
            height = height_;
            resolution = resolution_;
            origin_x = origin_x_;
            origin_y = origin_y_;
            goal_index_x = goal_index_x_;
            goal_index_y = goal_index_y_;
        }
    }

    if(!need_plan) return;
    LOG(INFO) << "[DIJKSTRA START]" << " , Start global planning.";

    if(!getStartIndexFromTf(origin_x, origin_y, resolution, start_index_x, start_index_y))
    {
        LOG(WARNING) << "[DIJKSTRA FAILED] Can not get start pose from tf.";
        return;
    }

    //dijkstra_core搜路
    DijkstraCore::Point start;
    start.x = start_index_x;
    start.y = start_index_y;

    DijkstraCore::Point goal;
    goal.x = goal_index_x;
    goal.y = goal_index_y;

    dijkstra_core_.setCostMap(cost_map, width, height, resolution, origin_x, origin_y);
    dijkstra_core_.setStartAndGoal(start, goal);

    bool plan_success = dijkstra_core_.makePlan();
    if(!plan_success)
    {
        LOG(WARNING) << "[DIJKSTRA FAILED] No valid path found.";
        return;
    }

    path_map_ = dijkstra_core_.getPathMap();
    path_msg_ = convertPolygonToPath(path_map_);
    path_pub_.publish(path_msg_);
    path_polygon_pub_.publish(path_map_);
    LOG(INFO) << "[DIJKSTRA SUCCESS] Path point size=" << path_map_.points.size();

}

nav_msgs::Path DijkstraInterface::convertPolygonToPath(const geometry_msgs::Polygon& polygon_path) const
{
    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = ros::Time::now();

    for(const geometry_msgs::Point32& point : polygon_path.points)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = point.z;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }

    return path;
}

void DijkstraInterface::processOnce()
{
    processGlobalPlanRequest();
}

const geometry_msgs::Polygon& DijkstraInterface::getPathMap() const
{
    return path_map_;
}

}
