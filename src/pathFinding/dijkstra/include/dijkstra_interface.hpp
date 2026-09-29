#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdint>
#include <vector>
#include <shared_mutex>
#include "logManager/logManager.h"
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace dijkstra
{
class DijkstraInterface
{
public:
    DijkstraInterface();
    ~DijkstraInterface() = default;

    // 读写锁，保护多线程（interface和core）读取相同资源
    std::shared_mutex mutex_;


    // 受到导航目标点之后，调用dijkstra_core开始搜路
    void processGlobalPlanRequest();

private:
    // 创建节点句柄
    ros::NodeHandle nh_;

    // DijkstraCore实例
    DijkstraCore dijkstra_core_;

    // 代价地图及其相关参数
    std::vector<int8_t> cost_map_; 
    int width_{0};
    int height_{0};
    double resolution_{0.0};
    double origin_x_{0.0};
    double origin_y_{0.0};

    // 导航目标点的索引坐标
    int goal_index_x_{0};
    int goal_index_y_{0};

    // 机器人的起始位置索引
    int start_index_x_{0};
    int start_index_y_{0};

    //请求全局规划开始的标志位
    bool request_global_plan_{false};

    // 代价地图订阅者以及回调函数
    ros::Subscriber costmap_sub_;
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // 订阅导航目标点及其回调函数
    ros::Subscriber goal_sub_;
    void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

    //订阅TF变换树得到机器人在map系下的位姿及其回调函数
    ros::Subscriber tf_sub_;
    void tfCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

}; 
} // namespace dijkstra
