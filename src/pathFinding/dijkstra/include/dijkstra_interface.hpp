#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include "logManager/logManager.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "dijkstra_core.hpp"

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

    void processOnce();

    const geometry_msgs::Polygon& getPathMap() const;

private:
    // 创建节点句柄
    ros::NodeHandle nh_;

    // DijkstraCore实例
    DijkstraCore dijkstra_core_;

    // TF监听器
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 代价地图及其相关参数
    std::vector<int8_t> cost_map_; 
    int width_{0};
    int height_{0};
    double resolution_{0.0};
    double origin_x_{0.0};
    double origin_y_{0.0};

    // 当前规划结果
    geometry_msgs::Polygon path_map_;
    nav_msgs::Path path_msg_;

    // 话题和坐标系配置
    std::string costmap_topic_{"/costmap"};
    std::string goal_topic_{"/goal"};
    std::string rviz_goal_topic_{"/move_base_simple/goal"};
    std::string path_topic_{"/global_path"};
    std::string path_polygon_topic_{"/global_path_polygon"};
    std::string map_frame_{"map"};
    std::string base_frame_{"base_link"};

    // 路径发布者
    ros::Publisher path_pub_;
    ros::Publisher path_polygon_pub_;

    // 导航目标点的索引坐标
    int goal_index_x_{0};
    int goal_index_y_{0};

    //请求全局规划开始的标志位
    bool request_global_plan_{false};

    // 代价地图订阅者以及回调函数
    ros::Subscriber costmap_sub_;
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // 订阅导航目标点及其回调函数
    ros::Subscriber goal_sub_;
    void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

    // 订阅RViz 2D Nav Goal及其回调函数
    ros::Subscriber rviz_goal_sub_;
    void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void setGoalFromMapPosition(double goal_x, double goal_y);

    nav_msgs::Path convertPolygonToPath(const geometry_msgs::Polygon& polygon_path) const;

    // 从TF变换树中获取机器人在代价地图中的索引坐标
    bool getStartIndexFromTf(double origin_x,
                             double origin_y,
                             double resolution,
                             int& start_index_x,
                             int& start_index_y);

}; 
} // namespace dijkstra
