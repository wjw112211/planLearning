#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_sdf/SignedDistance2d.hpp>

#include "logManager/logManager.h"

// 代价取值沿用 ROS costmap_2d 的约定：
//   0                 自由空间
//   1 ... 252         膨胀代价，离障碍越近代价越大
//   253               内切膨胀障碍，紧贴障碍表面的栅格
//   254               致命障碍
//   未知（NaN）        未知区域，转回 OccupancyGrid 时表现为 -1
namespace {
const float kFreeSpace = 0.0f;
const float kInscribedInflatedObstacle = 253.0f;
const float kLethalObstacle = 254.0f;
}  // namespace

/**
 * 把 nav_msgs/OccupancyGrid 转换成代价地图。
 *
 * 输出两路内容一致、只是消息类型不同的代价地图：
 *   - grid_map_msgs/GridMap  ：occupancy（原始占据，基础图层）+ cost（代价）两个图层
 *   - nav_msgs/OccupancyGrid ：cost 图层按 [0, 254] -> [0, 100] 归一化，供 RViz 与规划器使用
 */
class OccupancyGridToCostMap
{
public:
  OccupancyGridToCostMap()
      : privateNodeHandle_("~")
  {
    privateNodeHandle_.param<std::string>("input_topic", inputTopic_, "/map");
    privateNodeHandle_.param<std::string>("grid_map_topic", gridMapTopic_, "/grid_map");
    privateNodeHandle_.param<std::string>("costmap_topic", costmapTopic_, "/costmap");
    privateNodeHandle_.param<std::string>("occupancy_layer", occupancyLayer_, "occupancy");
    privateNodeHandle_.param<std::string>("cost_layer", costLayer_, "cost");

    // 代价地图参数，含义与 costmap_2d 的 obstacle_layer / inflation_layer 一致。
    privateNodeHandle_.param("lethal_threshold", lethalThreshold_, 65.0);
    privateNodeHandle_.param("inscribed_radius", inscribedRadius_, 0.20);
    privateNodeHandle_.param("inflation_radius", inflationRadius_, 0.50);
    privateNodeHandle_.param("cost_scaling_factor", costScalingFactor_, 10.0);

    gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(gridMapTopic_, 1, true);
    costmapPublisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>(costmapTopic_, 1, true);
    mapSubscriber_ = nodeHandle_.subscribe(inputTopic_, 1,
                                           &OccupancyGridToCostMap::mapCallback, this);

    LOG(INFO) << "Building cost map: " << inputTopic_ << " -> " << gridMapTopic_
              << " and " << costmapTopic_ << " (lethal_threshold "
              << lethalThreshold_ << ", inscribed_radius " << inscribedRadius_
              << " m, inflation_radius " << inflationRadius_ << " m).";
  }

private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message)
  {
    // 1) 占据栅格转成 grid_map 的 occupancy 图层。
    grid_map::GridMap gridMap;
    if (!grid_map::GridMapRosConverter::fromOccupancyGrid(*message, occupancyLayer_, gridMap)) {
      // 地图消息会不停重发，这里按时间节流，避免同一条警告刷屏。
      LOG_EVERY_T(WARNING, 5.0) << "Could not convert occupancy grid to grid map.";
      return;
    }

    // 2) 由 occupancy 图层算出 cost 图层（致命障碍 + 膨胀代价）。
    computeCostLayer(gridMap);

    // 3) 发布 grid_map 形式的代价地图。
    grid_map_msgs::GridMap gridMapMessage;
    grid_map::GridMapRosConverter::toMessage(gridMap, gridMapMessage);
    gridMapPublisher_.publish(gridMapMessage);

    // 4) 发布 OccupancyGrid 形式的代价地图，交给 RViz 显示和规划器使用。
    nav_msgs::OccupancyGrid costmapMessage;
    grid_map::GridMapRosConverter::toOccupancyGrid(
        gridMap, costLayer_, kFreeSpace, kLethalObstacle, costmapMessage);
    costmapPublisher_.publish(costmapMessage);
  }

  // 在 occupancy 图层上计算膨胀代价，结果写入 cost 图层。
  void computeCostLayer(grid_map::GridMap& gridMap)
  {
    const grid_map::Matrix& occupancy = gridMap.get(occupancyLayer_);
    const float resolution = static_cast<float>(gridMap.getResolution());

    // 先把 occupancy 图层二值化为障碍标记：达到致命阈值的算障碍，其余算自由空间。
    const Eigen::Matrix<bool, -1, -1> obstacles = occupancy.unaryExpr(
        [this](float value) {
          return std::isfinite(value) && value >= lethalThreshold_;
        });

    // 每个栅格到最近障碍表面的距离（单位：米）。自由栅格为正、障碍内部为负。
    // 这里直接复用 grid_map_sdf 的精确欧氏距离变换（grid_map_ros 已经依赖 grid_map_sdf）。
    const grid_map::Matrix clearance =
        grid_map::signed_distance_field::signedDistanceFromOccupancy(obstacles, resolution);

    grid_map::Matrix cost(occupancy.rows(), occupancy.cols());
    for (Eigen::Index row = 0; row < occupancy.rows(); ++row) {
      for (Eigen::Index col = 0; col < occupancy.cols(); ++col) {
        cost(row, col) = computeCost(occupancy(row, col), clearance(row, col));
      }
    }

    gridMap.add(costLayer_, cost);
    gridMap.setBasicLayers({occupancyLayer_});
  }

  // 单个栅格的代价，衰减公式与 costmap_2d 的膨胀层保持一致。
  float computeCost(float occupancyValue, float clearance) const
  {
    // 未知区域保持未知。
    if (!std::isfinite(occupancyValue)) {
      return std::numeric_limits<float>::quiet_NaN();
    }

    // 致命障碍。
    if (occupancyValue >= lethalThreshold_) {
      return kLethalObstacle;
    }

    // clearance 为无穷表示整张地图没有障碍；超出膨胀半径的栅格都是自由空间。
    if (!std::isfinite(clearance) || clearance > inflationRadius_) {
      return kFreeSpace;
    }

    // 落在内切半径内（紧贴障碍）的栅格直接取内切膨胀代价。
    if (clearance <= inscribedRadius_) {
      return kInscribedInflatedObstacle;
    }

    // 内切半径到膨胀半径之间：代价随距离指数衰减。
    const double factor = std::exp(-costScalingFactor_ * (clearance - inscribedRadius_));
    float value = static_cast<float>(std::ceil((kInscribedInflatedObstacle - 1.0f) * factor));
    if (value < 1.0f) {
      value = kFreeSpace;
    }
    return value;
  }

  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;
  ros::Subscriber mapSubscriber_;
  ros::Publisher gridMapPublisher_;
  ros::Publisher costmapPublisher_;
  std::string inputTopic_;
  std::string gridMapTopic_;
  std::string costmapTopic_;
  std::string occupancyLayer_;
  std::string costLayer_;
  double lethalThreshold_;
  double inscribedRadius_;
  double inflationRadius_;
  double costScalingFactor_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_grid_to_cost_map");
  // 日志文件名固定为 mapping（log/mapping.INFO、log/mapping.WARNING ...），保留 7 天
  LOG_INIT("./log", 7, 6 * 3600, "mapping");

  OccupancyGridToCostMap converter;
  ros::spin();
  return 0;
}
