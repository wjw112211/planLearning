#include <cmath>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

class OccupancyGridToGridMap
{
public:
  OccupancyGridToGridMap()
      : privateNodeHandle_("~")
  {
    privateNodeHandle_.param<std::string>("input_topic", inputTopic_, "/map");
    privateNodeHandle_.param<std::string>("output_topic", outputTopic_, "/grid_map");
    privateNodeHandle_.param<std::string>("occupancy_layer", occupancyLayer_, "occupancy");
    privateNodeHandle_.param<std::string>("flat_layer", flatLayer_, "flat");

    publisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(outputTopic_, 1, true);
    subscriber_ = nodeHandle_.subscribe(inputTopic_, 1,
                                        &OccupancyGridToGridMap::mapCallback, this);

    ROS_INFO_STREAM("Converting " << inputTopic_ << " to " << outputTopic_
                    << " using layer '" << occupancyLayer_ << "'.");
  }

private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message)
  {
    grid_map::GridMap gridMap;
    if (!grid_map::GridMapRosConverter::fromOccupancyGrid(
            *message, occupancyLayer_, gridMap)) {
      ROS_WARN_THROTTLE(5.0, "Could not convert occupancy grid to grid map.");
      return;
    }

    // Keep the map planar. The occupancy values remain in [0, 100].
    gridMap.add(flatLayer_, grid_map::Matrix::Zero(
        gridMap.getSize()(0), gridMap.getSize()(1)));
    gridMap.setBasicLayers({occupancyLayer_});

    grid_map_msgs::GridMap output;
    grid_map::GridMapRosConverter::toMessage(gridMap, output);
    publisher_.publish(output);
  }

  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string inputTopic_;
  std::string outputTopic_;
  std::string occupancyLayer_;
  std::string flatLayer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_grid_to_grid_map");
  OccupancyGridToGridMap converter;
  ros::spin();
  return 0;
}
