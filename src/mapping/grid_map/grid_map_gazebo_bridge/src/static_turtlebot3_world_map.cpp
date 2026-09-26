#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class StaticTurtlebot3WorldMap
{
public:
  StaticTurtlebot3WorldMap()
      : privateNodeHandle_("~")
  {
    privateNodeHandle_.param<std::string>("frame_id", frameId_, "map");
    privateNodeHandle_.param<std::string>("map_type", mapType_, "turtlebot3_world");
    privateNodeHandle_.param("resolution", resolution_, 0.05);
    privateNodeHandle_.param("width", width_, 200);
    privateNodeHandle_.param("height", height_, 200);
    privateNodeHandle_.param("origin_x", originX_, -5.0);
    privateNodeHandle_.param("origin_y", originY_, -5.0);
    privateNodeHandle_.param("publish_rate", publishRate_, 1.0);

    publisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    buildMap();
    publish();
    timer_ = nodeHandle_.createTimer(ros::Duration(1.0 / publishRate_),
                                     &StaticTurtlebot3WorldMap::timerCallback, this);

    ROS_INFO_STREAM("Publishing static " << mapType_ << " map on /map.");
  }

private:
  void buildMap()
  {
    map_.header.frame_id = frameId_;
    map_.info.resolution = resolution_;
    map_.info.width = width_;
    map_.info.height = height_;
    map_.info.origin.position.x = originX_;
    map_.info.origin.position.y = originY_;
    map_.info.origin.orientation.w = 1.0;
    map_.data.assign(width_ * height_, 0);

    if (mapType_ == "turtlebot3_world") {
      buildTurtlebot3World();
    } else if (mapType_ == "turtlebot3_square") {
      buildTurtlebot3Square();
    } else {
      ROS_WARN_STREAM("Unknown static map type '" << mapType_
                      << "'. Publishing an empty map.");
    }
  }

  void buildTurtlebot3World()
  {
    for (double x : {-1.1, 0.0, 1.1}) {
      for (double y : {-1.1, 0.0, 1.1}) {
        drawCircle(x, y, 0.18);
      }
    }

    drawCircle(3.5, 0.0, 0.55);
    drawCircle(1.8, 2.7, 0.38);
    drawCircle(1.8, -2.7, 0.38);
    drawCircle(-1.8, 2.7, 0.38);
    drawCircle(-1.8, -2.7, 0.38);
    drawRectangle(0.0, 0.0, 0.45, 4.2, -1.5708);
  }

  void buildTurtlebot3Square()
  {
    drawRectangle(0.0, 1.925, 4.0, 0.15, 0.0);
    drawRectangle(0.0, -1.925, 4.0, 0.15, 0.0);
    drawRectangle(-1.925, 0.0, 4.0, 0.15, -1.5708);
    drawRectangle(1.925, 0.0, 4.0, 0.15, 1.5708);
  }

  void drawCircle(double centerX, double centerY, double radius)
  {
    forEachCell([&](int index, double x, double y) {
      const double dx = x - centerX;
      const double dy = y - centerY;
      if (dx * dx + dy * dy <= radius * radius) {
        map_.data[index] = 100;
      }
    });
  }

  void drawRectangle(double centerX, double centerY, double lengthX, double lengthY,
                     double yaw)
  {
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    forEachCell([&](int index, double x, double y) {
      const double dx = x - centerX;
      const double dy = y - centerY;
      const double localX = c * dx - s * dy;
      const double localY = s * dx + c * dy;
      if (std::abs(localX) <= 0.5 * lengthX &&
          std::abs(localY) <= 0.5 * lengthY) {
        map_.data[index] = 100;
      }
    });
  }

  template <typename Callback>
  void forEachCell(Callback callback)
  {
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int index = y * width_ + x;
        const double worldX = originX_ + (x + 0.5) * resolution_;
        const double worldY = originY_ + (y + 0.5) * resolution_;
        callback(index, worldX, worldY);
      }
    }
  }

  void timerCallback(const ros::TimerEvent&)
  {
    publish();
  }

  void publish()
  {
    map_.header.stamp = ros::Time::now();
    map_.info.map_load_time = map_.header.stamp;
    publisher_.publish(map_);
  }

  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  nav_msgs::OccupancyGrid map_;
  std::string frameId_;
  std::string mapType_;
  double resolution_;
  double originX_;
  double originY_;
  double publishRate_;
  int width_;
  int height_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_turtlebot3_world_map");
  StaticTurtlebot3WorldMap staticMap;
  ros::spin();
  return 0;
}
