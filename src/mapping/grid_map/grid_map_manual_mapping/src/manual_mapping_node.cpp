// 手动建图 / 定位节点（grid_map_manual_mapping）。
//
// 干什么：
//   * 默认进入"定位模式"：从 maps_dir 里挑最新的 *.pgm（配同名 *.yaml 元信息）加载，发到 /map。
//   * 收到 "mapping"（RViz 面板的"建图模式"按钮，或 rostopic pub）后清空地图，
//     用 /scan + tf 边走边建，同样发到 /map。
//   * 提供 /manual_mapping/save_map（std_srvs/Trigger）：把当前地图写成
//     maps/map_<时间戳>.pgm + .yaml（map_server 那套格式），并同时更新 latest.pgm/latest.yaml。
//   * 状态文字发到 /manual_mapping/status，RViz 面板上显示。
//
// 内部只维护一份栅格：hits（被打到几次）/ misses（被激光穿过几次），
// 发布前换算成 OccupancyGrid：-1 未知、0 空地、100 障碍。
// 后面接现有的 grid_map_gazebo_bridge（/map -> /grid_map + /costmap），所以 RViz 里能看代价地图。
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "logManager/logManager.h"

namespace
{

const char* const kModeMapping = "mapping";
const char* const kModeLocalization = "localization";

// 栅格计数上限，防止长时间建图时计数溢出
const float kMaxCount = 1.0e6f;

std::string trim(const std::string& text)
{
  size_t begin = 0;
  while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])))
  {
    ++begin;
  }
  size_t end = text.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
  {
    --end;
  }
  return text.substr(begin, end - begin);
}

std::string baseName(const std::string& path)
{
  const size_t pos = path.find_last_of('/');
  return pos == std::string::npos ? path : path.substr(pos + 1);
}

std::string stemName(const std::string& path)
{
  const std::string name = baseName(path);
  const size_t pos = name.find_last_of('.');
  return pos == std::string::npos ? name : name.substr(0, pos);
}

bool fileExists(const std::string& path)
{
  struct stat info;
  return ::stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

// 相当于 mkdir -p
bool ensureDirectory(const std::string& path)
{
  if (path.empty())
  {
    return false;
  }
  std::string current;
  for (size_t i = 0; i < path.size(); ++i)
  {
    current += path[i];
    if (path[i] == '/' || i + 1 == path.size())
    {
      if (current == "/")
      {
        continue;
      }
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
      {
        return false;
      }
    }
  }
  return true;
}

std::string timeStamp()
{
  const std::time_t now = std::time(nullptr);
  std::tm local;
  localtime_r(&now, &local);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local);
  return std::string(buffer);
}

}  // namespace

namespace
{

// ---------------------------------------------------------------------------
// pgm / yaml 读写（map_server 的格式：二值/三值图像 + 元信息 yaml）
// ---------------------------------------------------------------------------

// 写 P5 二进制 pgm。OccupancyGrid 的第 0 行是最小的 y（图像下边），
// 而 pgm 的第 0 行是图像上边，所以这里把行序翻过来写。
bool writePgm(const std::string& path, int width, int height, const std::vector<int8_t>& data)
{
  if (static_cast<int>(data.size()) != width * height)
  {
    return false;
  }
  std::ofstream out(path.c_str(), std::ios::binary);
  if (!out)
  {
    return false;
  }
  out << "P5\n" << width << " " << height << "\n255\n";
  for (int y = height - 1; y >= 0; --y)
  {
    for (int x = 0; x < width; ++x)
    {
      const int8_t value = data[static_cast<size_t>(y) * width + x];
      // negate=0 的约定：0 = 障碍（黑）、254 = 空地（白）、205 = 未知（灰）
      const char pixel = (value < 0) ? static_cast<char>(205)
                                     : static_cast<char>(value >= 50 ? 0 : 254);
      out.put(pixel);
    }
  }
  return out.good();
}

bool writeMapYaml(const std::string& path, const std::string& imageName, double resolution,
                  double originX, double originY, double negate, double occupiedThresh,
                  double freeThresh, const std::string& comment)
{
  std::ofstream out(path.c_str());
  if (!out)
  {
    return false;
  }
  out << std::setprecision(9);
  out << "# " << comment << "\n";
  out << "image: " << imageName << "\n";
  out << "resolution: " << resolution << "\n";
  out << "origin: [" << originX << ", " << originY << ", 0.0]\n";
  out << "negate: " << negate << "\n";
  out << "occupied_thresh: " << occupiedThresh << "\n";
  out << "free_thresh: " << freeThresh << "\n";
  return out.good();
}

// 读 pgm（支持 P5 二进制 / P2 文本；8 位和 16 位都认）
bool readPgm(const std::string& path, int& width, int& height, std::vector<uint8_t>& pixels,
             std::string& error)
{
  std::ifstream in(path.c_str(), std::ios::binary);
  if (!in)
  {
    error = "打不开 " + path;
    return false;
  }
  std::string magic;
  in >> magic;
  if (magic != "P5" && magic != "P2")
  {
    error = "只支持 P5/P2 格式的 pgm（当前是 " + magic + "）";
    return false;
  }
  const auto readNumber = [&in](int& value) -> bool {
    std::string token;
    while (in >> token)
    {
      if (!token.empty() && token[0] == '#')
      {
        std::getline(in, token);
        continue;
      }
      value = std::atoi(token.c_str());
      return true;
    }
    return false;
  };
  int maxValue = 0;
  if (!readNumber(width) || !readNumber(height) || !readNumber(maxValue) || width <= 0 ||
      height <= 0 || maxValue <= 0)
  {
    error = "pgm 文件头不完整";
    return false;
  }
  in.get();  // 吃掉 maxValue 后面的空白

  pixels.assign(static_cast<size_t>(width) * height, 0);
  if (magic == "P5")
  {
    if (maxValue > 255)
    {
      std::vector<char> raw(static_cast<size_t>(width) * height * 2, 0);
      in.read(&raw[0], static_cast<std::streamsize>(raw.size()));
      if (in.gcount() != static_cast<std::streamsize>(raw.size()))
      {
        error = "pgm 数据不完整";
        return false;
      }
      for (size_t i = 0; i < pixels.size(); ++i)
      {
        const unsigned char high = static_cast<unsigned char>(raw[2 * i]);
        const unsigned char low = static_cast<unsigned char>(raw[2 * i + 1]);
        const unsigned int value = (static_cast<unsigned int>(high) << 8) | low;
        pixels[i] = static_cast<uint8_t>(value * 255 / maxValue);
      }
    }
    else
    {
      in.read(reinterpret_cast<char*>(&pixels[0]), static_cast<std::streamsize>(pixels.size()));
      if (in.gcount() != static_cast<std::streamsize>(pixels.size()))
      {
        error = "pgm 数据不完整";
        return false;
      }
      if (maxValue != 255)
      {
        for (size_t i = 0; i < pixels.size(); ++i)
        {
          pixels[i] = static_cast<uint8_t>(pixels[i] * 255 / maxValue);
        }
      }
    }
  }
  else
  {
    for (size_t i = 0; i < pixels.size(); ++i)
    {
      int value = 0;
      if (!readNumber(value))
      {
        error = "P2 数据不完整";
        return false;
      }
      pixels[i] = static_cast<uint8_t>(value * 255 / maxValue);
    }
  }
  return true;
}

}  // namespace

class ManualMappingNode
{
public:
  ManualMappingNode()
      : nodeHandle_(),
        privateNodeHandle_("~"),
        tfListener_(tfBuffer_),  // 必须写在 tfBuffer_ 后面
        lastPublishTime_(),
        mapFrame_("map"),
        odomFrame_("odom"),
        mapsDir_("."),
        defaultMode_(kModeLocalization),
        mode_(kModeLocalization),
        resolution_(0.05),
        configResolution_(0.05),
        width_(200),
        height_(200),
        configWidth_(200),
        configHeight_(200),
        originX_(-5.0),
        originY_(-5.0),
        configOriginX_(-5.0),
        configOriginY_(-5.0),
        maxRange_(3.5),
        hitThreshold_(2),
        publishRate_(1.0),
        occupiedThresh_(0.65),
        freeThresh_(0.196),
        negate_(0.0),
        clearOnMappingStart_(true),
        publishMapToOdom_(true),
        integratedScans_(0),
        skippedScans_(0)
  {
    // ---------------- 参数 ----------------
    std::string modeTopic;
    std::string statusTopic;
    std::string saveServiceName;
    std::string scanTopic;
    std::string mapTopic;
    privateNodeHandle_.param<std::string>("mode_topic", modeTopic, "/manual_mapping/mode");
    privateNodeHandle_.param<std::string>("status_topic", statusTopic, "/manual_mapping/status");
    privateNodeHandle_.param<std::string>("save_service", saveServiceName, "/manual_mapping/save_map");
    privateNodeHandle_.param<std::string>("scan_topic", scanTopic, "/scan");
    privateNodeHandle_.param<std::string>("map_topic", mapTopic, "/map");
    privateNodeHandle_.param<std::string>("map_frame", mapFrame_, mapFrame_);
    privateNodeHandle_.param<std::string>("odom_frame", odomFrame_, odomFrame_);
    privateNodeHandle_.param<std::string>("maps_dir", mapsDir_, mapsDir_);
    privateNodeHandle_.param<std::string>("default_mode", defaultMode_, defaultMode_);
    privateNodeHandle_.param("resolution", configResolution_, configResolution_);
    privateNodeHandle_.param("width", configWidth_, configWidth_);
    privateNodeHandle_.param("height", configHeight_, configHeight_);
    privateNodeHandle_.param("origin_x", configOriginX_, configOriginX_);
    privateNodeHandle_.param("origin_y", configOriginY_, configOriginY_);
    privateNodeHandle_.param("max_range", maxRange_, maxRange_);
    privateNodeHandle_.param("hit_threshold", hitThreshold_, hitThreshold_);
    privateNodeHandle_.param("publish_rate", publishRate_, publishRate_);
    privateNodeHandle_.param("occupied_thresh", occupiedThresh_, occupiedThresh_);
    privateNodeHandle_.param("free_thresh", freeThresh_, freeThresh_);
    privateNodeHandle_.param("negate", negate_, negate_);
    privateNodeHandle_.param("clear_on_mapping_start", clearOnMappingStart_, clearOnMappingStart_);
    privateNodeHandle_.param("publish_map_to_odom", publishMapToOdom_, publishMapToOdom_);

    // ---------------- 接口 ----------------
    mapPublisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>(mapTopic, 1, true);
    statusPublisher_ = nodeHandle_.advertise<std_msgs::String>(statusTopic, 1, true);
    scanSubscriber_ = nodeHandle_.subscribe(scanTopic, 1, &ManualMappingNode::scanCallback, this);
    modeSubscriber_ = nodeHandle_.subscribe(modeTopic, 1, &ManualMappingNode::modeCallback, this);
    saveService_ =
        nodeHandle_.advertiseService(saveServiceName, &ManualMappingNode::saveMapCallback, this);

    if (publishMapToOdom_)
    {
      publishMapToOdom();
    }

    LOG(INFO) << "manual mapping node up: mode=" << defaultMode_ << ", maps_dir=" << mapsDir_
              << ", canvas=" << configWidth_ << "x" << configHeight_ << " @ " << configResolution_
              << " m, origin=(" << configOriginX_ << ", " << configOriginY_ << ")";

    setMode(defaultMode_, true);
  }

private:
  // -------------------------------------------------------------------------
  // 模式切换：RViz 面板按钮 / rostopic pub 都走这里
  // -------------------------------------------------------------------------
  void modeCallback(const std_msgs::String::ConstPtr& message)
  {
    setMode(trim(message->data), false);
  }

  void setMode(const std::string& requested, bool fromStartup)
  {
    if (requested != kModeMapping && requested != kModeLocalization)
    {
      LOG(WARNING) << "unknown mode '" << requested << "', 只认 " << kModeMapping << " / "
                   << kModeLocalization;
      publishStatus("未知模式 " + requested + "（只认 mapping / localization）");
      return;
    }
    const bool changed = (requested != mode_);
    mode_ = requested;
    LOG(INFO) << "mode -> " << mode_ << (changed ? "" : " (重复触发)")
              << (fromStartup ? " [启动默认]" : "");

    if (mode_ == kModeMapping)
    {
      startMapping(changed);
    }
    else
    {
      enterLocalization();
    }
  }

  // freshStart = true 表示"刚从别的模式切过来"（这时才清空地图，避免重复触发把地图清掉）
  void startMapping(bool freshStart)
  {
    if (freshStart && clearOnMappingStart_)
    {
      resetCanvas(configResolution_, configWidth_, configHeight_, configOriginX_, configOriginY_);
      integratedScans_ = 0;
      skippedScans_ = 0;
      LOG(INFO) << "建图模式：地图已清空（" << width_ << "x" << height_ << " @ " << resolution_
                << " m），用键盘 wsad 控制机器人开始建图";
      publishStatus("建图模式：地图已清空，用键盘 wsad 控制机器人走一圈");
    }
    else if (freshStart)
    {
      publishStatus("建图模式：在现有地图上继续建（clear_on_mapping_start=false）");
    }
    else
    {
      publishStatus("建图模式：已经在建图了（地图没被清空，想重来就切一次定位模式再切回来）");
    }
    refreshMapData();
    publishMap();
    lastPublishTime_ = ros::Time::now();
  }

  void enterLocalization()
  {
    std::string pgmPath;
    std::string yamlPath;
    if (findLatestMap(pgmPath, yamlPath) && loadMap(pgmPath, yamlPath))
    {
      return;
    }
    // 一张地图都没有：发一张全未知的图，别让 RViz 里空着
    resetCanvas(configResolution_, configWidth_, configHeight_, configOriginX_, configOriginY_);
    refreshMapData();
    publishMap();
    lastPublishTime_ = ros::Time::now();
    publishStatus("定位模式：maps/ 里还没有地图，先按建图模式建一张并保存");
    LOG(WARNING) << "定位模式：在 " << mapsDir_ << " 里没找到可用的 pgm + yaml";
  }

  // -------------------------------------------------------------------------
  // 建图：把一帧激光按机器人的位姿"刻"进栅格
  // -------------------------------------------------------------------------
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    if (mode_ != kModeMapping)
    {
      return;
    }

    geometry_msgs::TransformStamped sensorToMap;
    try
    {
      sensorToMap = tfBuffer_.lookupTransform(mapFrame_, scan->header.frame_id, scan->header.stamp,
                                              ros::Duration(0.1));
    }
    catch (const tf2::TransformException& exception)
    {
      ++skippedScans_;
      LOG_EVERY_T(WARNING, 5.0) << "跳过一帧激光（tf 不可用）: " << exception.what();
      return;
    }

    const double originX = sensorToMap.transform.translation.x;
    const double originY = sensorToMap.transform.translation.y;
    const double originYaw = tf2::getYaw(sensorToMap.transform.rotation);

    int originIx = 0;
    int originIy = 0;
    if (!worldToGrid(originX, originY, originIx, originIy))
    {
      ++skippedScans_;
      LOG_EVERY_T(WARNING, 5.0) << "跳过一帧激光（激光原点在图外）";
      return;
    }

    const float rangeMax = std::min<float>(scan->range_max, static_cast<float>(maxRange_));
    const float rangeMin = std::max<float>(scan->range_min, 0.0f);
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
      const float range = scan->ranges[i];
      if (!std::isfinite(range) || range < rangeMin || range >= rangeMax)
      {
        continue;
      }
      const double angle =
          originYaw + scan->angle_min + static_cast<double>(i) * scan->angle_increment;
      const double endX = originX + range * std::cos(angle);
      const double endY = originY + range * std::sin(angle);
      int endIx = 0;
      int endIy = 0;
      if (!worldToGrid(endX, endY, endIx, endIy))
      {
        continue;  // 打到画布外面，这条不要了
      }
      castRay(originIx, originIy, endIx, endIy);  // 沿途记成空地
      markOccupied(endIx, endIy);                 // 终点记成障碍
    }

    ++integratedScans_;
    // 顺便按 publish_rate 节流把 /map 发出去。
    // 注意：这里刻意不用 ros::Timer —— Timer 回调跑在定时器线程里，
    // 会和本回调（全局队列线程）并发读写 hits_/misses_，属于数据竞争。
    const ros::Time now = ros::Time::now();
    if (lastPublishTime_.isZero() ||
        (now - lastPublishTime_).toSec() >= 1.0 / std::max(0.1, publishRate_))
    {
      refreshMapData();
      publishMap();
      lastPublishTime_ = now;
    }
    if (integratedScans_ == 1 || integratedScans_ % 20 == 0)
    {
      LOG(INFO) << "已融合 " << integratedScans_ << " 帧激光（跳过 " << skippedScans_ << " 帧）";
    }
  }

  // 从 (x0,y0) 到 (x1,y1) 画 Bresenham 直线：沿途格子记"空"，终点除外（终点是墙）
  void castRay(int x0, int y0, int x1, int y1)
  {
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    const int stepX = (x0 < x1) ? 1 : -1;
    const int stepY = (y0 < y1) ? 1 : -1;
    int error = dx - dy;
    int x = x0;
    int y = y0;
    while (x != x1 || y != y1)
    {
      markFree(x, y);
      const int error2 = 2 * error;
      if (error2 > -dy)
      {
        error -= dy;
        x += stepX;
      }
      if (error2 < dx)
      {
        error += dx;
        y += stepY;
      }
    }
  }

  void markFree(int ix, int iy)
  {
    const size_t index = static_cast<size_t>(iy) * width_ + ix;
    if (misses_[index] < kMaxCount)
    {
      misses_[index] += 1.0f;
    }
  }

  void markOccupied(int ix, int iy)
  {
    const size_t index = static_cast<size_t>(iy) * width_ + ix;
    if (hits_[index] < kMaxCount)
    {
      hits_[index] += 1.0f;
    }
  }

  bool worldToGrid(double x, double y, int& ix, int& iy) const
  {
    ix = static_cast<int>(std::floor((x - originX_) / resolution_));
    iy = static_cast<int>(std::floor((y - originY_) / resolution_));
    return ix >= 0 && ix < width_ && iy >= 0 && iy < height_;
  }

  // -------------------------------------------------------------------------
  // 地图数据 / 发布
  // -------------------------------------------------------------------------
  void resetCanvas(double resolution, int width, int height, double originX, double originY)
  {
    resolution_ = resolution;
    width_ = width;
    height_ = height;
    originX_ = originX;
    originY_ = originY;
    hits_.assign(static_cast<size_t>(width_) * height_, 0.0f);
    misses_.assign(static_cast<size_t>(width_) * height_, 0.0f);
    map_.header.frame_id = mapFrame_;
    map_.info.resolution = resolution_;
    map_.info.width = static_cast<uint32_t>(width_);
    map_.info.height = static_cast<uint32_t>(height_);
    map_.info.origin.position.x = originX_;
    map_.info.origin.position.y = originY_;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    map_.data.assign(static_cast<size_t>(width_) * height_, -1);
  }

  // hits / misses 换算成 -1 / 0 / 100
  void refreshMapData()
  {
    map_.data.assign(hits_.size(), -1);
    for (size_t i = 0; i < hits_.size(); ++i)
    {
      if (hits_[i] >= static_cast<float>(hitThreshold_))
      {
        map_.data[i] = 100;
      }
      else if (misses_[i] > 0.0f)
      {
        map_.data[i] = 0;
      }
    }
  }

  void publishMap()
  {
    map_.header.stamp = ros::Time::now();
    map_.info.map_load_time = map_.header.stamp;
    mapPublisher_.publish(map_);
  }

  void publishStatus(const std::string& text)
  {
    lastStatus_ = text;
    std_msgs::String message;
    message.data = text;
    statusPublisher_.publish(message);
    LOG(INFO) << "status: " << text;
  }

  // map->odom 静态 tf：和原来 static_turtlebot3_world_map 的做法一样，
  // 让 RViz 的固定坐标系 map 能用，也让 costmap 里的地图和机器人对齐
  void publishMapToOdom()
  {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = mapFrame_;
    transform.child_frame_id = odomFrame_;
    transform.transform.rotation.w = 1.0;
    staticTfBroadcaster_.sendTransform(transform);
    LOG(INFO) << "已发布静态 tf: " << mapFrame_ << " -> " << odomFrame_;
  }

  struct MapStats
  {
    size_t freeCells;
    size_t occupiedCells;
    size_t unknownCells;
  };

  MapStats countCells() const
  {
    MapStats stats = {0, 0, 0};
    for (size_t i = 0; i < map_.data.size(); ++i)
    {
      if (map_.data[i] < 0)
      {
        ++stats.unknownCells;
      }
      else if (map_.data[i] >= 50)
      {
        ++stats.occupiedCells;
      }
      else
      {
        ++stats.freeCells;
      }
    }
    return stats;
  }

  // -------------------------------------------------------------------------
  // 保存地图（RViz 面板的"保存地图"按钮 -> /manual_mapping/save_map）
  // -------------------------------------------------------------------------
  bool saveMapCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
  {
    (void)request;
    if (mode_ == kModeMapping)
    {
      refreshMapData();
    }
    const MapStats stats = countCells();
    if (stats.occupiedCells == 0)
    {
      LOG(WARNING) << "当前地图里没有任何障碍格子，确定要保存吗？（可能还没开始建图）";
    }

    const std::string stamp = timeStamp();
    const std::string imageName = "map_" + stamp + ".pgm";
    const std::string pgmPath = mapsDir_ + "/" + imageName;
    const std::string yamlPath = mapsDir_ + "/map_" + stamp + ".yaml";
    const std::string comment = "由 grid_map_manual_mapping/manual_mapping_node 保存于 " + stamp;

    if (!ensureDirectory(mapsDir_) || !writePgm(pgmPath, width_, height_, map_.data) ||
        !writeMapYaml(yamlPath, imageName, resolution_, originX_, originY_, negate_, occupiedThresh_,
                      freeThresh_, comment))
    {
      response.success = false;
      response.message = "保存失败（目录不可写？）：" + pgmPath;
      LOG(ERROR) << response.message;
      publishStatus("保存地图失败：" + pgmPath);
      return true;
    }

    // latest.pgm / latest.yaml：给定位模式一个固定的"最新地图"入口
    writePgm(mapsDir_ + "/latest.pgm", width_, height_, map_.data);
    writeMapYaml(mapsDir_ + "/latest.yaml", "latest.pgm", resolution_, originX_, originY_, negate_,
                 occupiedThresh_, freeThresh_, "最新一张地图，每次保存都会覆盖");

    std::ostringstream text;
    text << "已保存 " << imageName << "（" << width_ << "x" << height_ << " @ " << resolution_
         << " m；障碍 " << stats.occupiedCells << " 格，空地 " << stats.freeCells << " 格，未知 "
         << stats.unknownCells << " 格）";
    response.success = true;
    response.message = text.str();
    LOG(INFO) << response.message << " -> " << mapsDir_;
    publishStatus("保存地图：" + response.message);
    return true;
  }

  // -------------------------------------------------------------------------
  // 定位：挑最新的 pgm + yaml 加载
  // -------------------------------------------------------------------------
  bool findLatestMap(std::string& pgmPath, std::string& yamlPath) const
  {
    DIR* directory = ::opendir(mapsDir_.c_str());
    if (directory == nullptr)
    {
      LOG(WARNING) << "打不开地图目录 " << mapsDir_;
      return false;
    }
    std::time_t newest = 0;
    std::string best;
    while (struct dirent* entry = ::readdir(directory))
    {
      const std::string name = entry->d_name;
      if (name.size() <= 4 || name.compare(name.size() - 4, 4, ".pgm") != 0)
      {
        continue;
      }
      const std::string candidate = mapsDir_ + "/" + name;
      if (!fileExists(mapsDir_ + "/" + stemName(name) + ".yaml"))
      {
        continue;  // 没有元信息 yaml 的 pgm 不认
      }
      struct stat info;
      if (::stat(candidate.c_str(), &info) != 0)
      {
        continue;
      }
      if (info.st_mtime >= newest)
      {
        newest = info.st_mtime;
        best = candidate;
      }
    }
    ::closedir(directory);
    if (best.empty())
    {
      return false;
    }
    pgmPath = best;
    yamlPath = mapsDir_ + "/" + stemName(best) + ".yaml";
    return true;
  }

  bool loadMap(const std::string& pgmPath, const std::string& yamlPath)
  {
    double resolution = configResolution_;
    double originX = configOriginX_;
    double originY = configOriginY_;
    double negate = negate_;
    double occupiedThresh = occupiedThresh_;
    double freeThresh = freeThresh_;
    std::string imageName = baseName(pgmPath);
    try
    {
      const YAML::Node root = YAML::LoadFile(yamlPath);
      if (root["resolution"])
      {
        resolution = root["resolution"].as<double>();
      }
      if (root["negate"])
      {
        negate = root["negate"].as<double>();
      }
      if (root["occupied_thresh"])
      {
        occupiedThresh = root["occupied_thresh"].as<double>();
      }
      if (root["free_thresh"])
      {
        freeThresh = root["free_thresh"].as<double>();
      }
      if (root["origin"] && root["origin"].IsSequence() && root["origin"].size() >= 2)
      {
        originX = root["origin"][0].as<double>();
        originY = root["origin"][1].as<double>();
      }
      if (root["image"])
      {
        imageName = root["image"].as<std::string>();
      }
    }
    catch (const YAML::Exception& exception)
    {
      LOG(ERROR) << "读 " << yamlPath << " 失败: " << exception.what();
      return false;
    }

    std::string imagePath = pgmPath;
    if (baseName(imageName) != baseName(pgmPath))
    {
      const std::string fromYaml = mapsDir_ + "/" + baseName(imageName);
      imagePath = fileExists(fromYaml) ? fromYaml : pgmPath;
    }

    int width = 0;
    int height = 0;
    std::vector<uint8_t> pixels;
    std::string error;
    if (!readPgm(imagePath, width, height, pixels, error))
    {
      LOG(ERROR) << error;
      return false;
    }

    resetCanvas(resolution, width, height, originX, originY);
    for (size_t i = 0; i < pixels.size(); ++i)
    {
      // pgm 第 0 行是图像上边（最大的 y），OccupancyGrid 第 0 行是最小的 y，所以要翻行
      const int imageRow = static_cast<int>(i) / width;
      const int imageCol = static_cast<int>(i) % width;
      const int mapIndex = (height - 1 - imageRow) * width + imageCol;
      const double occupancy = (negate != 0.0) ? static_cast<double>(pixels[i]) / 255.0
                                              : 1.0 - static_cast<double>(pixels[i]) / 255.0;
      if (occupancy > occupiedThresh)
      {
        map_.data[mapIndex] = 100;
      }
      else if (occupancy < freeThresh)
      {
        map_.data[mapIndex] = 0;
      }
      else
      {
        map_.data[mapIndex] = -1;
      }
    }

    // 把读进来的地图"灌"进计数：clear_on_mapping_start=false 时可以在这张图上继续建
    hits_.assign(pixels.size(), 0.0f);
    misses_.assign(pixels.size(), 0.0f);
    for (size_t i = 0; i < map_.data.size(); ++i)
    {
      if (map_.data[i] >= 50)
      {
        hits_[i] = static_cast<float>(hitThreshold_);
      }
      else if (map_.data[i] == 0)
      {
        misses_[i] = 1.0f;
      }
    }

    publishMap();
    lastPublishTime_ = ros::Time::now();
    const MapStats stats = countCells();
    std::ostringstream text;
    text << "定位模式：已加载 " << baseName(imagePath) << "（" << width << "x" << height << " @ "
         << resolution << " m，障碍 " << stats.occupiedCells << " 格，空地 " << stats.freeCells
         << " 格）";
    publishStatus(text.str());
    LOG(INFO) << text.str();
    return true;
  }

  // -------------------------------------------------------------------------
  // 成员
  // -------------------------------------------------------------------------
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::StaticTransformBroadcaster staticTfBroadcaster_;
  ros::Publisher mapPublisher_;
  ros::Publisher statusPublisher_;
  ros::Subscriber scanSubscriber_;
  ros::Subscriber modeSubscriber_;
  ros::ServiceServer saveService_;
  ros::Time lastPublishTime_;
  std::string mapFrame_;
  std::string odomFrame_;
  std::string mapsDir_;
  std::string defaultMode_;
  std::string mode_;
  std::string lastStatus_;
  double resolution_;
  double configResolution_;
  int width_;
  int height_;
  int configWidth_;
  int configHeight_;
  double originX_;
  double originY_;
  double configOriginX_;
  double configOriginY_;
  double maxRange_;
  int hitThreshold_;
  double publishRate_;
  double occupiedThresh_;
  double freeThresh_;
  double negate_;
  bool clearOnMappingStart_;
  bool publishMapToOdom_;
  long integratedScans_;
  long skippedScans_;
  std::vector<float> hits_;
  std::vector<float> misses_;
  nav_msgs::OccupancyGrid map_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manual_mapping_node");
  // 日志文件名固定成 mapping（log/mapping.INFO、log/mapping.WARNING ...），保留 7 天
  LOG_INIT("./log", 7, 6 * 3600, "mapping");

  ManualMappingNode node;
  ros::spin();
  return 0;
}
