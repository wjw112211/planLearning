# 代价地图障碍物层代码教学（第 1 步）

> 对应启动文件：`src/bringup/launch/grid_map.launch`
>
> 目标：在现有“静态地图 → 代价地图”链路上，**新增一个节点**，把 `/scan` 的传感器数据动态写进代价地图。
>
> 本文只讲清“加在哪、怎么写”，不直接修改工程代码，由你按步骤自己改。

---

## 一、先确认现在缺什么

`grid_map.launch` 目前的代价地图链路是：

```
static_map_loader (/map, 静态 pgm)
        │
        ▼
occupancy_grid_to_cost_map  (grid_map_gazebo_bridge)
        │   mapCallback()          src/.../occupancy_grid_to_cost_map.cpp:75-98
        │   computeCostLayer()     :101-126   ← 只对静态 occupancy 做膨胀
        ▼
/grid_map  +  /costmap    →  RViz、dijkstra
```

`occupancy_grid_to_cost_map` 只订阅 `/map`（`input_topic`），**完全没有订阅 `/scan` 和 TF**，所以障碍物永远是静态图里的那一份，机器人前方新出现的箱子、行人不会进代价地图。

而建图时之所以能进，是因为 `manual_mapping_node` 自己订阅了 `/scan` 并做了栅格化：

| 位置 | 作用 |
|---|---|
| `grid_map_manual_mapping/src/manual_mapping_node.cpp:502-574` | `scanCallback`：查 `map <- base_scan` 的 TF，把每束激光从极坐标换成世界坐标 |
| 同文件 `:577-601` | `castRay`：Bresenham 画射线，沿途格子记“空” |
| 同文件 `:603-619` | `markFree` / `markOccupied`：misses / hits 计数 |
| 同文件 `:621-626` | `worldToGrid`：世界坐标 → 栅格索引 |
| 同文件 `:655-669` | `refreshMapData`：hits ≥ 阈值记障碍，否则看到过就是空地 |

这部分就是你要“搬”的核心逻辑。**区别**是：建图要把 hits/misses 永久保存；代价地图障碍物层要能**清除已经消失的动态障碍**，还要做膨胀，并且绝对不能把静态层覆盖掉。

---

## 二、新节点加在哪里

建议放在**现有的 `grid_map_gazebo_bridge` 包里**，不要新建包。理由：它本来就是“占据栅格 → 代价地图”这一层，障碍物层是同一链路的下一步；`logManager`、`grid_map_*` 依赖都已经配好，CMake 改动最小。

| 文件 | 动作 |
|---|---|
| `src/mapping/grid_map/grid_map_gazebo_bridge/src/costmap_obstacle_layer.cpp` | **新建**，新节点源码 |
| 同目录 `CMakeLists.txt` | 加 `sensor_msgs / tf2 / tf2_ros` 依赖 + `add_executable` |
| 同目录 `package.xml` | 加 `sensor_msgs / tf2`，把 `tf2_ros` 改成 `<depend>` |
| `src/bringup/config/grid_map.yaml` | 最后一步加 `costmap_obstacle_layer:` 参数段 |
| `src/bringup/launch/grid_map.launch` | 最后一步加节点；同时把 bridge 的 `costmap_topic` 改成 `/costmap_static` |

最终的拓扑会长这样（**这一步先不动 launch**）：

```
/map ──► occupancy_grid_to_cost_map ──► /costmap_static   （静态底图，仍然带膨胀）
                                              │
/scan ──► costmap_obstacle_layer ◄────────────┘
                  │  动态障碍 + 清除 + 膨胀
                  ▼
              /costmap   ──► RViz、dijkstra（这两个不用改）
```

为什么让 bridge 改发 `/costmap_static`：新节点既要读底图又要发 `/costmap`，如果底图还叫 `/costmap` 就会**自己订阅自己**形成回环。分成两个话题最干净，而且 RViz 和 dijkstra 订阅的 `/costmap` 名字保持不变。

---

## 三、分步路线（循序渐进，每次只做一步）

| 步骤 | 目标 | 验证方式 |
|---|---|---|
| **第 1 步（本次）** | 新建节点骨架：参数、订阅、发布、日志；静态层原样透传 | 能编译，`/costmap_obstacle_test` 和 `/costmap` 内容一致 |
| 第 2 步 | 按静态图对齐动态层缓冲；`/scan` 查 TF，算出每束激光终点 | 日志能打印激光原点/终点，不再只有静态层 |
| 第 3 步 | 只标终点：把终点写成 100，和静态层取 max 后发布 | 机器人前方放个箱子，RViz 里立刻出现障碍 |
| 第 4 步 | 补 Bresenham 射线清除 + hits 阈值 | 箱子移开后障碍会消失 |
| 第 5 步 | 动态障碍膨胀（复用 `computeCost` 的指数衰减公式） | 障碍周围出现灰色膨胀带 |
| 第 6 步 | 改 launch / yaml 接入正式话题，联调 RViz + dijkstra | `roslaunch bringup grid_map.launch` 一步到位 |

---

## 四、第 1 步：只搭骨架 + 静态层透传

### 4.1 新建源文件

新建 `src/mapping/grid_map/grid_map_gazebo_bridge/src/costmap_obstacle_layer.cpp`，内容如下：

```cpp
// 代价地图障碍物层（costmap_obstacle_layer）。
//
// 干什么：
//   * 订阅静态代价地图（默认 /costmap_static，由 occupancy_grid_to_cost_map 发布）当底图；
//   * 订阅激光（默认 /scan），把传感器实时看到的障碍叠加到底图上；
//   * 发布合并后的 nav_msgs/OccupancyGrid 到 /costmap，供 RViz 与 dijkstra 使用。
//
// 第 1 步只搭骨架：参数、订阅/发布、日志，并把静态层原样转发出去。
// 后面几步再往 scanCallback 里填：坐标变换 -> 终点标记 -> 射线清除 -> 膨胀。
#include <cstddef>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>

#include "logManager/logManager.h"

class CostmapObstacleLayer
{
public:
  CostmapObstacleLayer()
      : privateNodeHandle_("~"),
        tfListener_(tfBuffer_)  // 必须先于 tfListener_ 构造
  {
    // ---------------- 参数 ----------------
    privateNodeHandle_.param<std::string>("scan_topic", scanTopic_, "/scan");
    privateNodeHandle_.param<std::string>("static_costmap_topic", staticCostmapTopic_,
                                          "/costmap_static");
    privateNodeHandle_.param<std::string>("output_costmap_topic", outputCostmapTopic_, "/costmap");
    privateNodeHandle_.param<std::string>("map_frame", mapFrame_, "map");

    // ---------------- 接口 ----------------
    staticCostmapSubscriber_ = nodeHandle_.subscribe(
        staticCostmapTopic_, 1, &CostmapObstacleLayer::staticCostmapCallback, this);
    scanSubscriber_ =
        nodeHandle_.subscribe(scanTopic_, 1, &CostmapObstacleLayer::scanCallback, this);
    costmapPublisher_ =
        nodeHandle_.advertise<nav_msgs::OccupancyGrid>(outputCostmapTopic_, 1, true);

    LOG(INFO) << "代价地图障碍物层已启动: " << scanTopic_ << " + " << staticCostmapTopic_
              << " -> " << outputCostmapTopic_ << " (map_frame=" << mapFrame_ << ")";
  }

private:
  // 静态层：直接缓存下来当底图。注意是拷贝，后面不能改它。
  void staticCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message)
  {
    staticCostmap_ = *message;
    hasStaticCostmap_ = true;

    LOG(INFO) << "收到静态代价地图: " << message->info.width << "x" << message->info.height
              << " @ " << message->info.resolution << " m, frame=" << message->header.frame_id;

    // 第 1 步先原样转发，用来确认话题链路通了。
    costmapPublisher_.publish(staticCostmap_);
  }

  // 第 1 步只统计收到多少帧，算法从第 2 步开始填。
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    ++scanCount_;
    LOG_EVERY_T(INFO, 5.0) << "已收到 " << scanCount_ << " 帧激光 (frame="
                            << scan->header.frame_id << ", " << scan->ranges.size() << " 束)";
  }

  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;
  ros::Subscriber staticCostmapSubscriber_;
  ros::Subscriber scanSubscriber_;
  ros::Publisher costmapPublisher_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  nav_msgs::OccupancyGrid staticCostmap_;
  bool hasStaticCostmap_ = false;
  std::size_t scanCount_ = 0;

  std::string scanTopic_;
  std::string staticCostmapTopic_;
  std::string outputCostmapTopic_;
  std::string mapFrame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_obstacle_layer");
  // 和其它 mapping 节点保持一致，日志落到 log/mapping.INFO 等文件
  LOG_INIT("./log", 7, 6 * 3600, "mapping");

  CostmapObstacleLayer node;
  ros::spin();
  return 0;
}
```

### 4.2 改 `CMakeLists.txt`

`find_package(catkin ...)` 里补 `sensor_msgs tf2 tf2_ros`：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  grid_map_core
  grid_map_msgs
  grid_map_ros
  grid_map_sdf
  logManager
)
```

`catkin_package(CATKIN_DEPENDS ...)` 里同样补上这三项。

然后在 `static_turtlebot3_world_map` 那一组后面加：

```cmake
add_executable(costmap_obstacle_layer
  src/costmap_obstacle_layer.cpp
)

add_dependencies(costmap_obstacle_layer
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(costmap_obstacle_layer
  ${catkin_LIBRARIES}
)
```

`install(TARGETS ...)` 里把 `costmap_obstacle_layer` 加进去：

```cmake
install(TARGETS
    occupancy_grid_to_cost_map
    static_turtlebot3_world_map
    costmap_obstacle_layer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 4.3 改 `package.xml`

```xml
  <depend>sensor_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
```

注意：原来的 `<exec_depend>tf2_ros</exec_depend>` 要**删掉或合并**，否则同一个包同时出现 `depend` 和 `exec_depend` 会告警。

### 4.4 编译并验证

```bash
cd /home/wjw/Algorithm
source /opt/ros/noetic/setup.bash
catkin_make --pkg grid_map_gazebo_bridge -j4
source devel/setup.bash
```

先启动原来的 `roslaunch bringup grid_map.launch`（**这一步不改 launch，原来的 `/costmap` 仍由 bridge 发布**），再单独跑新节点，把输入接到 `/costmap`、输出起个临时名字，避免回环：

```bash
rosrun grid_map_gazebo_bridge costmap_obstacle_layer \
  _static_costmap_topic:=/costmap \
  _output_costmap_topic:=/costmap_obstacle_test
```

验证：

```bash
# 1) 节点在不在
rosnode info /costmap_obstacle_layer

# 2) 透传的图和原静态图是否同尺寸/同原点
rostopic echo -n1 /costmap/info
rostopic echo -n1 /costmap_obstacle_test/info

# 3) 激光有没有进来（日志里应该每 5 秒一条“已收到 N 帧激光”）
tail -f /home/wjw/Algorithm/log/mapping.INFO
```

预期：能编译通过、节点常驻、`/costmap_obstacle_test` 和 `/costmap` 的 `info` 完全一致，日志每 5 秒打印一次激光帧数。**此时 RViz 里的障碍物还是静态的，这是正常的**，第 2、3 步才会真正把 `/scan` 写进去。

---

## 五、第 1 步想让你先记住的两点

1. **几何不要硬编码**：后面动态层的 `resolution / width / height / origin_x / origin_y` 必须从 `staticCostmap_.info` 里取，不能像建图那样写死 200×200、原点 (-5,-5)，否则静态层和动态层会错格。
2. **静态层只读**：`staticCostmap_` 永远不动，动态结果单独放一个 `std::vector<int8_t>`，发布时再合并。这样静态地图不会被激光“擦掉”。

---

## 六、下一步

先按 4.1～4.4 改完并编译；如果编译或日志有报错，把报错贴出来。

确认第 1 步通了，再进入**第 2 步：几何对齐 + TF + 每束激光终点的计算**。

