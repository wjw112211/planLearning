#pragma once

#include <vector>
#include <cstdint>
#include <geometry_msgs/Polygon.h>
#include "logManager/logManager.h"
#include <queue>
#include <limits>
#include <algorithm>

namespace dijkstra
{
class DijkstraCore
{
public:
    DijkstraCore() = default;
    ~DijkstraCore() = default;
    
    // 定义起点和终点的索引坐标结构
    struct Point
    {
        int x; // 索引坐标的x值
        int y; // 索引坐标的y值
        // 重载点 等于，便于后续判断两个节点是否相等
        bool operator==(const Point& other) const
        {
            return x == other.x && y == other.y;
        }
    };

    // 定义图搜索的节点结构
    struct Node
    {
        Point point; // 节点的索引坐标
        double g; // 从起点到该节点的累计代价
    };

    // 设置代价地图，一维存储
    void setCostMap(const std::vector<int8_t>& cost_map, 
        int width, 
        int height,
        double resolution,
        double origin_x,
        double origin_y);

    // 设置起点和终点的索引坐标
    void setStartAndGoal(const Point start, const Point goal);

    // 执行Dijkstra规划
    bool makePlan();

    // 获取map系坐标下的路径
    const geometry_msgs::Polygon& getPathMap() const;


private:
    // 代价地图及其相关参数
    std::vector<int8_t> cost_map_;
    int width_{0};
    int height_{0};
    double resolution_{0.0};
    double origin_x_{0.0};
    double origin_y_{0.0};

    // 定义起点和终点
    Point start_{0, 0};
    Point goal_{0, 0};

    // 四连通搜索方向
    const std::vector<Point> directions_ = {
        {1, 0},     // 右
        {-1, 0},    // 左
        {0, 1},     // 上
        {0, -1}     // 下
    };


    // 索引坐标的路径，从起点到终点
    std::vector<Point> path_index_;

    // map系坐标下的路径，从起点到终点
    geometry_msgs::Polygon path_map_;

    //索引方式为 index = y * width + x
    // 二维索引转换为一维索引
    inline int toIndex(int x, int y); 
    // 一维索引转换为二维索引
    inline Point toPoint(int index);

    // 判断点是否在地图范围内
    bool isInside(const Point& point);

    // 判断点是否是障碍物里面，或者在位置区域中
    bool isObstacle(const Point& point);

    // 获取点的代价值
    int getCost(const Point& point);

    // 以四连通（上下左右）的方式搜索节点
    std::vector<Point> getNeighbors4(const Point& point);

    // 通过结构体定义比较规则
    struct NodeCompare
    {
        bool operator()(const Node& a, const Node& b) const
        {
            return a.g > b.g;
        }
    };

    // 使用优先级队列定义open_list_
    std::priority_queue<Node, std::vector<Node>, NodeCompare> open_list_;

    // 使用数组定义close_list_
    std::vector<bool> close_list_;

    // 记录起点到每个节点的最小累积代价
    std::vector<double> g_score_;
    
    //记录每个节点的父节点索引，用于路径回溯
    std::vector<int> parent_;

    //核心搜索函数
    bool searchPath();

    // 回溯
    void retracePath(int goal_index);

    // 将索引路径转换到map坐标系
    void convertPathToMap();


};
} // namespace dijkstra
