#include "dijkstra_core.hpp"

namespace dijkstra
{          
// 设置代价地图        
void DijkstraCore::setCostMap(const std::vector<int8_t>& cost_map, 
    int width,
    int height,
    double resolution,
    double origin_x,
    double origin_y)
{
    cost_map_ = cost_map;
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
}

// 设置起点和终点，在索引坐标下
void DijkstraCore::setStartAndGoal(const Point start, const Point goal)
{
    start_ = start;
    goal_ = goal;
}

// 执行Dijkstra规划
bool DijkstraCore::makePlan()
{
    return searchPath();
}

// 获取map系坐标下的路径
const geometry_msgs::Polygon& DijkstraCore::getPathMap() const
{
    return path_map_;
}

// 二维索引转换为一维索引
int DijkstraCore::toIndex(int x, int y)
{
    return y * width_ + x;
}

// 一维索引转换为二维索引
DijkstraCore::Point DijkstraCore::toPoint(int index)
{
    Point point;
    point.x = index % width_;
    point.y = index / width_;
    return point;
}

// 判断点是否在地图范围内
bool DijkstraCore::isInside(const Point& point)
{
    return point.x >= 0 && point.x < width_ &&
           point.y >= 0 && point.y < height_;
}    

// 获取该点的代价值
// 0～99为自由区域，值越小代价越低，代表越安全
// -1为未知区域，建图时没有去过的位置
// 100为障碍物的位置
int DijkstraCore::getCost(const Point& point)
{
    if(!isInside(point))
    {
        return -1; // 如果点不在地图范围内，返回-1表示位置区域
    }

    return static_cast<int>(cost_map_[toIndex(point.x, point.y)]);    
}

// 判断点是否在障碍物里面，或者在位置区域中
bool DijkstraCore::isObstacle(const Point& point)
{
    int cost = getCost(point);

    return cost < 0 || cost >= 100;
}

// 四连通路径搜索
std::vector<DijkstraCore::Point> DijkstraCore::getNeighbors4(const Point& point)
{
    std::vector<Point> neighbors;
    // 遍历四个方向的节点
    // 排除地图外的节点
    // 排除障碍物中，或者未知区域的节点
    for(const Point& direction : directions_)
    {
        Point next;
        next.x = point.x + direction.x;
        next.y = point.y + direction.y;

        if(!isInside(next)) continue;
        if(isObstacle(next)) continue;
        
        neighbors.push_back(next);
    }

    return neighbors;
}

// 回溯
void DijkstraCore::retracePath(int goal_index)
{
    path_index_.clear();

    int current_index = goal_index;

    while(current_index != -1)
    {
        Point current_point = toPoint(current_index);
        path_index_.push_back(current_point);

        current_index = parent_[current_index];
    }

    std::reverse(path_index_.begin(), path_index_.end());
    convertPathToMap();
}

// 将索引路径转换到map坐标系
void DijkstraCore::convertPathToMap()
{
    path_map_.points.clear();

    for(const Point& point : path_index_)
    {
        geometry_msgs::Point32 map_point;
        map_point.x = origin_x_ + (static_cast<double>(point.x) + 0.5) * resolution_;
        map_point.y = origin_y_ + (static_cast<double>(point.y) + 0.5) * resolution_;
        map_point.z = 0.0;

        path_map_.points.push_back(map_point);
    }
}

// 搜索函数，暴露给外部使用
bool DijkstraCore::searchPath()
{
    // 开始议论搜索，把各个容器恢复成初始状态
    int map_size = width_ * height_;
    path_index_.clear();
    path_map_.points.clear();

    if(map_size <= 0 || static_cast<int>(cost_map_.size()) != map_size)
    {
        LOG(WARNING) << "Invalid costmap, width=" << width_ << ", height=" << height_
                     << ", cost_map size=" << cost_map_.size();
        return false;
    }

    if(!isInside(start_) || !isInside(goal_))
    {
        LOG(WARNING) << "Start or goal is outside map, start=(" << start_.x << "," << start_.y
                     << "), goal=(" << goal_.x << "," << goal_.y << ")";
        return false;
    }

    if(isObstacle(start_) || isObstacle(goal_))
    {
        LOG(WARNING) << "Start or goal is obstacle, start cost=" << getCost(start_)
                     << ", goal cost=" << getCost(goal_);
        return false;
    }

    close_list_.assign(map_size, false);
    g_score_.assign(map_size, std::numeric_limits<double>::infinity());
    parent_.assign(map_size, -1);
    
    // 如果open_list_不为空，遍历弹出所有元素
    while (!open_list_.empty())
    {
        open_list_.pop();
    }

    // 处理起点
    int start_index = toIndex(start_.x, start_.y);
    g_score_[start_index] = 0.0;
    Node start_node;
    start_node.point = start_;
    start_node.g = 0.0;
    open_list_.push(start_node);

    // 从起点开始搜索
    while(!open_list_.empty())
    {
        Node cur = open_list_.top();
        open_list_.pop();

        // 如果当前点在close_list_中，跳过当前点的四连通搜索
        // 如果但前点不再close_list_中，说明当前点没有进行搜索
        // 然后对当前点的四连通进行搜索,并加入到close_list_中
        int cur_index = toIndex(cur.point.x, cur.point.y);
        if(close_list_[cur_index]) continue;
        close_list_[cur_index] = true;

        // 如果但前点是终点，搜索完成
        // 其他情况下，进行四连通搜索
        if(cur.point == goal_) 
        {
            retracePath(cur_index);
            return true;
        }

        // 找四连通的邻居
        // 这些邻居所在的代价值需要是安全的
        std::vector<Point> neighbors = getNeighbors4(cur.point);

        // 遍历邻居，做松弛更新
        for(const Point& neighbor : neighbors)
        {
            int neighbor_index = toIndex(neighbor.x, neighbor.y);
            if(close_list_[neighbor_index]) continue;
            double new_g = g_score_[cur_index] + 1.0 + static_cast<double>(getCost(neighbor));
            if(new_g < g_score_[neighbor_index])
            {
                g_score_[neighbor_index] = new_g;
                parent_[neighbor_index] = cur_index;

                Node neighbor_node;
                neighbor_node.point = neighbor;
                neighbor_node.g = new_g;

                open_list_.push(neighbor_node);
            }
        }

    }
    
    return false;
}


} //namespace dijkstra
