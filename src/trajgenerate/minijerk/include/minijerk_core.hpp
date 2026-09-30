#pragma once    

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <shared_mutex>

namespace minijerk
{
class MinijerkCore
{
public:
    MinijerkCore() = default;
    ~MinijerkCore() = default;


private:
    // 获取读写锁
    std::shared_mutex& mutex_;
    void getLock(auto& lock);


    // 获取需要平滑处理的轨迹
    nav_msgs::Path front_path_;
    void getTrajectory(const nav_msgs::Path& path);

    






};
} // namespace minijerk