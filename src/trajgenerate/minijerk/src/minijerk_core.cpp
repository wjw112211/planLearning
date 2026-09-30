#include "minijerk/minijerk_core.h"

namespace minijerk
{

// 获取读写锁
void MinijerkCore::getMutex(auto& lock)
{
    mutex_ = lock;
}

// 获取需要经过平滑的轨迹
void MinijerkCore::getTrajectory(const nav_msgs::Path& path)
{
    std::unique_lock lock(mutex_);
    front_path_ = path;

    lock.unlock();
    LOG(INFO) << "Get trajectory, size: " << front_path_.size();     
}





} // namespace minijerk