#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include "logManager/logManager.h"

namespace minijerk
{
class MinijerkCore
{
public:
    // 构造函数：引用接口层的读写锁，core 和 interface 共用同一份轨迹数据
    explicit MinijerkCore(std::shared_mutex& mutex);
    ~MinijerkCore() = default;

    // 设置采样间隔，单位 m：粗糙轨迹按弧长等间隔重采样，间隔越小平滑后的轨迹越密
    void setSampleInterval(double sample_interval);

    // 设置特征点阈值，单位 m：Douglas-Peucker 容许的最大横向偏差，
    // 阈值越大选出来的特征点越少、平滑后的轨迹偏离粗糙轨迹越多
    void setFeatureEpsilon(double feature_epsilon);

    // 设置保真项权重：目标函数是 min( Σjerk² + weight * Σ(x - 粗糙轨迹)² )
    // weight 越大越贴近粗糙轨迹（平滑越弱），越小越平滑。
    // 它同时保证 QP 严格凸：只有两个特征点时，纯最小 jerk 的解不唯一（二次序列的
    // 三阶差分都是 0），加了保真项才有唯一解，不会解出一条乱漂的曲线
    void setFidelityWeight(double fidelity_weight);

    // 获取需要经过平滑处理的轨迹（加写锁保存，线程安全）
    void getTrajectory(const nav_msgs::Path& path);

    // 轨迹平滑主流程：采样 -> 选特征点 -> mini-jerk 求解 -> 输出平滑轨迹
    // 成功返回 true，并把结果写进 smooth_path
    bool smooth(nav_msgs::Path& smooth_path);

private:
    // 平面点，轨迹平滑只关心 x、y
    struct Point2d
    {
        double x{0.0};
        double y{0.0};
    };

    // 接口层的读写锁
    std::shared_mutex& mutex_;

    // 需要平滑处理的粗糙轨迹
    nav_msgs::Path front_path_;

    // 采样间隔，单位 m
    double sample_interval_{0.1};

    // 特征点阈值，单位 m
    double feature_epsilon_{0.05};

    // 保真项权重，见 setFidelityWeight 的说明
    double fidelity_weight_{1.0};

    // 1、沿弧长等间隔重采样，得到待平滑的点序列
    bool sampleTrajectory(const nav_msgs::Path& path, std::vector<Point2d>& samples) const;

    // 2、用 Douglas-Peucker 选出特征点，返回采样序列中的下标（升序，包含首尾点）
    std::vector<int> pickFeaturePoints(const std::vector<Point2d>& samples) const;

    // 点到线段的距离，Douglas-Peucker 判断中间点能不能省略时使用
    static double distanceToSegment(const Point2d& point, const Point2d& start, const Point2d& end);

    // 3、mini-jerk 平滑：三阶差分平方和为目标、特征点为等式约束，用 OSQP 求解
    bool smoothByMinJerk(const std::vector<Point2d>& samples,
                         const std::vector<int>& feature_indices,
                         std::vector<Point2d>& smooth_points) const;

    // 单个坐标轴的 QP 求解（x、y 的目标矩阵和约束矩阵相同，只有右端项不同）
    bool solveAxisQp(const std::vector<double>& sample_value,
                     const std::vector<int>& feature_indices,
                     std::vector<double>& smooth_value) const;

    // 4、把平滑后的点打包成 nav_msgs::Path，姿态取平滑轨迹的切线方向
    void buildSmoothPath(const nav_msgs::Path& rough_path,
                         const std::vector<Point2d>& points,
                         nav_msgs::Path& smooth_path) const;
};
} // namespace minijerk
