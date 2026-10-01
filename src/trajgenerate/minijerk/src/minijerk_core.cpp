#include "minijerk_core.hpp"

#include <algorithm>
#include <cmath>
// QP 求解器 OSQP 的 C 接口（由 ros-noetic-osqp-vendor 提供）。
// osqp::osqp 这个 CMake 目标把 include 目录指到了 <前缀>/include/osqp，
// 所以这里直接包含 osqp.h
#include <osqp.h>

namespace minijerk
{
namespace
{
// 三阶差分（jerk 的离散近似）在同一行里的系数，一行的形式是：
//   jerk_k ≈ x[k+3] - 3 * x[k+2] + 3 * x[k+1] - x[k]
// 下标表示该系数相对本行起始列的偏移
constexpr double kJerkStencil[4] = {-1.0, 3.0, -3.0, 1.0};

// 三阶差分的带宽：相距超过 3 的两个采样点不会出现在同一个差分行里
constexpr int kJerkBand = 3;

// 重采样时判断两个点是否重合，避免零长度线段导致插值除零
constexpr double kMinSegmentLength = 1e-6;

// OSQP 求解精度：路径坐标是米级，1e-6 足以让等式约束点准确落在特征点上
constexpr double kQpEpsAbs = 1e-6;
constexpr double kQpEpsRel = 1e-6;
constexpr int kQpMaxIter = 4000;
} // namespace

// 构造函数：引用接口层的读写锁
MinijerkCore::MinijerkCore(std::shared_mutex& mutex)
    : mutex_(mutex)
{
}

// 设置采样间隔
void MinijerkCore::setSampleInterval(double sample_interval)
{
    if(sample_interval <= kMinSegmentLength)
    {
        LOG(WARNING) << "invalid sample interval: " << sample_interval
                     << ", keep " << sample_interval_;
        return;
    }

    sample_interval_ = sample_interval;
}

// 设置特征点阈值
void MinijerkCore::setFeatureEpsilon(double feature_epsilon)
{
    if(feature_epsilon < 0.0)
    {
        LOG(WARNING) << "invalid feature epsilon: " << feature_epsilon
                     << ", keep " << feature_epsilon_;
        return;
    }

    feature_epsilon_ = feature_epsilon;
}

// 设置保真项权重
void MinijerkCore::setFidelityWeight(double fidelity_weight)
{
    if(fidelity_weight <= 0.0)
    {
        LOG(WARNING) << "invalid fidelity weight: " << fidelity_weight
                     << " (must be positive), keep " << fidelity_weight_;
        return;
    }

    fidelity_weight_ = fidelity_weight;
}

// 获取需要经过平滑的轨迹
void MinijerkCore::getTrajectory(const nav_msgs::Path& path)
{
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        front_path_ = path;
    }

    LOG(INFO) << "Get trajectory, size: " << path.poses.size();
}

// 轨迹平滑主流程
bool MinijerkCore::smooth(nav_msgs::Path& smooth_path)
{
    // 加锁拷贝一份粗糙轨迹，后面的采样和求解都在锁外进行，不长时间占用锁
    nav_msgs::Path rough_path;
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        rough_path = front_path_;
    }

    // 1、在粗糙轨迹上按固定弧长采样
    std::vector<Point2d> samples;
    if(!sampleTrajectory(rough_path, samples))
    {
        LOG(WARNING) << "[MINIJERK FAILED] sample trajectory failed, rough point size="
                     << rough_path.poses.size();
        return false;
    }

    // 2、选出特征点：首尾点，以及横向偏差超阈值的点（转弯处的点）
    const std::vector<int> feature_indices = pickFeaturePoints(samples);

    // 3、以特征点为等式约束做 mini-jerk 平滑
    std::vector<Point2d> smooth_points;
    if(!smoothByMinJerk(samples, feature_indices, smooth_points))
    {
        LOG(WARNING) << "[MINIJERK FAILED] minijerk smoothing failed, sample size="
                     << samples.size();
        return false;
    }

    // 4、打包成 nav_msgs::Path
    buildSmoothPath(rough_path, smooth_points, smooth_path);

    LOG(INFO) << "[MINIJERK DONE] sample size=" << samples.size()
              << ", feature size=" << feature_indices.size()
              << ", smooth size=" << smooth_path.poses.size();
    return true;
}

// 沿弧长等间隔重采样：粗糙轨迹一般是栅格搜索出来的，点间距不均匀，
// 采样成等间距的点序列后，后面的三阶差分才有一致的物理含义
bool MinijerkCore::sampleTrajectory(const nav_msgs::Path& path, std::vector<Point2d>& samples) const
{
    samples.clear();

    // 先去掉重合点，同时记录每个有效点到起点的累积弧长
    std::vector<Point2d> raw_points;
    std::vector<double> raw_length;
    raw_points.reserve(path.poses.size());
    raw_length.reserve(path.poses.size());

    for(const geometry_msgs::PoseStamped& pose : path.poses)
    {
        Point2d point;
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;

        if(raw_points.empty())
        {
            raw_length.push_back(0.0);
        }
        else
        {
            const double step = std::hypot(point.x - raw_points.back().x,
                                           point.y - raw_points.back().y);
            // 和上一个点重合的点直接丢掉，保证每段弧长都是正的
            if(step < kMinSegmentLength) continue;

            raw_length.push_back(raw_length.back() + step);
        }

        raw_points.push_back(point);
    }

    if(raw_points.size() < 2)
    {
        LOG(WARNING) << "rough path is too short, valid point size=" << raw_points.size();
        return false;
    }

    const double total_length = raw_length.back();
    if(total_length < kMinSegmentLength)
    {
        LOG(WARNING) << "rough path length is too short: " << total_length;
        return false;
    }

    // 按固定弧长间隔等分，点数向上取整，首尾点一定在采样结果里
    const int num_samples = std::max(2, static_cast<int>(std::ceil(total_length / sample_interval_)) + 1);
    samples.reserve(num_samples);

    int segment = 0;

    for(int i = 0; i < num_samples; ++i)
    {
        // 第 i 个采样点到起点的弧长，首尾分别对应 0 和 total_length
        const double target = total_length * static_cast<double>(i) / static_cast<double>(num_samples - 1);

        // 弧长单调递增，用一个游标顺序推进就能定位所在线段
        while(segment + 1 < static_cast<int>(raw_points.size()) && raw_length[segment + 1] < target)
        {
            ++segment;
        }

        const double segment_length = raw_length[segment + 1] - raw_length[segment];
        const double ratio = (target - raw_length[segment]) / segment_length;

        Point2d point;
        point.x = raw_points[segment].x + ratio * (raw_points[segment + 1].x - raw_points[segment].x);
        point.y = raw_points[segment].y + ratio * (raw_points[segment + 1].y - raw_points[segment].y);
        samples.push_back(point);
    }

    return true;
}

// 用 Douglas-Peucker 选特征点：首尾点一定保留，中间点看它到首尾连线的横向偏差，
// 偏差超过阈值说明这个点不能省略（在转弯处），作为特征点继续细分两侧
std::vector<int> MinijerkCore::pickFeaturePoints(const std::vector<Point2d>& samples) const
{
    std::vector<int> feature_indices;

    if(samples.size() < 2)
    {
        for(size_t i = 0; i < samples.size(); ++i)
        {
            feature_indices.push_back(static_cast<int>(i));
        }
        return feature_indices;
    }

    std::vector<bool> is_feature(samples.size(), false);
    is_feature.front() = true;
    is_feature.back() = true;

    // 用显式栈代替递归，轨迹点很多时也不会把调用栈压爆
    std::vector<std::pair<size_t, size_t>> pending;
    pending.push_back({0, samples.size() - 1});

    while(!pending.empty())
    {
        const auto [first, last] = pending.back();
        pending.pop_back();

        if(last <= first + 1) continue;

        double max_distance = 0.0;
        size_t max_index = first;

        for(size_t i = first + 1; i < last; ++i)
        {
            const double distance = distanceToSegment(samples[i], samples[first], samples[last]);
            if(distance > max_distance)
            {
                max_distance = distance;
                max_index = i;
            }
        }

        if(max_distance > feature_epsilon_)
        {
            is_feature[max_index] = true;
            pending.push_back({first, max_index});
            pending.push_back({max_index, last});
        }
    }

    for(size_t i = 0; i < is_feature.size(); ++i)
    {
        if(is_feature[i])
        {
            feature_indices.push_back(static_cast<int>(i));
        }
    }

    return feature_indices;
}

// 点到线段的距离
double MinijerkCore::distanceToSegment(const Point2d& point, const Point2d& start, const Point2d& end)
{
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;

    // 线段退化成点，直接算点距
    if(length_squared < kMinSegmentLength * kMinSegmentLength)
    {
        return std::hypot(point.x - start.x, point.y - start.y);
    }

    // 把点投影到线段所在直线，投影参数截断在 [0, 1] 内，得到线段上的最近点
    double ratio = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared;
    ratio = std::min(1.0, std::max(0.0, ratio));

    return std::hypot(point.x - (start.x + ratio * dx), point.y - (start.y + ratio * dy));
}

// mini-jerk 平滑：x、y 两个方向分别构建 QP
bool MinijerkCore::smoothByMinJerk(const std::vector<Point2d>& samples,
                                   const std::vector<int>& feature_indices,
                                   std::vector<Point2d>& smooth_points) const
{
    smooth_points.clear();

    // 采样点少于 4 个就构不出三阶差分，退化成不平滑，直接返回采样点
    if(samples.size() < 4)
    {
        LOG(WARNING) << "sampled point size " << samples.size() << " < 4, skip minijerk smoothing";
        smooth_points = samples;
        return !samples.empty();
    }

    if(feature_indices.size() < 2)
    {
        LOG(WARNING) << "feature point size " << feature_indices.size() << " < 2, need start and goal";
        return false;
    }

    std::vector<double> sample_x(samples.size(), 0.0);
    std::vector<double> sample_y(samples.size(), 0.0);
    for(size_t i = 0; i < samples.size(); ++i)
    {
        sample_x[i] = samples[i].x;
        sample_y[i] = samples[i].y;
    }

    // 两个方向的二次型和约束形式完全一样，分别求解即可
    std::vector<double> smooth_x;
    std::vector<double> smooth_y;
    if(!solveAxisQp(sample_x, feature_indices, smooth_x)) return false;
    if(!solveAxisQp(sample_y, feature_indices, smooth_y)) return false;

    smooth_points.resize(samples.size());
    for(size_t i = 0; i < samples.size(); ++i)
    {
        smooth_points[i].x = smooth_x[i];
        smooth_points[i].y = smooth_y[i];
    }

    return true;
}

// 单个坐标轴的 mini-jerk QP
//
//   min  Σ_k (x[k+3] - 3 * x[k+2] + 3 * x[k+1] - x[k])^2 + w * Σ_k (x[k] - r[k])^2
//   s.t. x[i] = p[i]，i 取遍特征点下标
//
// 第一项是三阶差分（≈ jerk * dt^3），第二项是保真项，让结果贴在粗糙轨迹附近；
// 三阶差分是线性算子，所以目标函数是关于 x 的二次型、约束是线性的，构成标准 QP。
// 采样间隔固定时三阶差分与 jerk 只差常数因子 1/dt^3，不影响最优解，所以直接最小化差分平方和。
//
// 保真项不能省：三阶差分的零空间是三次以内的序列（首尾三阶差分都算不出来），只固定首尾两个点
// 时最小 jerk 的解不唯一，求解器可能给出乱漂的曲线；加上 w > 0 的保真项后 P 正定，解唯一。
// 求解交给 OSQP（ADMM 迭代），全程不做矩阵求逆。
bool MinijerkCore::solveAxisQp(const std::vector<double>& sample_value,
                               const std::vector<int>& feature_indices,
                               std::vector<double>& smooth_value) const
{
    const int n = static_cast<int>(sample_value.size());
    const int m = static_cast<int>(feature_indices.size());
    if(n < 4 || m < 2) return false;

    // ---------- 目标函数矩阵 P = 2 * (D3ᵀ * D3 + w * I)，只存上三角 ----------
    // D3 是 (n-3) x n 的三阶差分矩阵，第 k 行只覆盖 k ~ k+3 列。
    // 这里不做矩阵乘法，直接按差分系数的自相关算出带状元素：
    //   P(i, j) = Σ_k c[i - k] * c[j - k]，c 就是那 4 个差分系数；
    // i、j 相距超过 3 时没有一行能同时覆盖这两列，结果为 0，所以 P 是带宽为 3 的带状矩阵。
    std::vector<c_float> p_values;
    std::vector<c_int> p_rows;
    std::vector<c_int> p_col_pointer(n + 1, 0);
    p_values.reserve(4 * n);
    p_rows.reserve(4 * n);

    for(int col = 0; col < n; ++col)
    {
        p_col_pointer[col] = static_cast<c_int>(p_values.size());

        // OSQP 要求 P 是上三角矩阵：行下标不大于列下标
        const int row_begin = std::max(0, col - kJerkBand);
        for(int row = row_begin; row <= col; ++row)
        {
            // 第 k 行要同时覆盖 row、col 两列：k <= row 且 k + 3 >= col；
            // 另外差分矩阵只有 n - 3 行，k 最大是 n - 4
            const int k_begin = std::max(0, col - kJerkBand);
            const int k_end = std::min(row, n - 4);

            double value = 0.0;
            for(int k = k_begin; k <= k_end; ++k)
            {
                value += kJerkStencil[row - k] * kJerkStencil[col - k];
            }

            // 保真项 w * (x - r)² 的二次部分正好落在对角线上，顺便保证 P 正定
            if(row == col) value += fidelity_weight_;

            if(std::abs(value) < 1e-12) continue;

            p_values.push_back(static_cast<c_float>(2.0 * value));
            p_rows.push_back(static_cast<c_int>(row));
        }
    }
    p_col_pointer[n] = static_cast<c_int>(p_values.size());

    // ---------- 等式约束矩阵 A：每个特征点一行，在该下标处取 1 ----------
    std::vector<c_float> a_values(m, 1.0);
    std::vector<c_int> a_rows(m, 0);
    std::vector<c_int> a_col_pointer(n + 1, 0);

    for(int k = 0; k < m; ++k)
    {
        a_rows[k] = static_cast<c_int>(k);
    }

    // 特征点下标严格递增且不重复，顺序扫一遍就能填出列指针
    for(int col = 0, k = 0; col < n; ++col)
    {
        a_col_pointer[col] = static_cast<c_int>(k);
        if(k < m && feature_indices[k] == col) ++k;
    }
    a_col_pointer[n] = static_cast<c_int>(m);

    // ---------- 线性项与边界：等式约束 x[i] = 特征点坐标 ----------
    // 保真项展开是 w * x² - 2 * w * r * x + w * r²，常数项不影响最优解，
    // 所以线性项取 q = -2 * w * r（二次部分已经加在 P 的对角线上）
    std::vector<c_float> q(n, 0.0);
    for(int i = 0; i < n; ++i)
    {
        q[i] = static_cast<c_float>(-2.0 * fidelity_weight_ * sample_value[i]);
    }

    std::vector<c_float> lower(m, 0.0);
    std::vector<c_float> upper(m, 0.0);

    for(int k = 0; k < m; ++k)
    {
        const c_float value = static_cast<c_float>(sample_value[feature_indices[k]]);
        lower[k] = value;
        upper[k] = value; // 上下界相同，OSQP 按等式约束处理
    }

    // csc_matrix 只是把已有的三块数组包装起来（不拷贝内容），
    // osqp_setup 会把数据拷进它自己的工作区，所以只要保证数组在本函数结束前有效
    csc* P = csc_matrix(n, n, static_cast<c_int>(p_values.size()),
                        p_values.data(), p_rows.data(), p_col_pointer.data());
    csc* A = csc_matrix(m, n, static_cast<c_int>(m),
                        a_values.data(), a_rows.data(), a_col_pointer.data());
    if(P == nullptr || A == nullptr)
    {
        LOG(WARNING) << "[MINIJERK FAILED] allocate qp matrix failed.";
        c_free(P);
        c_free(A);
        return false;
    }

    OSQPData data;
    data.n = static_cast<c_int>(n);
    data.m = static_cast<c_int>(m);
    data.P = P;
    data.A = A;
    data.q = q.data();
    data.l = lower.data();
    data.u = upper.data();

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0; // 求解过程不往终端刷屏，结果写日志
    settings.polish = 1;  // 打开解的精修，让特征点上的等式约束更精确
    settings.max_iter = kQpMaxIter;
    settings.eps_abs = kQpEpsAbs;
    settings.eps_rel = kQpEpsRel;

    OSQPWorkspace* work = nullptr;
    const c_int setup_flag = osqp_setup(&work, &data, &settings);
    if(setup_flag != 0 || work == nullptr)
    {
        LOG(WARNING) << "[MINIJERK FAILED] osqp setup failed, exit flag=" << setup_flag;
        c_free(P);
        c_free(A);
        return false;
    }

    osqp_solve(work);

    const c_int status = work->info->status_val;
    // SOLVED_INACCURATE 表示没有达到目标精度但解可用，路径平滑这个量级可以接受，只记一条警告
    const bool solved = (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE);

    if(solved)
    {
        smooth_value.resize(n);
        for(int i = 0; i < n; ++i)
        {
            smooth_value[i] = static_cast<double>(work->solution->x[i]);
        }

        LOG(INFO) << "osqp solved, status=" << work->info->status
                  << ", iter=" << work->info->iter
                  << ", pri_res=" << work->info->pri_res
                  << ", dua_res=" << work->info->dua_res;

        if(status == OSQP_SOLVED_INACCURATE)
        {
            LOG(WARNING) << "osqp solved inaccurately, iterate count=" << work->info->iter;
        }
    }
    else
    {
        LOG(WARNING) << "[MINIJERK FAILED] osqp solve failed, status=" << work->info->status;
    }

    // osqp_cleanup 只释放 OSQP 内部的数据副本
    osqp_cleanup(work);

    // 这里只释放 csc 结构体本身：x、i、p 三块内存由 std::vector 管理，
    // 不能用 csc_spfree，否则会去 free 不属于它的内存
    c_free(P);
    c_free(A);

    return solved;
}

// 把平滑后的点打包成 nav_msgs::Path
void MinijerkCore::buildSmoothPath(const nav_msgs::Path& rough_path,
                                   const std::vector<Point2d>& points,
                                   nav_msgs::Path& smooth_path) const
{
    smooth_path.poses.clear();
    smooth_path.poses.reserve(points.size());

    // 坐标系沿用粗糙轨迹的，时间戳用当前时刻
    smooth_path.header = rough_path.header;
    smooth_path.header.stamp = ros::Time::now();

    for(size_t i = 0; i < points.size(); ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = smooth_path.header;
        pose.pose.position.x = points[i].x;
        pose.pose.position.y = points[i].y;
        pose.pose.position.z = 0.0;

        // 姿态取平滑轨迹的切线方向：中间点用前向差分，末点用后向差分
        double yaw = 0.0;
        if(points.size() >= 2)
        {
            const size_t next = (i + 1 < points.size()) ? (i + 1) : i;
            const size_t prev = (i + 1 < points.size()) ? i : (i - 1);
            const double dx = points[next].x - points[prev].x;
            const double dy = points[next].y - points[prev].y;

            if(std::hypot(dx, dy) > kMinSegmentLength)
            {
                yaw = std::atan2(dy, dx);
            }
        }

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(0.5 * yaw);
        pose.pose.orientation.w = std::cos(0.5 * yaw);

        smooth_path.poses.push_back(pose);
    }
}

} // namespace minijerk
