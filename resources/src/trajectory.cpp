#include <SimpleSLAM/resources/trajectory.hpp>

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace simpleslam {

void Trajectory::append(Timestamp ts, const SE3d& pose) {
    std::lock_guard lock(mutex_);
    poses_.push_back({ts, pose});
}

size_t Trajectory::size() const {
    std::lock_guard lock(mutex_);
    return poses_.size();
}

bool Trajectory::empty() const {
    std::lock_guard lock(mutex_);
    return poses_.empty();
}

std::vector<Trajectory::PoseEntry> Trajectory::entries() const {
    std::lock_guard lock(mutex_);
    return poses_;
}

std::optional<SE3d> Trajectory::poseAt(Timestamp ts) const {
    std::lock_guard lock(mutex_);
    if (poses_.empty()) return std::nullopt;
    if (ts <= poses_.front().timestamp) return poses_.front().pose;
    if (ts >= poses_.back().timestamp) return poses_.back().pose;

    // 找到第一个 timestamp > ts 的条目
    auto it = std::upper_bound(
        poses_.begin(), poses_.end(), ts,
        [](Timestamp t, const PoseEntry& e) { return t < e.timestamp; });
    assert(it != poses_.begin());

    const auto& after = *it;
    const auto& before = *std::prev(it);

    // 线性插值比例
    const double ratio = (ts - before.timestamp) / (after.timestamp - before.timestamp);

    // 平移线性插值 + 旋转 SLERP
    const Vec3d t_interp =
        (1.0 - ratio) * before.pose.translation() + ratio * after.pose.translation();

    // manif SE3d 内部存 quaternion，通过 4x4 矩阵提取旋转
    const Eigen::Matrix3d R_before = before.pose.rotation();
    const Eigen::Matrix3d R_after = after.pose.rotation();
    const Eigen::Quaterniond q_before(R_before);
    const Eigen::Quaterniond q_after(R_after);
    const Eigen::Quaterniond q_interp = q_before.slerp(ratio, q_after);

    return SE3d(t_interp, q_interp);
}

void Trajectory::exportTUM(const std::string& path) const {
    std::lock_guard lock(mutex_);
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Trajectory: cannot open " + path);
    }
    file << std::fixed << std::setprecision(9);

    for (const auto& entry : poses_) {
        const auto& t = entry.pose.translation();
        const Eigen::Quaterniond q(entry.pose.rotation());
        // TUM 格式: timestamp tx ty tz qx qy qz qw
        file << entry.timestamp << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << "\n";
    }
}

void Trajectory::exportKITTI(const std::string& path) const {
    std::lock_guard lock(mutex_);
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Trajectory: cannot open " + path);
    }
    file << std::fixed << std::setprecision(9);

    for (const auto& entry : poses_) {
        // KITTI 格式: 3×4 变换矩阵的前 3 行，行优先，12 个数用空格分隔
        const Eigen::Matrix4d T = entry.pose.transform();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (r > 0 || c > 0) file << " ";
                file << T(r, c);
            }
        }
        file << "\n";
    }
}

void Trajectory::clear() {
    std::lock_guard lock(mutex_);
    poses_.clear();
}

}  // namespace simpleslam
