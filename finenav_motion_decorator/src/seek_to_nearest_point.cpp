// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "seek_to_nearest_point.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>
#include <limits>
#include <cstddef>

namespace finenav {

SeekToNearestPoint::SeekToNearestPoint(Eigen::Vector3d reference, bool use_2d)
    : reference_(std::move(reference)), use_2d_(use_2d)
{}

void SeekToNearestPoint::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    if (!(traj.valid_fields & T::TRAJ_POSE)) {
        throw std::runtime_error(
            "SeekToNearestPoint: trajectory does not have TRAJ_POSE populated");
    }

    if (traj.poses.empty()) {
        return;
    }

    std::size_t nearest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < traj.poses.size(); ++i) {
        const auto& p = traj.poses[i].position;
        const double dx = p.x - reference_.x();
        const double dy = p.y - reference_.y();
        const double dz = use_2d_ ? 0.0 : (p.z - reference_.z());
        const double dist_sq = dx * dx + dy * dy + dz * dz;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = i;
        }
    }

    // Silently no-op: nearest point is already the first waypoint
    if (nearest_idx == 0) {
        return;
    }

    motion_decorator::detail::erase_front(traj, nearest_idx);
}

} // namespace finenav

