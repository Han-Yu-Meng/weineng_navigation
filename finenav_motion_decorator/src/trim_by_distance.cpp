// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "trim_by_distance.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>
#include <cmath>

namespace finenav {

TrimByDistance::TrimByDistance(double max_distance_m) : max_distance_m_(max_distance_m)
{
    if (max_distance_m_ <= 0.0) {
        throw std::invalid_argument("TrimByDistance: max_distance_m must be > 0");
    }
}

void TrimByDistance::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    if (!(traj.valid_fields & T::TRAJ_POSE)) {
        throw std::runtime_error(
            "TrimByDistance: trajectory does not have TRAJ_POSE populated");
    }

    if (traj.poses.size() < 2) {
        return;  // 0 or 1 point — no arc-length to compute, nothing to trim
    }

    // Accumulate arc-length; stop at the first point that would exceed the limit
    double accumulated = 0.0;
    std::size_t keep = traj.poses.size();  // default: keep all

    for (std::size_t i = 1; i < traj.poses.size(); ++i) {
        const auto& prev = traj.poses[i - 1].position;
        const auto& curr = traj.poses[i].position;

        const double dx = curr.x - prev.x;
        const double dy = curr.y - prev.y;
        const double dz = curr.z - prev.z;
        accumulated += std::sqrt(dx * dx + dy * dy + dz * dz);

        if (accumulated > max_distance_m_) {
            keep = i;  // keep points [0, i), i.e. up to but not including i
            break;
        }
    }

    // Silently no-op: total path is within the limit
    if (keep >= traj.poses.size()) {
        return;
    }

    motion_decorator::detail::resize_trajectory(traj, keep);
}

} // namespace finenav

