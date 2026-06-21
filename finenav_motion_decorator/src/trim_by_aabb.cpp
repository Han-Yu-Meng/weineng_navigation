// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "trim_by_aabb.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>

namespace finenav {

TrimByAABB::TrimByAABB(Eigen::AlignedBox3d bounds)
    : bounds_(std::move(bounds))
{
    if (bounds_.isEmpty()) {
        throw std::invalid_argument("TrimByAABB: bounds must not be empty (min > max on some axis)");
    }
}

void TrimByAABB::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    if (!(traj.valid_fields & T::TRAJ_POSE)) {
        throw std::runtime_error(
            "TrimByAABB: trajectory does not have TRAJ_POSE populated");
    }

    // Find the first waypoint outside the bounding box; keep [0, keep).
    std::size_t keep = traj.poses.size(); // default: keep all
    for (std::size_t i = 0; i < traj.poses.size(); ++i) {
        const auto& p = traj.poses[i].position;
        const Eigen::Vector3d pt(p.x, p.y, p.z);
        if (!bounds_.contains(pt)) {
            keep = i;
            break;
        }
    }

    // Silently no-op: all waypoints are inside the box
    if (keep >= traj.poses.size()) {
        return;
    }

    motion_decorator::detail::resize_trajectory(traj, keep);
}

} // namespace finenav

