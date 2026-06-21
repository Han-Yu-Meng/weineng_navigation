// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "trim_by_count.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>

namespace finenav {

TrimByCount::TrimByCount(std::size_t max_points) : max_points_(max_points)
{
    if (max_points_ == 0) {
        throw std::invalid_argument("TrimByCount: max_points must be > 0");
    }
}

void TrimByCount::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    // Determine current size from the first populated field
    std::size_t current_size = 0;
    if      (traj.valid_fields & T::TRAJ_POSE) current_size = traj.poses.size();
    else if (traj.valid_fields & T::TRAJ_VEL)  current_size = traj.twists.size();
    else if (traj.valid_fields & T::TRAJ_TIME) current_size = traj.time_from_start.size();

    // Silently no-op: already within the limit or trajectory is empty
    if (current_size <= max_points_) {
        return;
    }

    motion_decorator::detail::resize_trajectory(traj, max_points_);
}

} // namespace finenav
