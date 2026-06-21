// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "trim_by_duration.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>
#include <algorithm>

namespace finenav {

TrimByDuration::TrimByDuration(double max_duration_sec) : max_duration_sec_(max_duration_sec)
{
    if (max_duration_sec_ <= 0.0) {
        throw std::invalid_argument("TrimByDuration: max_duration_sec must be > 0");
    }
}

void TrimByDuration::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    if (!(traj.valid_fields & T::TRAJ_TIME)) {
        throw std::runtime_error(
            "TrimByDuration: trajectory does not have TRAJ_TIME populated");
    }

    if (traj.time_from_start.empty()) {
        return;  // nothing to trim
    }

    // Find the first index whose time_from_start exceeds the limit
    std::size_t keep = traj.time_from_start.size();
    for (std::size_t i = 0; i < traj.time_from_start.size(); ++i) {
        if (motion_decorator::detail::duration_to_sec(traj.time_from_start[i]) > max_duration_sec_) {
            keep = i;
            break;
        }
    }

    // Silently no-op: all points are already within the limit
    if (keep >= traj.time_from_start.size()) {
        return;
    }

    motion_decorator::detail::resize_trajectory(traj, keep);
}

} // namespace finenav

