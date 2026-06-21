// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_msgs/msg/trajectory.hpp"
#include <cstddef>

namespace finenav::motion_decorator::detail {

/**
 * @brief Resize all populated fields of a trajectory to new_size consistently.
 *        Caller is responsible for ensuring new_size <= current size.
 */
inline void resize_trajectory(finenav_msgs::msg::Trajectory& traj, std::size_t new_size)
{
    using T = finenav_msgs::msg::Trajectory;
    if (traj.valid_fields & T::TRAJ_POSE)  traj.poses.resize(new_size);
    if (traj.valid_fields & T::TRAJ_VEL)   traj.twists.resize(new_size);
    if (traj.valid_fields & T::TRAJ_ACCEL) traj.accels.resize(new_size);
    if (traj.valid_fields & T::TRAJ_TIME)  traj.time_from_start.resize(new_size);
}

/**
 * @brief Convert a builtin_interfaces::msg::Duration to seconds (double).
 */
inline double duration_to_sec(const builtin_interfaces::msg::Duration& d)
{
    return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

/**
 * @brief Erase the first n elements from all populated fields of a trajectory.
 *        Caller is responsible for ensuring n <= current size.
 */
inline void erase_front(finenav_msgs::msg::Trajectory& traj, std::size_t n)
{
    using T = finenav_msgs::msg::Trajectory;
    if (n == 0) return;
    auto erase_n = [n](auto& vec) {
        vec.erase(vec.begin(), vec.begin() + static_cast<std::ptrdiff_t>(n));
    };
    if (traj.valid_fields & T::TRAJ_POSE)  erase_n(traj.poses);
    if (traj.valid_fields & T::TRAJ_VEL)   erase_n(traj.twists);
    if (traj.valid_fields & T::TRAJ_ACCEL) erase_n(traj.accels);
    if (traj.valid_fields & T::TRAJ_TIME)  erase_n(traj.time_from_start);
}

} // namespace finenav::motion_decorator::detail

