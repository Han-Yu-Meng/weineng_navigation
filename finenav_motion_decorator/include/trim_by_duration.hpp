// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"

namespace finenav {

/**
 * @brief Stateless decorator that trims a trajectory to only waypoints whose
 *        time_from_start is strictly less than or equal to `max_duration_sec`.
 *
 * Requires the trajectory to have TRAJ_TIME populated.
 *
 * - Silently no-ops if the trajectory is already within the limit.
 * - Throws std::invalid_argument if max_duration_sec <= 0 at construction time.
 * - Throws std::runtime_error if TRAJ_TIME is not set on the trajectory (programmer error).
 */
class TrimByDuration : public IMotionDecorator {
public:
    /** @brief Construct a TrimByDuration decorator.
     *  @param max_duration_sec  Maximum time-from-start to keep (seconds, must be > 0).
     *  @throws std::invalid_argument if max_duration_sec <= 0.
     */
    explicit TrimByDuration(double max_duration_sec);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    double max_duration_sec_;
};

} // namespace finenav

