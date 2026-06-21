// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"

namespace finenav {

/**
 * @brief Stateless decorator that trims a trajectory so that the cumulative
 *        arc-length along the path does not exceed `max_distance_m`.
 *
 * Distance is computed as the sum of consecutive Euclidean distances between
 * pose positions (x, y, z). Requires the trajectory to have TRAJ_POSE populated.
 *
 * - Silently no-ops if the total path length is already within the limit.
 * - Throws std::invalid_argument if max_distance_m <= 0 at construction time.
 * - Throws std::runtime_error if TRAJ_POSE is not set on the trajectory (programmer error).
 */
class TrimByDistance : public IMotionDecorator {
public:
    /** @brief Construct a TrimByDistance decorator.
     *  @param max_distance_m  Maximum cumulative arc-length to keep (metres, must be > 0).
     *  @throws std::invalid_argument if max_distance_m <= 0.
     */
    explicit TrimByDistance(double max_distance_m);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    double max_distance_m_;
};

} // namespace finenav

