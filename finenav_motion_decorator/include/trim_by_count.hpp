// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"
#include <cstddef>

namespace finenav {

/**
 * @brief Stateless decorator that trims a trajectory to at most `max_points`
 *        waypoints, counting from the beginning of the path.
 *
 * All populated fields (poses, twists, accels, time_from_start) are resized
 * consistently according to the trajectory's valid_fields mask.
 *
 * - Silently no-ops when the trajectory already has <= max_points waypoints.
 * - Throws std::invalid_argument if max_points == 0 at construction time.
 */
class TrimByCount : public IMotionDecorator {
public:
    /** @brief Construct a TrimByCount decorator.
     *  @param max_points  Maximum number of waypoints to keep (must be > 0).
     *  @throws std::invalid_argument if max_points == 0.
     */
    explicit TrimByCount(std::size_t max_points);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    std::size_t max_points_;
};

} // namespace finenav
