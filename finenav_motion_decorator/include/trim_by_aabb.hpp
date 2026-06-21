// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"
#include <Eigen/Geometry>

namespace finenav {

/**
 * @brief Stateless decorator that trims a trajectory by discarding all waypoints
 *        that lie outside an axis-aligned bounding box (AABB).
 *
 * The decorator scans the trajectory from the front and retains only the
 * contiguous prefix of waypoints that are contained within the box.
 * The first waypoint found outside the box — and all following waypoints —
 * are removed.
 *
 * Although the bounding box is 3-D (Eigen::AlignedBox3d), a 2-D XY-only
 * check can be achieved by passing infinite Z extents:
 * @code
 *   Eigen::AlignedBox3d box;
 *   box.min() = {map_min_x, map_min_y, -std::numeric_limits<double>::infinity()};
 *   box.max() = {map_max_x, map_max_y,  std::numeric_limits<double>::infinity()};
 * @endcode
 *
 * Requires TRAJ_POSE to be populated.
 *
 * - Silently no-ops if all waypoints are inside the box.
 * - Throws std::runtime_error if TRAJ_POSE is not set (programmer error).
 */
class TrimByAABB : public IMotionDecorator {
public:
    /**
     * @brief Construct a TrimByAABB decorator.
     * @param bounds  3-D axis-aligned bounding box. Waypoints outside this box
     *                (first occurrence onwards) are removed.
     * @throws std::invalid_argument if bounds is empty (min > max on any axis).
     */
    explicit TrimByAABB(Eigen::AlignedBox3d bounds);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    Eigen::AlignedBox3d bounds_;
};

} // namespace finenav

