// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"
#include <Eigen/Core>

namespace finenav {

/**
 * @brief Stateless decorator that advances the trajectory start to the waypoint
 *        nearest to a given reference position, discarding all preceding points.
 *
 * "Seek" mirrors file-seek semantics: the read head is repositioned to the
 * nearest entry rather than truncating the tail.
 *
 * Nearest-point search is performed in the XY plane by default (use_2d = true),
 * which is more robust on uneven terrain. Pass use_2d = false to use full 3-D
 * Euclidean distance instead.
 *
 * Requires TRAJ_POSE to be populated.
 *
 * - Silently no-ops if the trajectory is empty or the nearest point is already
 *   the first waypoint.
 * - Throws std::runtime_error if TRAJ_POSE is not set (programmer error).
 */
class SeekToNearestPoint : public IMotionDecorator {
public:
    /**
     * @brief Construct a SeekToNearestPoint decorator.
     * @param reference  Reference position in the map frame.
     * @param use_2d     If true (default), only XY distance is used for the
     *                   nearest-point search; Z is ignored.
     */
    explicit SeekToNearestPoint(Eigen::Vector3d reference, bool use_2d = true);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    Eigen::Vector3d reference_;
    bool use_2d_;
};

} // namespace finenav

