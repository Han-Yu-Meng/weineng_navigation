// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_msgs/msg/trajectory.hpp"

namespace finenav {

/**
 * @brief Interface for stateless motion decorators.
 * Modifies the trajectory in-place to avoid copies.
 */
class IMotionDecorator {
public:
    virtual ~IMotionDecorator() = default;

    /**
     * @brief Apply the decorator to the trajectory.
     * @param trajectory The trajectory to be modified in-place.
     * @note Only throw when there is lethal failure
     */
    virtual void operator()(finenav_msgs::msg::Trajectory& trajectory) const = 0;
};

/**
 * @brief Enable pipe syntax for decorators: traj | DecoratorA() | DecoratorB()
 */
inline finenav_msgs::msg::Trajectory& operator|(
    finenav_msgs::msg::Trajectory& trajectory,
    const IMotionDecorator& decorator)
{
    decorator(trajectory);
    return trajectory;
}

} // namespace finenav

