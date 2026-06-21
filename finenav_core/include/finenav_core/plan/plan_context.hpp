// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_msgs/msg/robot_state.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <string>

namespace finenav {

/**
 * @brief PrePlanContext — 规划前回调所携带的上下文。
 *
 * 回调可修改 robot_state / goal_pose / ref_traj，
 * 也可通过返回 false 中止此次规划。
 */
struct PrePlanContext {
    const std::string& active_planner_id;            // [RO] 当前请求的规划器 ID
    finenav_msgs::msg::RobotState& robot_state;      // [RW] 当前机器人状态（可被回调修改）
    geometry_msgs::msg::Pose& goal_pose;             // [RW] 目标位姿
    finenav_msgs::msg::Trajectory& ref_traj;         // [RW] 规划参考轨迹
};

/**
 * @brief PostPlanContext — 规划后回调所携带的上下文。
 *
 * 回调可修改 result_traj，也可通过返回 false 将规划标记为失败。
 */
struct PostPlanContext {
    const std::string& active_planner_id;       // [RO] 当前请求的规划器 ID
    const bool is_success;                      // [RO] 规划结果是否成功
    finenav_msgs::msg::Trajectory& result_traj; // [RW] 规划结果轨迹
};

} // namespace finenav

