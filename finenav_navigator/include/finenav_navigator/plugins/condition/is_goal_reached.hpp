// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav {

/**
 * @brief IsGoalReached — 检查机器人是否已到达目标（位置 + 速度双重判定）。
 *
 * Blackboard 输入:
 *   - robot_pose  (geometry_msgs::msg::Pose):  当前机器人位姿
 *   - robot_twist (geometry_msgs::msg::Twist): 当前机器人速度
 *   - goal        (geometry_msgs::msg::Pose):  导航目标位姿
 *
 * 输入端口:
 *   - tolerance        (double, 默认 0.25 m):   到达判定的距离阈值
 *   - speed_tolerance  (double, 默认 0.1 m/s):  到达判定的合速度阈值
 *                      设为 0.0 可禁用速度检查（仅判断位置）
 *
 * 仅当位置距离 < tolerance 且合速度 < speed_tolerance 时返回 SUCCESS。
 */
class IsGoalReached : public BT::ConditionNode {
public:
    IsGoalReached(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Pose> ("robot_pose",      "{robot_pose}",  "Current robot pose"),
            BT::InputPort<geometry_msgs::msg::Twist>("robot_twist",     "{robot_twist}", "Current robot twist"),
            BT::InputPort<geometry_msgs::msg::Pose> ("goal",            "{goal}",        "Goal pose"),
            BT::InputPort<double>("tolerance",       0.25, "Goal position tolerance [m]"),
            BT::InputPort<double>("speed_tolerance", 0.1,  "Goal speed tolerance [m/s]; set 0 to disable"),
        };
    }

    BT::NodeStatus tick() override {
        geometry_msgs::msg::Pose robot_pose, goal;
        geometry_msgs::msg::Twist robot_twist;
        double tolerance       = 0.25;
        double speed_tolerance = 0.1;

        if (!getInput("robot_pose", robot_pose) || !getInput("goal", goal)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("tolerance",       tolerance);
        getInput("speed_tolerance", speed_tolerance);

        // 位置检查
        const double dx   = robot_pose.position.x - goal.position.x;
        const double dy   = robot_pose.position.y - goal.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist >= tolerance) {
            return BT::NodeStatus::FAILURE;
        }

        // 速度检查（speed_tolerance <= 0 时跳过）
        if (speed_tolerance > 0.0) {
            if (!getInput("robot_twist", robot_twist)) {
                // 速度未注入时降级为仅位置判断
                return BT::NodeStatus::SUCCESS;
            }
            const double vx    = robot_twist.linear.x;
            const double vy    = robot_twist.linear.y;
            const double speed = std::sqrt(vx * vx + vy * vy);
            if (speed >= speed_tolerance) {
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::SUCCESS;
    }
};

} // namespace finenav

