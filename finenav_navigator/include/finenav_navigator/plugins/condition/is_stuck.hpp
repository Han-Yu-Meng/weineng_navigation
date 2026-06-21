// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav {

/**
 * @brief IsStuck — 检测机器人是否在一段时间内没有充分移动（卡死）。
 *
 * Blackboard 输入:
 *   - robot_pose (geometry_msgs::msg::Pose): 当前机器人位姿
 *
 * 输入端口:
 *   - dist_threshold (double, 默认 0.05 m): 时间窗内最小位移阈值
 *   - timeout_s      (double, 默认 5.0 s):  触发卡死判定的时间窗长度
 *
 * 返回 SUCCESS 表示已卡死，FAILURE 表示正常移动中。
 * halt() 重置所有内部状态（适用于恢复行为后重新检测）。
 */
// TODO: 可能需要重构，需要参考BT4库提供的文档
// Claude的输出
// BT::ConditionNode::halt() 是 final 的，不能被覆写。改用 SimpleConditionNode 的机制，或通过 setPreTickFunction 替代。实际上 BT4
// 的 ConditionNode 不支持自定义 halt，要用 StatefulActionNode 或继承 BT::ConditionNode 并在构造时注册。
// 最干净的解法：将 IsStuck 改为继承 BT::SimpleConditionNode 的函数对象，或用 BT::ConditionNode 并把状态重置放在 tick 逻辑里
// （检测到自身被重新激活时）。由于 halt 不可覆写，IsStuck 改为在每次 tick 时检测自己是否是首次被 tick（通过时间戳或标志位的特殊值）：

class IsStuck : public BT::ConditionNode {
public:
    IsStuck(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config), ref_set_(false) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Pose>("robot_pose",      "{robot_pose}", "Current robot pose"),
            BT::InputPort<double>("dist_threshold", 0.05,  "Minimum displacement [m] within window"),
            BT::InputPort<double>("timeout_s",      5.0,   "Time window [s] for stuck detection"),
        };
    }

    BT::NodeStatus tick() override {
        geometry_msgs::msg::Pose robot_pose;
        if (!getInput("robot_pose", robot_pose)) {
            return BT::NodeStatus::FAILURE;
        }
        double dist_threshold = 0.05;
        double timeout_s      = 5.0;
        getInput("dist_threshold", dist_threshold);
        getInput("timeout_s",      timeout_s);

        const double cur_x = robot_pose.position.x;
        const double cur_y = robot_pose.position.y;
        const auto now = std::chrono::steady_clock::now();

        // BT::ConditionNode::halt() is final — detect "cold start" by checking
        // whether the node was last ticked more than timeout_s ago (i.e. it was
        // dormant / freshly reactivated).  In that case, reset the window.
        if (ref_set_) {
            const double gap =
                std::chrono::duration<double>(now - last_tick_time_).count();
            if (gap > timeout_s) {
                ref_set_ = false; // dormant long enough — treat as fresh start
            }
        }
        last_tick_time_ = now;

        if (!ref_set_) {
            ref_x_        = cur_x;
            ref_y_        = cur_y;
            window_start_ = now;
            ref_set_      = true;
            return BT::NodeStatus::FAILURE;
        }

        const double dx    = cur_x - ref_x_;
        const double dy    = cur_y - ref_y_;
        const double moved = std::sqrt(dx * dx + dy * dy);

        if (moved > dist_threshold) {
            ref_x_        = cur_x;
            ref_y_        = cur_y;
            window_start_ = now;
            return BT::NodeStatus::FAILURE;
        }

        const double elapsed =
            std::chrono::duration<double>(now - window_start_).count();
        return (elapsed > timeout_s) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    bool   ref_set_{false};
    double ref_x_{0.0}, ref_y_{0.0};
    std::chrono::steady_clock::time_point window_start_;
    std::chrono::steady_clock::time_point last_tick_time_;
};

} // namespace finenav


