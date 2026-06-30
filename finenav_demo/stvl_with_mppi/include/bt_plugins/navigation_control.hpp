// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <atomic>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "finenav_navigator/thirdparty/ros_node_params.hpp"

namespace finenav {

/**
 * @brief NavigationControl — 订阅 /navigation_enabled 话题的 BT 条件节点。
 *
 * 作为 RosNodePlugin，订阅 /navigation_enabled (std_msgs::msg::Bool)。
 * 每次 tick 检查最新值，true 返回 SUCCESS，false 返回 FAILURE。
 * 默认值 true（导航启用），直到收到第一条消息为止。
 *
 * /navigation_enabled 话题由 goal_pose_bridge_node 的
 * /start_navigation、/stop_navigation 服务更新（transient_local QoS）。
 */
class NavigationControl : public BT::ConditionNode
{
public:
    NavigationControl(const std::string& name,
                      const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params);

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    std::atomic<bool> navigation_enabled_{true};
};

} // namespace finenav
