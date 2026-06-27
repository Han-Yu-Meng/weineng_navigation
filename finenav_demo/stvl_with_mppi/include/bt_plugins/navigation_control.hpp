// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "finenav_navigator/thirdparty/ros_node_params.hpp"

namespace finenav {

/**
 * @brief NavigationControl — 处理 /stop_navigation 和 /start_navigation 服务的 BT 条件节点。
 *
 * 作为 BT::ConditionNode，每次 tick 检查导航是否启用。
 * 如果导航被停止，返回 FAILURE；如果导航启用，返回 SUCCESS。
 *
 * 服务:
 *   - /stop_navigation (std_srvs::srv::Trigger): 停止导航
 *   - /start_navigation (std_srvs::srv::Trigger): 启动导航
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
    void stopNavigationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void startNavigationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

    std::atomic<bool> navigation_enabled_{true};
    bool initialized_{false};
};

} // namespace finenav
