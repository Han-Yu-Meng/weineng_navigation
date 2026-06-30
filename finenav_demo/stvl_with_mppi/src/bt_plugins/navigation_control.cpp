// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "bt_plugins/navigation_control.hpp"

namespace finenav {

NavigationControl::NavigationControl(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::ConditionNode(name, conf), node_(params.nh)
{
    // 创建停止服务
    stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/stop_navigation",
        std::bind(&NavigationControl::stopNavigationCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 创建启动服务
    start_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/start_navigation",
        std::bind(&NavigationControl::startNavigationCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    initialized_ = true;

    RCLCPP_INFO(node_->get_logger(),
        "[NavigationControl] Initialized. Services: /stop_navigation, /start_navigation");
}

void NavigationControl::stopNavigationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;  // 未使用

    navigation_enabled_.store(false);

    response->success = true;
    response->message = "Navigation stopped.";

    RCLCPP_INFO(node_->get_logger(), "[NavigationControl] Navigation stopped.");
}

void NavigationControl::startNavigationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;  // 未使用

    navigation_enabled_.store(true);

    response->success = true;
    response->message = "Navigation started.";

    RCLCPP_INFO(node_->get_logger(), "[NavigationControl] Navigation started.");
}

BT::NodeStatus NavigationControl::tick()
{
    if (!initialized_) {
        return BT::NodeStatus::FAILURE;
    }

    // 检查导航是否启用
    if (navigation_enabled_.load()) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

} // namespace finenav

#include "finenav_navigator/thirdparty/plugins.hpp"
CreateRosNodePlugin(finenav::NavigationControl, "NavigationControl");
