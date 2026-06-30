// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "bt_plugins/navigation_control.hpp"
#include "finenav_navigator/thirdparty/plugins.hpp"

namespace finenav {

NavigationControl::NavigationControl(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::ConditionNode(name, conf), node_(params.nh)
{
    // 订阅 /navigation_enabled 话题，使用 transient_local 确保晚加入也能收到最后一条消息
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/navigation_enabled",
        rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            navigation_enabled_.store(msg->data);
        });
}

BT::NodeStatus NavigationControl::tick()
{
    return navigation_enabled_.load()
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
}

} // namespace finenav

CreateRosNodePlugin(finenav::NavigationControl, "NavigationControl");
