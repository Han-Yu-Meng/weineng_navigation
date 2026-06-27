// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <std_msgs/msg/string.hpp>
#include "finenav_navigator/thirdparty/bt_topic_pub_node.hpp"

namespace finenav {

/**
 * @brief PublishNavigationState — 向 /navigation_state 话题发布导航状态字符串。
 *
 * 作为 BT::RosTopicPubNode<String> 的同步动作节点，tick 一次即发布并立即返回 SUCCESS。
 * 用于在行为树的关键位置广播当前导航状态。
 *
 * 输入端口:
 *   - topic_name (string, 默认 "/navigation_state"): 要发布的话题名称
 *   - state (string): 要发布的状态字符串，可选值:
 *       "IDLE" - 空闲状态
 *       "NAVIGATING" - 导航运行状态
 *       "GOAL_REACHED" - 导航成功状态
 *       "MPPI_OBSTACLE_BLOCKED" - 被障碍物遮挡 MPPI 失败状态
 */
class PublishNavigationState : public BT::RosTopicPubNode<std_msgs::msg::String>
{
public:
    PublishNavigationState(const std::string& name,
                           const BT::NodeConfig& conf,
                           const BT::RosNodeParams& params)
        : BT::RosTopicPubNode<std_msgs::msg::String>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<std::string>("state", "Navigation state string to publish")
        });
    }

    bool setMessage(std_msgs::msg::String& msg) override
    {
        std::string state;
        if (!getInput("state", state)) {
            return false;
        }
        msg.data = state;
        return true;
    }
};

} // namespace finenav
