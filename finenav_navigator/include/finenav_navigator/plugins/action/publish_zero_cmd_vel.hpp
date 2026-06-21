// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include "finenav_navigator/thirdparty/bt_topic_pub_node.hpp"

namespace finenav {

/**
 * @brief PublishZeroCmdVel — 向指定话题发布一条零速度指令（linear/angular 全为 0）。
 *
 * 作为 BT::RosTopicPubNode<Twist> 的同步动作节点，tick 一次即发布并立即返回 SUCCESS。
 * 典型用途：放置在导航成功/卡死检测后的 Sequence 末尾，确保机器人停止运动。
 *
 * 输入端口:
 *   - topic_name (string, 默认 "/cmd_vel"): 要发布的话题名称
 */
class PublishZeroCmdVel : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
    PublishZeroCmdVel(const std::string& name,
                      const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params)
        : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        // 继承 topic_name 端口；这里不添加额外端口
        return providedBasicPorts({});
    }

    bool setMessage(geometry_msgs::msg::Twist& msg) override
    {
        // 零速：全部分量保持默认值 0.0
        msg = geometry_msgs::msg::Twist{};
        return true;
    }
};

} // namespace finenav

