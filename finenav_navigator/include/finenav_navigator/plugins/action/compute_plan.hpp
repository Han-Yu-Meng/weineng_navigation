// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_navigator/thirdparty/bt_action_node.hpp"


namespace finenav {

/**
 * @brief A behavior tree ActionNode that requests a plan from finenav_msgs::action::ComputePlan action server
 */
class ComputePlanAction : public BT::RosActionNode<finenav_msgs::action::ComputePlan> {
   public:
    ComputePlanAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<geometry_msgs::msg::Pose>("goal", "Target pose for path planning"), // TODO: or PoseStamped?
            BT::InputPort<std::string>("planner_id", "ID of the planner to use")
        });
    }

    bool setGoal(Goal& goal) override;

    BT::NodeStatus onResultReceived(const WrappedResult& result) override;

};

}