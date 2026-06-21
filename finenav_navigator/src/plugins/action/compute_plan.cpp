// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/compute_plan.hpp"

namespace finenav {

ComputePlanAction::ComputePlanAction(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosActionNode<finenav_msgs::action::ComputePlan>(name, conf, params) {}

bool ComputePlanAction::setGoal(Goal& goal) {
    if (!getInput("goal", goal.goal) ||
        !getInput("planner_id", goal.planner_id)) {
        RCLCPP_ERROR(logger(), "ComputePlan: missing input ports");
        return false;
    }
    return true;
}
// TODO: different action server for different control layer?
BT::NodeStatus ComputePlanAction::onResultReceived(const WrappedResult& result) {

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        std:: cout << "ComputePlanAction of " << rclcpp_action::to_string(result.goal_id) << " succeeded." << std::endl;
        // TODO: turn plannng_time into xxx.xxx seconds
        result.result->planning_time;

        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

}

#include "finenav_navigator/thirdparty/plugins.hpp"
CreateRosNodePlugin(finenav::ComputePlanAction, "ComputePlan");
