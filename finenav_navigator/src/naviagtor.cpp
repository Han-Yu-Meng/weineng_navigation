// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/navigator.hpp"
#include <geometry_msgs/msg/pose.hpp>

namespace finenav {

Navigator::Navigator(rclcpp::Node::SharedPtr parent_node)
{
    // ── 创建私有子节点（参数前缀由 yaml 的顶层 key "finenav_navigator" 决定）──
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        parent_node->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_navigator",            // 节点名称，与 yaml 顶层 key 一致
        parent_node->get_name(),        // 命名空间继承宿主节点名
        pnode_opts);

    // ── 读取参数 ──────────────────────────────────────────────────────────────
    param_listener_ = std::make_unique<finenav_navigator::ParamListener>(pnode_);
    params_ = param_listener_->get_params();

    // ── 创建 Action Server ────────────────────────────────────────────────────
    action_server_ = std::make_unique<ActionServer>(
        pnode_,
        params_.action_name,
        std::bind(&Navigator::execute, this));

    // ── 初始化行为树引擎 ──────────────────────────────────────────────────────
    bt_engine_ = std::make_unique<BehaviorTreeEngine>(pnode_);
    if (!params_.plugins.empty())       { bt_engine_->registerPlugins(params_.plugins); }
    if (!params_.behavior_trees.empty()){ bt_engine_->registerBehaviorTrees(params_.behavior_trees); }

    // ── 启动 spin 线程（最后执行，确保所有订阅/服务已注册）────────────────────
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Initialized and spinning.");
}

void Navigator::execute()
{
    RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Executing action...");

    auto goal = action_server_->get_current_goal();

    try {
        tree_ = bt_engine_->factory().createTree(goal->behavior_tree);
    } catch (std::exception& ex) {
        RCLCPP_ERROR(pnode_->get_logger(),
            "[Navigator] Failed to create tree [%s]: %s",
            goal->behavior_tree.c_str(), ex.what());
        action_server_->terminate_all();
        return;
    }

    auto blackboard = tree_.rootBlackboard();
    blackboard->set("goal", goal->pose.pose);

    rclcpp::WallRate loop_rate(params_.tick_frequency);
    RCLCPP_INFO(pnode_->get_logger(),
        "[Navigator] Starting navigation to (%.2f, %.2f)",
        goal->pose.pose.position.x, goal->pose.pose.position.y);

    while (rclcpp::ok()) {
        if (action_server_->is_cancel_requested()) {
            RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Cancellation requested.");
            tree_.haltTree();
            action_server_->terminate_all();
            return;
        }

        if (action_server_->is_preempt_requested()) {
            RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Preemption requested.");
            tree_.haltTree();
            return;
        }

        // 每 tick 前将最新机器人位姿和速度注入 Blackboard，供条件节点读取
        if (robot_pose_provider_) {
            blackboard->set<geometry_msgs::msg::Pose>("robot_pose", robot_pose_provider_());
        }
        if (robot_twist_provider_) {
            blackboard->set<geometry_msgs::msg::Twist>("robot_twist", robot_twist_provider_());
        }

        auto status = tree_.tickExactlyOnce();

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Navigation succeeded.");
            action_server_->succeeded_current();
            return;
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(pnode_->get_logger(), "[Navigator] Navigation failed.");
            action_server_->terminate_all();
            return;
        }

        // TODO: 发布 Feedback

        loop_rate.sleep();
    }
}

} // namespace finenav
