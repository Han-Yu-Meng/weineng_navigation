// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "finenav_navigator/behavior_tree_engine.hpp"

#include "finenav_util/simple_action_server.hpp"
#include "finenav_util/node_thread.hpp"
#include "finenav_msgs/action/navigate_to_pose.hpp"

#include "finenav_navigator/navigator_params.hpp"

namespace finenav {

/**
 * @brief Navigator — 行为树驱动的任务级导航器。
 *
 * 遵循框架 NodeThread 设计模式：
 *  - 内部持有私有子节点 pnode_（名称固定为 "finenav_navigator"）
 *  - pnode_ 由 NodeThread 在独立线程中 spin，对外部调用方完全透明
 *  - 构造函数完成全部初始化，无需二阶段 initialize()
 *
 * 使用前须调用 set_robot_state_provider() 注入位姿提供函数；
 * FineNavEngine::createControlLayer() 会自动完成注入。
 *
 * @param parent_node  宿主节点（用于获取 use_sim_time 及命名空间继承）
 */
class Navigator {
public:
    using ActionT            = finenav_msgs::action::NavigateToPose;
    using ActionServer       = SimpleActionServer<ActionT>;
    using RobotPoseProvider  = std::function<geometry_msgs::msg::Pose()>;
    using RobotTwistProvider = std::function<geometry_msgs::msg::Twist()>;

    explicit Navigator(rclcpp::Node::SharedPtr parent_node);

    // 禁止复制
    Navigator(const Navigator&)            = delete;
    Navigator& operator=(const Navigator&) = delete;

    ~Navigator() = default;

    /**
     * @brief 注入机器人位姿提供函数。
     *
     * 每次 BT tick 前调用，将最新位姿写入 Blackboard ("robot_pose")，
     * 供 IsGoalReached、IsStuck 等条件节点读取。
     * 由 FineNavEngine 在构造时自动注入。
     */
    void set_robot_pose_provider(RobotPoseProvider provider) {
        robot_pose_provider_ = std::move(provider);
    }

    /**
     * @brief 注入机器人速度提供函数。
     *
     * 每次 BT tick 前调用，将最新速度写入 Blackboard ("robot_twist")，
     * 供 IsGoalReached 等条件节点读取。
     * 由 FineNavEngine 在构造时自动注入。
     */
    void set_robot_twist_provider(RobotTwistProvider provider) {
        robot_twist_provider_ = std::move(provider);
    }

private:
    void execute();

    // ── ROS 基础设施 ──────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr       pnode_;
    std::unique_ptr<NodeThread>   pnode_thread_;

    // ── 业务组件 ──────────────────────────────────────────────────────────────
    std::unique_ptr<BehaviorTreeEngine> bt_engine_;
    std::unique_ptr<ActionServer>       action_server_;

    finenav_navigator::Params                         params_;
    std::unique_ptr<finenav_navigator::ParamListener> param_listener_;

    RobotPoseProvider  robot_pose_provider_;
    RobotTwistProvider robot_twist_provider_;

    BT::Tree tree_;
};

} // namespace finenav

