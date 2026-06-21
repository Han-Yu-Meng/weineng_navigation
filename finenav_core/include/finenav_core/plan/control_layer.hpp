// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_msgs/msg/robot_state.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_core/concepts.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/plan_context.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/planner_set.hpp"
#include "finenav_core/plan/ros_conversions.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_util/simple_action_server.hpp"
#include "finenav_util/node_thread.hpp"

namespace finenav {

// ==============================================================================
// ControlLayer Declaration
// ==============================================================================

/**
 * @brief ControlLayer — 将 Planner、MapView 与执行策略绑定为一个规划层。
 *
 * @tparam PlannersT      PlannerSet<...>，枚举所有支持的规划器类型。
 * @tparam ExecutionPolicy  执行策略类型，默认 SingleShotPolicy。
 *                          支持扩展：新增策略只需定义 policy struct + 添加对应 executeImpl() 重载。
 */
template <typename PlannersT, typename ExecutionPolicy = SingleShotPolicy>
class ControlLayer {
public:
    using MapViewsVariant = GenerateMapViewsVariant<PlannersT>;
    using ActionT         = finenav_msgs::action::ComputePlan;
    using ActionServer    = SimpleActionServer<ActionT>;

    /**
     * @brief BoundMapView — 地图视图 + RAII 读锁的聚合体。
     *
     * lock_guard 持有 LockedMapRO<MapT> 的 shared_ptr，保证在 planner.plan() 整个
     * 调用期间读锁不被释放；view_variant 是规划器实际读取的视图指针。
     */
    struct BoundMapView {
        MapViewsVariant view_variant;
        std::shared_ptr<void> lock_guard; ///< 持有 LockedMapRO<MapT>，离开作用域后自动释放读锁
    };

    using MapViewFactory      = std::function<BoundMapView()>;
    using PrePlanCallback     = std::function<bool(PrePlanContext&)>;
    using PostPlanCallback    = std::function<bool(PostPlanContext&)>;
    using RobotStateProvider  = std::function<finenav_msgs::msg::RobotState()>;

    explicit ControlLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& layer_name,
        ExecutionPolicy policy = ExecutionPolicy{});

    void setInputTopic(const std::string& topic);
    void setOutputTopic(const std::string& topic);
    std::string getName() const;
    void setupDataPorts();

    /**
     * @brief 将 MapServer 与规划器所需的地图视图绑定。
     *
     * 框架在每次调用 planner.plan() 前自动通过 server 申请读锁，并将加锁后的
     * const MapT& 传给 builder 以构造 IMapView；读锁在 plan() 返回后自动释放。
     *
     * @tparam MapT      地图类型，从 server 参数自动推导，必须满足 MapConcept
     * @tparam BuilderFn 任意可调用类型，签名：(const MapT&) -> MapViewsVariant-compatible
     * @param server     提供地图数据的 MapServer（弱引用持有）
     * @param builder    适配函数：接收加锁的 const MapT&，返回可构造 MapViewsVariant 的值
     */
    template <MapConcept MapT, typename BuilderFn>
    requires std::is_invocable_v<BuilderFn, const MapT&> &&
             std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&>, MapViewsVariant>
    void bind_map_view(std::shared_ptr<MapServer<MapT>> server, BuilderFn&& builder);

    void set_on_pre_plan(PrePlanCallback cb)               { on_pre_plan_           = std::move(cb); }
    void set_on_post_plan(PostPlanCallback cb)             { on_post_plan_          = std::move(cb); }
    void set_robot_state_provider(RobotStateProvider prov) { robot_state_provider_  = std::move(prov); }


    const std::string& get_active_planner_id() const { return active_planner_id_; }

protected:
    MapViewFactory      map_view_factory_;
    PrePlanCallback     on_pre_plan_;
    PostPlanCallback    on_post_plan_;
    RobotStateProvider  robot_state_provider_;

    rclcpp::Node::SharedPtr      node_;
    rclcpp::Node::SharedPtr      pnode_;
    std::unique_ptr<NodeThread>  pnode_thread_;

    typename PlannersT::Tuple planners_instance_;

    std::string     active_planner_id_;
    ExecutionPolicy policy_;

private:
    // ---- Utilities ----
    template <typename T>
    void keep_alive(std::shared_ptr<T> handle);

    void PlanRefCallback(const finenav_msgs::msg::Trajectory& msg);

    // ---- Episode entry-point ----
    void execute();

    // ---- Policy dispatch — add new overload to support new execution mode ----
    void executeImpl(SingleShotPolicy& policy);
    void executeImpl(TrackingPolicy&   policy);

    // ---- Shared helpers (used by both policies) ----

    /** @brief 对匹配 active_planner_id_ 的规划器调用 reset()，在 Episode 边界处触发。 */
    void resetActivePlanner();

    /**
     * @brief 执行一次完整规划步骤：pre-cb → ctx → 地图锁 → plan() → post-cb → 发布。
     * @return true 表示规划成功并发布了结果。
     */
    bool planOnce(geometry_msgs::msg::Pose goal_pose);

    // ---- Data ----
    std::string layer_name_;
    std::string input_topic_;
    std::string output_topic_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<ActionServer> action_server_;

    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr plan_ref_sub_;
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr    plan_result_pub_;

    std::mutex                       plan_ref_mutex_;
    finenav_msgs::msg::Trajectory    plan_ref_msg_;

    std::vector<std::shared_ptr<void>> param_handles_; ///< 防止参数回调句柄被析构
};

} // namespace finenav

#include "finenav_core/plan/control_layer_impl.hpp"
