// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <concepts>
#include <optional>
#include <utility>
#include <Eigen/Dense>

#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_msgs/msg/robot_state.hpp"
#include "finenav_core/plan/data_packet.hpp"

namespace finenav {


using Position3D = Eigen::Vector3d;
using Region3D = Eigen::AlignedBox3d;

/**
 * @brief MapConcept
 * @details
 * Map 被定义为一个具有视窗机制的空间数据容器，M: Postion3D -> DataType
 * * 在外部视角下，核心特性：
 * * 视窗机制: 仅允许操作以当前中心的矩形范围内的有效数据，视窗外的数据被视为失效（视窗可以被定义为无穷大）
 */
template <typename T>
concept MapConcept =
    std::default_initializable<T> &&
    std::movable<T> &&
    requires(T map, const Position3D& pos, const typename T::ConfigType& config, const typename T::DataType& data) {
    // --- 基本配置 ---
    typename T::ConfigType;
    typename T::DataType;
    { map.configure(config) } -> std::same_as<void>; // 配置地图参数

    // --- 视窗操作 ---
    { std::as_const(map).isInside(pos) } -> std::same_as<bool>; // 检查是否在当前视窗内
    { std::as_const(map).getWindowBounds() } -> std::convertible_to<Region3D>; // 获取当前视窗范围
    { std::as_const(map).getWindowCenter() } -> std::convertible_to<Position3D>; // 获取当前视窗中心
    { map.shiftWindowTo(pos) } -> std::same_as<void>;
    };

/**
 * @brief PlanningContext (规划上下文)
 * @details 包含算法在"这一帧"规划所需的所有不可变数据。
 *
 * robot_state 由框架从外部 RobotStateProvider 注入，算法开发者无需关心
 * finenav_localizer 内部的数学表示（NavState<S>），直接使用 geometry_msgs 组合体即可。
 */
template <typename PlannerT>
struct PlanningContext {
    finenav_msgs::msg::RobotState robot_state; ///< 当前机器人状态（pose / twist / accel）
    geometry_msgs::msg::Pose goal_pose;
    InputDataT<PlannerT> ref_traj; // TODO: 验证空轨迹的逻辑
};

/**
 * @brief PlannerConcept TODO：是否考虑改名为 Motion Generator
 * @details
 * Planner 被定义为一个访问地图试图并生成轨迹的算法模块，P: (State, Reference, Map) -> Trajectory
 * * 在外部视角下，核心特性：
 * 1. 特质自省： 暴露它的类型特质，以区分是否需要参考轨迹
 * 2. 不持有地图，只接受地图试图 MapView
 */
template <typename T>
concept PlannerConcept = std::default_initializable<T> &&
    requires {
        typename T::ConfigType;
        typename T::IMapView;
        typename InputDataT<T>;
        typename OutputDataT<T>;
    } &&
    requires(T planner, const typename T::ConfigType& config, const PlanningContext<T>& ctx, const typename T::IMapView& map_view) {

        // --- 基本配置 ---
        { planner.configure(config) } -> std::same_as<void>;

        // --- 重置接口（Episode 边界，由框架在新目标到来时调用）---
        { planner.reset() } -> std::same_as<void>;

        // --- 规划接口 ---
        { planner.plan(ctx, map_view) } -> std::same_as<std::optional<OutputDataT<T>>>;
};


}
