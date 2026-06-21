// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/concepts.hpp" // TODO: 为什么这个文件也必不可少
#include "finenav_util/algo_configurator.hpp"
#include "finenav_interface/test_planner_params.hpp"

namespace finenav {
class SimpleTestPlanner {
public:
    SimpleTestPlanner() = default;

    using ConfigType = test_planner::Params;
    static constexpr IOProfile InputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };
    static constexpr IOProfile OutputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };

    class IMapView {
    public:
        virtual ~IMapView() = default;
        virtual double getCost(const int x) const = 0;
    };

    using Context = PlanningContext<SimpleTestPlanner>;
    using Result  = std::optional<OutputDataT<SimpleTestPlanner>>;

    void configure(const ConfigType &params) {
        if (params.background.b == 255) { /* 复杂的分支控制逻辑 */}
    }

    void reset() {
        // SimpleTestPlanner 无跨 Episode 状态，空实现即可
    }

    Result plan(const Context& ctx, const IMapView& map_view) {
        // 当前机器人状态（位姿 + 速度），由框架通过 RobotStateProvider 注入
        const auto& robot_state = ctx.robot_state;
        const auto& start_pose  = robot_state.pose;   // geometry_msgs::msg::Pose
        const auto& start_twist = robot_state.twist;  // geometry_msgs::msg::Twist

        auto goal_pose = ctx.goal_pose;
        auto ref_traj  = ctx.ref_traj;

        // 执行规划算法，e.g. 生成一条直线路径

        // 装填输出轨迹
        return std::nullopt;
    }
};
}

FINENAV_REGISTER_ALGO_CONFIG(finenav::SimpleTestPlanner, test_planner)