// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Dense>
#include <concepts>
#include <type_traits>

namespace finenav {

using Vector3D    = Eigen::Vector3d;
using Position3D  = Eigen::Vector3d;
using Orientation = Eigen::Quaterniond;
using Region3D    = Eigen::AlignedBox3d;
using Pose        = Eigen::Isometry3d;

using Vector   = Eigen::Vector3d;
using Position = Eigen::Vector3d;
using Index    = Eigen::Vector3i;
using Size     = Eigen::Vector3i;
using Length   = Eigen::Vector3d;

/**
 * @brief 碰撞规则 Concept
 * @details 任何可被调用、接受 const Position3D& 并返回 bool 的可调用对象均满足此约束。
 */
template <typename F>
concept CollisionRule =
    std::invocable<F, const Position3D&> &&
    std::same_as<std::invoke_result_t<F, const Position3D&>, bool>;

/**
 * @brief 代价规则 Concept
 * @details 任何可被调用、接受 const Position3D& 并返回 int 的可调用对象均满足此约束。
 */
template <typename F>
concept CostRule =
    std::invocable<F, const Position3D&> &&
    std::same_as<std::invoke_result_t<F, const Position3D&>, int>;

} // namespace finenav
