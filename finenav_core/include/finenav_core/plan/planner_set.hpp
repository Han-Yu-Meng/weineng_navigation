// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <tuple>
#include <variant>
#include <memory>

namespace finenav {

// ==============================================================================
// PlannerSet — 声明一组规划器类型，供 ControlLayer 枚举
// ==============================================================================

template <typename... Ts>
struct PlannerSet {
    using Tuple = std::tuple<Ts...>;
};

// ==============================================================================
// GenerateMapViewsVariant — 从 PlannerSet 推导地图视图 variant 类型
// ==============================================================================

template <typename PlannerT>
struct GetMapViewSharedPtr {
    using type = std::shared_ptr<typename PlannerT::IMapView>;
};

template <typename Tuple> struct ToVariant;

template <typename... Planners>
struct ToVariant<std::tuple<Planners...>> {
    using type = std::variant<typename GetMapViewSharedPtr<Planners>::type...>;
};

/**
 * @brief 从 PlannerSet 中提取每个规划器的 IMapView，生成 std::variant<shared_ptr<...>...>。
 *
 * 用于 ControlLayer::BoundMapView::view_variant 成员的类型推导。
 */
template <typename PlannerSetT>
using GenerateMapViewsVariant = typename ToVariant<typename PlannerSetT::Tuple>::type;

} // namespace finenav

