// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <cmath>

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_planner_astar/astar_params.hpp"

#include "astar_algorithm.hpp"
#include "imap_view.hpp"
#include "empty_map_view.hpp"


namespace finenav {

// =============================================================
// AstarPathSearch
//
// FineNav planner that performs uniform-grid A* path search.
//
// The user implements AstarPathSearch::IMapView (= astar::IMapView)
// to connect their map data.  No knowledge of Vec3, grid cells,
// or A* internals is required.
//
// Usage:
//   class MyMapView : public AstarPathSearch::IMapView {
//       bool   isWalkable(const Position3D& p) const override { ... }
//       double getCost   (const Position3D& p) const override { ... }
//       double getStepSize()                   const override { ... }
//   };
//   planner.plan(ctx, MyMapView{...});
// =============================================================

class AstarPathSearch {
public:
    AstarPathSearch() = default;

    // ── FineNav framework metadata ────────────────────────────

    using ConfigType = astar_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose  = false,
        .vel   = false,
        .accel = false,
        .time  = false
    };
    static constexpr IOProfile OutputProfile = {
        .pose  = true,
        .vel   = false,
        .accel = false,
        .time  = false
    };

    // ── User-facing map interface ─────────────────────────────
    //
    // Defined in imap_view.hpp.  Exposed here via 'using' so
    // users write  AstarPathSearch::IMapView  without needing
    // to know the astar:: namespace.

    using IMapView = astar::IMapView;

    // ── FineNav planner interface ─────────────────────────────

    using Context = PlanningContext<AstarPathSearch>;
    using Result  = std::optional<OutputDataT<AstarPathSearch>>;

    void configure(const ConfigType& params);

    Result plan(const Context& ctx, const IMapView& map_view);

private:

    // Pure A* search parameters — no grid discretisation here.
    // Step size comes from IMapView::getStepSize() at plan time.
    astar::AStarAlgorithm::Connectivity connectivity_   = astar::AStarAlgorithm::Connectivity::N8;
    astar::Heuristic                    heuristic_      = astar::Heuristic::Euclidean;
    double goal_tolerance_m_  = 0.5;    // metres (converted to cells at plan time)
    int    max_iterations_    = 1000000;
    double max_planning_time_ = 5.0;
    double movement_weight_      = 1.0;
    double greedy_weight_        = 1.001;

    // ── empty map mode ─────────────────────────────────────────
    bool   use_empty_map_          = false;
    double empty_map_resolution_   = 0.05;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::AstarPathSearch, astar_planner)
