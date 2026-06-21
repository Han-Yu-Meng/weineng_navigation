// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "astar_path_search.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────

void AstarPathSearch::configure(const ConfigType& params)
{
    // connectivity encodes both dimension and neighbourhood size
    if      (params.connectivity == "4")  connectivity_ = astar::AStarAlgorithm::Connectivity::N4;
    else if (params.connectivity == "8")  connectivity_ = astar::AStarAlgorithm::Connectivity::N8;
    else if (params.connectivity == "6")  connectivity_ = astar::AStarAlgorithm::Connectivity::N6;
    else if (params.connectivity == "18") connectivity_ = astar::AStarAlgorithm::Connectivity::N18;
    else                                  connectivity_ = astar::AStarAlgorithm::Connectivity::N26;

    if      (params.heuristic == "Manhattan")  heuristic_ = astar::Heuristic::Manhattan;
    else if (params.heuristic == "Chebyshev")  heuristic_ = astar::Heuristic::Chebyshev;
    else                                        heuristic_ = astar::Heuristic::Euclidean;

    goal_tolerance_m_  = params.goal_tolerance;
    max_iterations_    = params.max_iterations;
    max_planning_time_ = params.max_planning_time;
    movement_weight_   = params.movement_weight;
    greedy_weight_     = params.greedy_weight;

    use_empty_map_        = params.use_empty_map;
    empty_map_resolution_ = params.empty_map_resolution;
}

// ── plan ───────────────────────────────────────────────────────────────────

AstarPathSearch::Result AstarPathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    // ── empty map mode: use an all-walkable map instead of the real one ───
    astar::EmptyMapView empty_map(empty_map_resolution_);
    const IMapView& active_view = use_empty_map_
        ? static_cast<const IMapView&>(empty_map)
        : map_view;

    // ── read step size from the map (single source of truth) ──────────────
    const double step = active_view.getStepSize();

    // ── world ↔ cell helpers ───────────────────────────────────────────────
    //
    // Origin is always (0,0,0) in FineNav (WORLD_FIXED_FRAME).
    // Cell centre = (ix + 0.5) * step  for each axis.

    auto worldToCell = [step](double x, double y, double z) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / step)),
            static_cast<int>(std::floor(y / step)),
            static_cast<int>(std::floor(z / step))
        };
    };

    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return { (c.x + 0.5) * step,
                 (c.y + 0.5) * step,
                 (c.z + 0.5) * step };
    };

    // ── lambdas injected into AStarAlgorithm ──────────────────────────────

    auto is_walkable = [&](const astar::Vec3& c) -> bool {
        return active_view.isWalkable(cellCentre(c));
    };
    auto get_cost = [&](const astar::Vec3& c) -> double {
        return active_view.getCost(cellCentre(c));
    };

    // ── run A* in grid space ───────────────────────────────────────────────

    const auto& sp = ctx.robot_state.pose.position;
    const auto& gp = ctx.goal_pose.position;

    astar::Vec3 start = worldToCell(sp.x, sp.y, sp.z);
    astar::Vec3 goal  = worldToCell(gp.x, gp.y, gp.z);

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(connectivity_);
    algo.setHeuristic(heuristic_);
    algo.setGoalTolerance(goal_tolerance_m_ / step);  // metres → cells
    algo.setMaxIterations(max_iterations_);
    algo.setMaxPlanningTime(max_planning_time_);

    auto cell_path = algo.findPath(start, goal);
    if (!cell_path)
        return std::nullopt;

    // ── convert cell path → world poses ───────────────────────────────────

    OutputDataT<AstarPathSearch> output;
    output.poses.reserve(cell_path->size());

    for (const auto& cell : *cell_path)
    {
        Position3D w = cellCentre(cell);

        geometry_msgs::msg::Pose p;
        p.position.x    = w.x();
        p.position.y    = w.y();
        p.position.z    = w.z();
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    }

    return output;
}

} // namespace finenav

