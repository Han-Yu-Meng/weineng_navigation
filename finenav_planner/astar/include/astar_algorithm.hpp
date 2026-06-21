// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <optional>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>

#include "vec3.hpp"
#include "heuristic.hpp"

namespace astar {

// =============================================================
// AstarNode  —  internal grid search node, always Vec3-based
// =============================================================

struct AstarNode {
    Vec3   pos;
    double g = std::numeric_limits<double>::max();
    double h = 0.0;
    double f = std::numeric_limits<double>::max();
    Vec3   parent{};   // sentinel: parent == pos means this is the start

    AstarNode() = default;
    explicit AstarNode(const Vec3& p) : pos(p) {}
    void updateF() { f = g + h; }
};

struct AstarNodeCompare {
    bool operator()(const AstarNode& a, const AstarNode& b) const
    { return a.f > b.f; }
};

// =============================================================
// AStarAlgorithm
//
// A plain uniform-grid A* engine.
//
// Neighbourhood is controlled by Connectivity:
//   N4  — 2-D,  4 orthogonal neighbours
//   N8  — 2-D,  8 neighbours  (4 ortho + 4 diagonal)
//   N6  — 3-D,  6 face neighbours
//   N18 — 3-D, 18 neighbours  (6 face + 12 edge-diagonal)
//   N26 — 3-D, 26 neighbours  (18 + 8 corner-diagonal)
//
// Step costs are pure geometry — not user-configurable:
//   1-axis step  →  1.0
//   2-axis step  →  √2  ≈ 1.4142
//   3-axis step  →  √3  ≈ 1.7321
// =============================================================

class AStarAlgorithm {
public:

    enum class Connectivity { N4, N8, N6, N18, N26 };

    using IsWalkableFn = std::function<bool(const Vec3&)>;
    using GetCostFn    = std::function<double(const Vec3&)>;

    AStarAlgorithm(IsWalkableFn is_walkable, GetCostFn get_cost);

    // ── setters ───────────────────────────────────────────────
    void setConnectivity(Connectivity c)  { connectivity_      = c; }
    void setHeuristic(Heuristic h)        { heuristic_         = h; }
    void setGoalTolerance(double t)       { goal_tolerance_    = t; }
    void setMaxIterations(int m)          { max_iterations_    = m; }
    void setMaxPlanningTime(double t)     { max_planning_time_ = t; }

    // ── main search ───────────────────────────────────────────
    std::optional<std::vector<Vec3>> findPath(const Vec3& start, const Vec3& goal);

private:
    std::vector<Vec3>  getNeighbors(const Vec3& pos) const;
    static double      calculateStepCost(const Vec3& from, const Vec3& to);
    static double      cellDistance(const Vec3& a, const Vec3& b);
    std::vector<Vec3>  reconstructPath(const Vec3& goal) const;

    // ── injected queries ──────────────────────────────────────
    IsWalkableFn is_walkable_;
    GetCostFn    get_cost_;

    // ── parameters ────────────────────────────────────────────
    Connectivity connectivity_    = Connectivity::N8;
    Heuristic    heuristic_       = Heuristic::Euclidean;
    double       goal_tolerance_  = 0.25;
    int          max_iterations_  = 1000000;
    double       max_planning_time_ = 5.0;
    double       movement_weight_      = 1.0;
    double       greedy_weight_        = 1.001;

    // ── search state (reset each findPath call) ───────────────
    std::priority_queue<AstarNode, std::vector<AstarNode>, AstarNodeCompare> open_set_;
    std::unordered_set<Vec3, Vec3Hash>            visited_;
    std::unordered_map<Vec3, AstarNode, Vec3Hash> came_from_;
};

} // namespace astar

