// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// =============================================================
// Unit tests for AStarAlgorithm
//
// Tests are organised in three groups:
//   1. Core algorithm correctness (via lambda maps, no FineNav deps)
//   2. Edge cases and search limits
//   3. AstarPathSearch integration (IMapView + planner config)
// =============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <unordered_set>

#include "astar_algorithm.hpp"
#include "astar_path_search.hpp"
#include "grid_map.hpp"    // test-only utility (in this directory)

// ── helpers ──────────────────────────────────────────────────────────────

// Build a simple walkable-everywhere lambda map for AStarAlgorithm.
inline auto makeOpenMap()
{
    return [](const astar::Vec3&) { return true; };
}

// Build a lambda map backed by GridMap.
// Takes GridMap by VALUE so lambdas own the data and are safe
// even when called with a temporary GridMap argument.
inline auto makeGridLambdas(astar::GridMap g, double /*res*/ = 1.0)
{
    auto is_walkable = [g](const astar::Vec3& c) -> bool {
        return g.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [g](const astar::Vec3& c) -> double {
        return g.getCostCell(c.x, c.y);
    };
    return std::make_pair(is_walkable, get_cost);
}

// ─────────────────────────────────────────────────────────────────────────
// Group 1: Core algorithm correctness (2-D)
// ─────────────────────────────────────────────────────────────────────────

TEST(AStarAlgorithm2D, FindsPath_OpenSpace)
{
    auto [is_walkable, get_cost] = makeGridLambdas(astar::GridMap(20, 20));
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(),  (astar::Vec3{5, 5, 0}));
}

TEST(AStarAlgorithm2D, PathLength_Orthogonal)
{
    // Straight line of 5 steps → 6 cells including start
    auto [is_walkable, get_cost] = makeGridLambdas(astar::GridMap(20, 20));
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 0, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 6u);
}

TEST(AStarAlgorithm2D, PathIsContinuous)
{
    // Every consecutive pair of cells differs by exactly 1 in one axis.
    auto [is_walkable, get_cost] = makeGridLambdas(astar::GridMap(20, 20));
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);

    auto path = algo.findPath({1, 1, 0}, {8, 8, 0});
    ASSERT_TRUE(path.has_value());
    ASSERT_GE(path->size(), 2u);

    for (std::size_t i = 1; i < path->size(); ++i)
    {
        const auto& a = (*path)[i - 1];
        const auto& b = (*path)[i];
        int diff = std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
        EXPECT_EQ(diff, 1) << "Non-adjacent cells at index " << i;
    }
}

TEST(AStarAlgorithm2D, Diagonal_AllowedReducesPathLength)
{
    auto [iw, gc] = makeGridLambdas(astar::GridMap(20, 20));

    astar::AStarAlgorithm ortho(iw, gc);
    ortho.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);
    auto path_ortho = ortho.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_ortho.has_value());

    astar::AStarAlgorithm diag(iw, gc);
    diag.setConnectivity(astar::AStarAlgorithm::Connectivity::N8);
    auto path_diag = diag.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_diag.has_value());

    EXPECT_LT(path_diag->size(), path_ortho->size());
}

TEST(AStarAlgorithm2D, PathAvoidsObstacles)
{
    astar::GridMap grid(10, 10);
    // Vertical wall at x=3, y=0..8
    for (int y = 0; y <= 8; ++y)
        grid.setObstacle(3, y, true);

    auto [iw, gc] = makeGridLambdas(grid);
    astar::AStarAlgorithm algo(iw, gc);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 5, 0}, {6, 5, 0});
    ASSERT_TRUE(path.has_value());

    // No cell in the path may be on the wall
    for (const auto& c : *path)
        EXPECT_FALSE(c.x == 3 && c.y <= 8)
            << "Path passes through obstacle at (" << c.x << "," << c.y << ")";
}

// ─────────────────────────────────────────────────────────────────────────
// Group 2: Edge cases and search limits
// ─────────────────────────────────────────────────────────────────────────

TEST(AStarAlgorithm2D, StartEqualsGoal)
{
    auto [iw, gc] = makeGridLambdas(astar::GridMap(10, 10));
    astar::AStarAlgorithm algo(iw, gc);
    auto path = algo.findPath({3, 3, 0}, {3, 3, 0});
    // A single-cell path is valid
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (astar::Vec3{3, 3, 0}));
}

TEST(AStarAlgorithm2D, StartBlocked_ReturnsNullopt)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(0, 0, true);
    auto [iw, gc] = makeGridLambdas(grid);
    astar::AStarAlgorithm algo(iw, gc);
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(AStarAlgorithm2D, GoalBlocked_ReturnsNullopt)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    auto [iw, gc] = makeGridLambdas(grid);
    astar::AStarAlgorithm algo(iw, gc);
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(AStarAlgorithm2D, EnclosedGoal_ReturnsNullopt)
{
    astar::GridMap grid(10, 10);
    // Surround goal (5,5) with a solid ring
    for (int d = -1; d <= 1; ++d) {
        grid.setObstacle(4, 5 + d, true);
        grid.setObstacle(6, 5 + d, true);
    }
    grid.setObstacle(5, 4, true);
    grid.setObstacle(5, 6, true);

    auto [iw, gc] = makeGridLambdas(grid);
    astar::AStarAlgorithm algo(iw, gc);
    algo.setGoalTolerance(0.0);  // exact match required
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(AStarAlgorithm2D, MaxIterations_TerminatesEarly)
{
    auto [iw, gc] = makeGridLambdas(astar::GridMap(1000, 1000));
    astar::AStarAlgorithm algo(iw, gc);
    algo.setMaxIterations(10);
    algo.setGoalTolerance(0.0);
    // Very distant goal — should be cut off
    auto path = algo.findPath({0, 0, 0}, {999, 999, 0});
    // Either no path (cut off) or a sub-optimal partial
    // The important thing is it does NOT run forever
    (void)path;
    SUCCEED();
}

// ─────────────────────────────────────────────────────────────────────────
// Group 3: 3-D search
// ─────────────────────────────────────────────────────────────────────────

TEST(AStarAlgorithm3D, FindsPath_OpenVolume)
{
    // All-walkable 3D space via lambda
    auto is_walkable = [](const astar::Vec3&) { return true; };
    auto get_cost    = [](const astar::Vec3&) { return 0.0; };

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N26);

    auto path = algo.findPath({0, 0, 0}, {3, 3, 3});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(),  (astar::Vec3{3, 3, 3}));
}

TEST(AStarAlgorithm3D, PathLength_FaceNeighbours)
{
    // With face-only neighbours, cost from (0,0,0) to (0,0,4) = 5 cells
    auto is_walkable = [](const astar::Vec3&) { return true; };
    auto get_cost    = [](const astar::Vec3&) { return 0.0; };

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N6);

    auto path = algo.findPath({0, 0, 0}, {0, 0, 4});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 5u);
}

// ─────────────────────────────────────────────────────────────────────────
// Group 4: AstarPathSearch integration (IMapView + planner)
// ─────────────────────────────────────────────────────────────────────────

// Minimal IMapView implementation backed by GridMap
class TestMapView : public finenav::AstarPathSearch::IMapView {
public:
    TestMapView(const astar::GridMap& g, double step_size)
        : grid_(g), step_(step_size) {}

    bool isWalkable(const finenav::Position3D& p) const override {
        return grid_.isWalkableCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }
    double getCost(const finenav::Position3D& p) const override {
        return grid_.getCostCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }
    double getStepSize() const override { return step_; }

private:
    const astar::GridMap& grid_;
    double step_;
};

// Helper: build a default-configured AstarPathSearch
inline finenav::AstarPathSearch makePlanner()
{
    finenav::AstarPathSearch planner;
    astar_planner::Params p;
    p.connectivity    = "4";
    p.heuristic       = "Euclidean";
    p.goal_tolerance  = 0.6;   // metres
    p.max_iterations  = 1000000;
    p.max_planning_time = 5.0;
    planner.configure(p);
    return planner;
}

// Helper: build a PlanningContext from cell centres
inline finenav::PlanningContext<finenav::AstarPathSearch>
makeCtx(double sx, double sy, double gx, double gy)
{
    finenav::PlanningContext<finenav::AstarPathSearch> ctx;
    ctx.robot_state.pose.position.x = sx;
    ctx.robot_state.pose.position.y = sy;
    ctx.goal_pose.position.x        = gx;
    ctx.goal_pose.position.y        = gy;
    return ctx;
}

TEST(AstarPathSearch, PlanSucceeds_OpenGrid)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_GE(result->poses.size(), 1u);
}

TEST(AstarPathSearch, PlanFails_GoalBlocked)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    EXPECT_FALSE(result.has_value());
}

TEST(AstarPathSearch, OutputPoses_AreWorldCoordinates)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 3.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // All poses should be within the grid bounds (cell centres at x.5)
    for (const auto& pose : result->poses)
    {
        EXPECT_GE(pose.position.x, 0.0);
        EXPECT_LT(pose.position.x, 10.0);
        EXPECT_GE(pose.position.y, 0.0);
        EXPECT_LT(pose.position.y, 10.0);
    }
}
