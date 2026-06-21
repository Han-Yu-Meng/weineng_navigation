// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// =============================================================
// Eval: AstarPathSearch scenario evaluation
//
// Simulates a robot navigating a 2-D obstacle field and prints
// a visual map + metrics (path length, number of waypoints).
// This is not a unit test — it is a qualitative evaluation tool
// for inspecting planner behaviour in a controlled scenario.
//
// Run:  ros2 run finenav_planner_astar astar_eval
// =============================================================

#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

#include "astar_path_search.hpp"

// ── Concrete IMapView ──────────────────────────────────────────────────────
//
// The user never needs to include grid_map.hpp or know about Vec3.
// They only provide world-coordinate answers.

#include "grid_map.hpp"   // test utility (cell-indexed storage)

class ScenarioMapView : public finenav::AstarPathSearch::IMapView {
public:
    // Origin is always (0,0,0) in FineNav (WORLD_FIXED_FRAME).
    ScenarioMapView(const astar::GridMap& g, double res)
        : grid_(g), res_(res) {}

    bool isWalkable(const finenav::Position3D& p) const override {
        int cx = static_cast<int>(std::floor(p.x() / res_));
        int cy = static_cast<int>(std::floor(p.y() / res_));
        return grid_.isWalkableCell(cx, cy);
    }

    double getCost(const finenav::Position3D& p) const override {
        int cx = static_cast<int>(std::floor(p.x() / res_));
        int cy = static_cast<int>(std::floor(p.y() / res_));
        return grid_.getCostCell(cx, cy);
    }

    double getStepSize() const override { return res_; }

private:
    const astar::GridMap& grid_;
    double res_;
};

// ── Scenario definition ────────────────────────────────────────────────────

struct Scenario {
    const char* name;
    int         width;
    int         height;
    double      resolution;
    // Start / goal in world coords (metres)
    double sx, sy, gx, gy;
    std::string connectivity;   // "4"/"8" for 2-D, "6"/"18"/"26" for 3-D
    // Obstacle setter: called with the GridMap
    void (*build_map)(astar::GridMap&);
};

static void build_L_obstacle(astar::GridMap& g)
{
    // Horizontal wall row 3, cols 2-7
    for (int x = 2; x <= 7; ++x) g.setObstacle(x, 3, true);
    // Vertical wall col 5, rows 3-7
    for (int y = 3; y <= 7; ++y) g.setObstacle(5, y, true);
}

static void build_maze(astar::GridMap& g)
{
    // Alternating horizontal walls with a gap at alternating ends
    for (int x = 0; x <= 7; ++x) g.setObstacle(x, 2, true);   // gap at x=8,9
    for (int x = 2; x <= 9; ++x) g.setObstacle(x, 5, true);   // gap at x=0,1
    for (int x = 0; x <= 7; ++x) g.setObstacle(x, 8, true);   // gap at x=8,9
}

static const Scenario kScenarios[] = {
    {
        "L-shaped obstacle (N4 — orthogonal only)",
        10, 10, 1.0,
        1.5, 1.5, 8.5, 8.5,
        "4",
        build_L_obstacle
    },
    {
        "L-shaped obstacle (N8 — with diagonal)",
        10, 10, 1.0,
        1.5, 1.5, 8.5, 8.5,
        "8",
        build_L_obstacle
    },
    {
        "Maze (N4 — orthogonal only)",
        10, 10, 1.0,
        0.5, 0.5, 9.5, 9.5,
        "4",
        build_maze
    },
};

// ── Rendering ─────────────────────────────────────────────────────────────

static void render(const astar::GridMap& grid, int W, int H, double res,
                   const std::vector<geometry_msgs::msg::Pose>& poses,
                   int start_cx, int start_cy,
                   int goal_cx,  int goal_cy)
{
    std::unordered_set<long long> path_set;
    for (const auto& p : poses)
    {
        int cx = static_cast<int>(std::floor(p.position.x / res));
        int cy = static_cast<int>(std::floor(p.position.y / res));
        path_set.insert(static_cast<long long>(cy) * W + cx);
    }

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            long long key = static_cast<long long>(y) * W + x;
            if      (x == start_cx && y == start_cy)  std::cout << "S ";
            else if (x == goal_cx  && y == goal_cy)   std::cout << "G ";
            else if (!grid.isWalkableCell(x, y))       std::cout << "# ";
            else if (path_set.count(key))              std::cout << "* ";
            else                                       std::cout << ". ";
        }
        std::cout << "\n";
    }
}

// ── main ──────────────────────────────────────────────────────────────────

#include <unordered_set>

int main()
{
    for (const auto& sc : kScenarios)
    {
        std::cout << "\n═══════════════════════════════════════════\n";
        std::cout << "Scenario: " << sc.name << "\n";
        std::cout << "═══════════════════════════════════════════\n";

        // 1. Build map data
        astar::GridMap grid(sc.width, sc.height);
        sc.build_map(grid);

        // 2. Wrap in IMapView  (step size comes from the view, not from params)
        ScenarioMapView view(grid, sc.resolution);

        // 3. Configure planner
        finenav::AstarPathSearch planner;
        astar_planner::Params params;
        params.connectivity     = sc.connectivity;
        params.heuristic        = "Euclidean";
        params.goal_tolerance   = sc.resolution * 0.6;
        params.max_iterations   = 1000000;
        params.max_planning_time= 5.0;
        planner.configure(params);

        // 4. Plan and time it
        finenav::PlanningContext<finenav::AstarPathSearch> ctx;
        ctx.robot_state.pose.position.x = sc.sx;
        ctx.robot_state.pose.position.y = sc.sy;
        ctx.goal_pose.position.x        = sc.gx;
        ctx.goal_pose.position.y        = sc.gy;

        const auto t0 = std::chrono::steady_clock::now();
        auto result = planner.plan(ctx, view);
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count() * 1000.0;

        // 5. Report
        if (!result.has_value()) {
            std::cout << "  [FAIL] No path found.\n";
            continue;
        }

        const auto& poses = result->poses;

        // Compute path length in world coords
        double length = 0.0;
        for (std::size_t i = 1; i < poses.size(); ++i)
        {
            double dx = poses[i].position.x - poses[i-1].position.x;
            double dy = poses[i].position.y - poses[i-1].position.y;
            length += std::sqrt(dx*dx + dy*dy);
        }

        std::cout << "  Waypoints : " << poses.size()   << "\n";
        std::cout << "  Length    : " << std::fixed << std::setprecision(2)
                  << length << " m\n";
        std::cout << "  Time      : " << elapsed << " ms\n\n";

        int scx = static_cast<int>(std::floor(sc.sx / sc.resolution));
        int scy = static_cast<int>(std::floor(sc.sy / sc.resolution));
        int gcx = static_cast<int>(std::floor(sc.gx / sc.resolution));
        int gcy = static_cast<int>(std::floor(sc.gy / sc.resolution));

        render(grid, sc.width, sc.height, sc.resolution, poses, scx, scy, gcx, gcy);
    }

    return 0;
}
