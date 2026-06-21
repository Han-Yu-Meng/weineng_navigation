// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "astar_algorithm.hpp"

#include <algorithm>
#include <array>
#include <chrono>

namespace astar {

// ── constructor ────────────────────────────────────────────────────────────

AStarAlgorithm::AStarAlgorithm(IsWalkableFn is_walkable, GetCostFn get_cost)
    : is_walkable_(std::move(is_walkable)),
      get_cost_(std::move(get_cost))
{}

// ── findPath ───────────────────────────────────────────────────────────────

std::optional<std::vector<Vec3>> AStarAlgorithm::findPath(
    const Vec3& start,
    const Vec3& goal)
{
    if (!is_walkable_(start) || !is_walkable_(goal))
        return std::nullopt;

    open_set_ = {};
    visited_.clear();
    came_from_.clear();

    int  iterations = 0;
    auto start_time = std::chrono::steady_clock::now();

    std::optional<AstarNode> best_goal_node;
    double best_goal_dist = std::numeric_limits<double>::infinity();

    AstarNode start_node(start);
    start_node.g      = 0.0;
    start_node.h      = calculateHeuristic(heuristic_, start, goal);
    start_node.updateF();
    start_node.parent = start;   // self-sentinel

    open_set_.push(start_node);
    came_from_[start] = start_node;

    while (!open_set_.empty())
    {
        if (max_iterations_ >= 0 && iterations++ > max_iterations_)
            break;

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > max_planning_time_)
            break;

        AstarNode current = open_set_.top();
        open_set_.pop();

        if (visited_.contains(current.pos)) continue;
        visited_.insert(current.pos);

        if (current.pos == goal)
            return reconstructPath(goal);

        double goal_dist = cellDistance(current.pos, goal);
        if (goal_dist < best_goal_dist)
        {
            best_goal_dist = goal_dist;
            best_goal_node = current;
        }

        for (const Vec3& nb_pos : getNeighbors(current.pos))
        {
            if (!is_walkable_(nb_pos))      continue;
            if (visited_.contains(nb_pos))  continue;

            double step_cost   = calculateStepCost(current.pos, nb_pos);
            double map_cost    = get_cost_(nb_pos);

            // movement_weight是运动移动的权重，用于配平用户传入的map_cost，map_cost可以为任意值，不局限于传统ros2的costmap_2d的0-255范围。
            // 比如如果用户的地图代价是0.0-1.0，运动权重可以设置为10.0，这样A*就会更重视运动距离，避免过度受地图代价影响。
            double tentative_g = current.g + step_cost * (1.0 * movement_weight_ + map_cost);


            if (!came_from_.contains(nb_pos) ||
                tentative_g < came_from_[nb_pos].g)
            {
                AstarNode nb(nb_pos);
                nb.g      = tentative_g;

                // 设置贪心权重greedy_weight，让用户决定该算法是否要更加贪心（更快但可能更次优）。
                nb.h      = calculateHeuristic(heuristic_, nb_pos, goal) * greedy_weight_;
                nb.updateF();
                nb.parent = current.pos;

                came_from_[nb_pos] = nb;
                open_set_.push(nb);
            }
        }
    }

    if (best_goal_node.has_value() && best_goal_dist <= goal_tolerance_)
        return reconstructPath(best_goal_node->pos);

    return std::nullopt;
}

// ── getNeighbors ───────────────────────────────────────────────────────────

std::vector<Vec3> AStarAlgorithm::getNeighbors(const Vec3& pos) const
{
    static const std::array<Vec3, 4> ortho2d = {{
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0}
    }};
    static const std::array<Vec3, 4> diag2d = {{
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0}
    }};
    static const std::array<Vec3, 6> face3d = {{
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
    }};
    static const std::array<Vec3, 12> edge3d = {{
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
        {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
        {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1}
    }};
    static const std::array<Vec3, 8> corner3d = {{
        {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
        {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}
    }};

    std::vector<Vec3> neighbors;

    switch (connectivity_)
    {
        case Connectivity::N4:
            for (const auto& o : ortho2d)  neighbors.push_back(pos + o);
            break;

        case Connectivity::N8:
            for (const auto& o : ortho2d)  neighbors.push_back(pos + o);
            for (const auto& o : diag2d)   neighbors.push_back(pos + o);
            break;

        case Connectivity::N6:
            for (const auto& o : face3d)   neighbors.push_back(pos + o);
            break;

        case Connectivity::N18:
            for (const auto& o : face3d)   neighbors.push_back(pos + o);
            for (const auto& o : edge3d)   neighbors.push_back(pos + o);
            break;

        case Connectivity::N26:
            for (const auto& o : face3d)   neighbors.push_back(pos + o);
            for (const auto& o : edge3d)   neighbors.push_back(pos + o);
            for (const auto& o : corner3d) neighbors.push_back(pos + o);
            break;
    }

    return neighbors;
}

// ── calculateStepCost ─────────────────────────────────────────────────────
// Pure geometry: 1 axis → 1.0,  2 axes → √2,  3 axes → √3

double AStarAlgorithm::calculateStepCost(const Vec3& from, const Vec3& to)
{
    int nd = (from.x != to.x) + (from.y != to.y) + (from.z != to.z);
    if (nd <= 1) return 1.0;
    if (nd == 2) return std::sqrt(2.0);
    return std::sqrt(3.0);
}

// ── cellDistance ──────────────────────────────────────────────────────────

double AStarAlgorithm::cellDistance(const Vec3& a, const Vec3& b)
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// ── reconstructPath ───────────────────────────────────────────────────────

std::vector<Vec3> AStarAlgorithm::reconstructPath(const Vec3& goal) const
{
    std::vector<Vec3> path;
    Vec3 cur = goal;
    while (true)
    {
        path.push_back(cur);
        const auto& node = came_from_.at(cur);
        if (cur == node.parent) break;
        cur = node.parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace astar

