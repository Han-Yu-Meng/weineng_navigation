// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <limits>
#include <cmath>

namespace astar {

// =============================================================
// GridMap
//
// A simple 2-D grid data container for standalone testing.
// Cells are indexed by integer (x, y) coordinates.
//
// This class has NO FineNav / ROS dependencies and does NOT
// implement AstarPathSearch::IMapView directly.  In main.cpp
// (or any test), wrap it in a concrete IMapView subclass that
// calls the cell-query methods below after converting its
// world-coordinate input with the planner's own resolution.
// =============================================================

class GridMap {
public:
    GridMap(int width, int height, double default_cost = 0.0)
        : width_(width), height_(height),
          costs_(width * height, default_cost)
    {}

    // ── setters ───────────────────────────────────────────────

    void setObstacle(int x, int y, bool obstacle) {
        if (isValid(x, y))
            costs_[idx(x, y)] = obstacle ? INF_COST : 0.0;
    }

    void setCost(int x, int y, double cost) {
        if (isValid(x, y))
            costs_[idx(x, y)] = cost;
    }

    // ── cell-space queries (used by a wrapping IMapView) ──────

    bool isWalkableCell(int x, int y) const {
        return isValid(x, y) && costs_[idx(x, y)] < INF_COST;
    }

    double getCostCell(int x, int y) const {
        return isValid(x, y) ? costs_[idx(x, y)] : INF_COST;
    }

    // ── metadata ──────────────────────────────────────────────

    int getWidth()  const { return width_;  }
    int getHeight() const { return height_; }

private:
    bool isValid(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }
    int idx(int x, int y) const { return y * width_ + x; }

    int    width_, height_;
    std::vector<double> costs_;
    static constexpr double INF_COST = 1e9;
};

} // namespace astar

