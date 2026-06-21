// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "imap_view.hpp"

namespace astar {

// =============================================================
// EmptyMapView
//
// An IMapView implementation that treats the entire world as
// completely empty (all points walkable, zero traversal cost).
//
// This is useful when you want to run A* without a real global
// map — for example, testing the planner in an obstacle-free
// environment, or when you only have local sensing and want
// A* to assume open space globally.
//
// The resolution (step size) is user-configurable; default is
// 0.05 m, matching common occupancy grid resolutions.
// =============================================================

class EmptyMapView : public IMapView {
public:
    explicit EmptyMapView(double resolution = 0.05)
        : resolution_(resolution) {}

    // ── IMapView interface ──────────────────────────────────────

    /// All points are walkable.
    bool isWalkable(const finenav::Position3D& /*p*/) const override {
        return true;
    }

    /// Zero traversal cost everywhere.
    double getCost(const finenav::Position3D& /*p*/) const override {
        return 0.0;
    }

    /// Returns the user-configured step size (default 0.05 m).
    double getStepSize() const override {
        return resolution_;
    }

private:
    double resolution_;
};

} // namespace astar
