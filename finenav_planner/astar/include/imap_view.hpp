// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include "finenav_core/concepts.hpp"   // Position3D
namespace astar {
// =============================================================
// IMapView  —  user-facing world-coordinate map interface
//
// Implement this to connect your map to AstarPathSearch.
// You do not need to know anything about Vec3, grid cells, or
// A* internals.
//
// ── Two things you provide ────────────────────────────────────
//
//   1. World-coordinate queries
//      isWalkable / getCost
//      All inputs are in world space (metres, WORLD_FIXED_FRAME).
//
//   2. Search step size  (getStepSize)
//      "When A* moves one cell, how many metres does it travel?"
//      This is the only discretisation knowledge the planner
//      needs — and your map already has this answer.
//
//      OccupancyGrid  →  return info.resolution
//      Voxel map      →  return voxel_size
//      Custom SDF     →  return desired_sampling_interval
//
// ── Usage example ─────────────────────────────────────────────
//
//   class OccGridView : public astar::IMapView {
//       bool isWalkable(const Position3D& p) const override {
//           // look up p in your occupancy grid
//       }
//       double getCost(const Position3D& p) const override {
//           return 0.0;   // uniform cost
//       }
//       double getStepSize() const override {
//           return occ_grid_.info.resolution;  // e.g. 0.05 m
//       }
//   };
//
// Note: In FineNav-Engine every map uses WORLD_FIXED_FRAME, so
// the grid origin is always (0, 0, 0) and you never need to
// specify it.
// =============================================================
class IMapView {
public:
    virtual ~IMapView() = default;
    // ── world-coordinate queries ──────────────────────────────
    /// Can the robot pass through world point p?
    virtual bool isWalkable(const finenav::Position3D& p) const = 0;
    /// Extra traversal cost at p (must be >= 0).
    /// A* edge weight = step_cost * (1 + getCost(p)).
    /// Return 0.0 for a uniform-cost map.
    virtual double getCost(const finenav::Position3D& p) const = 0;
    // ── discretisation hint ───────────────────────────────────
    /// Distance in metres that A* travels with each grid step.
    /// The planner uses this to discretise the world internally —
    /// you never touch Vec3 or cell indices.
    ///
    /// Return whatever "one cell" means for your map:
    ///   OccupancyGrid  →  info.resolution
    ///   Voxel map      →  voxel edge length
    ///   Custom         →  desired sampling interval
    virtual double getStepSize() const = 0;
};
} // namespace astar
