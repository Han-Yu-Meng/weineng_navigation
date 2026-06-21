// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <algorithm>
#include "vec3.hpp"

namespace astar {

enum class Heuristic {
    Euclidean,
    Manhattan,
    Chebyshev
};

// Heuristic operates on Vec3 cells.
// For 2-D grids z is always 0, so the 3-D formula degrades correctly.
inline double calculateHeuristic(
    Heuristic   type,
    const Vec3& from,
    const Vec3& to)
{
    double dx = std::abs(from.x - to.x);
    double dy = std::abs(from.y - to.y);
    double dz = std::abs(from.z - to.z);

    switch (type)
    {
    case Heuristic::Euclidean:
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    case Heuristic::Manhattan:
        return dx + dy + dz;
    case Heuristic::Chebyshev:
        return std::max({dx, dy, dz});
    default:
        return dx + dy + dz;
    }
}

} // namespace astar

