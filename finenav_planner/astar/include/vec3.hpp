// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once
#include <cstddef>
#include <cmath>

namespace astar {

    struct Vec3 {
        int x, y, z;

        Vec3(int x = 0, int y = 0, int z = 0) : x(x), y(y), z(z) {}

        bool operator==(const Vec3& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        Vec3 operator+(const Vec3& other) const {
            return Vec3{x + other.x, y + other.y, z + other.z};
        }

        double distance(const Vec3& other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            double dz = z - other.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }

        int manhattan(const Vec3& other) const {
            return std::abs(x - other.x) + std::abs(y - other.y) + std::abs(z - other.z);
        }
    };

    struct Vec3Hash {
        std::size_t operator()(const Vec3& v) const noexcept {
            // FNV-like combination
            std::size_t h = static_cast<std::size_t>(v.x);
            h ^= (static_cast<std::size_t>(v.y) << 20) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= (static_cast<std::size_t>(v.z) << 40) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };

} // namespace astar

