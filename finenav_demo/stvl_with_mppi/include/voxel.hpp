// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once  // ← 添加这行

#include <cmath>
#include <rclcpp/rclcpp.hpp>
struct Voxel {
    float z_height = NAN;
    bool is_static = false;
    rclcpp::Time timestamp{};
};


