// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_util/algo_configurator.hpp"
#include "finenav_interface/test_map_params.hpp"

namespace finenav {



class SimpleTestMap {
public:
    using DataType = int;
    using ConfigType = test_map::Params;

    SimpleTestMap() = default;

    void configure(const ConfigType &params) {
        // 假设有 resolution 和 name 参数
        if (params.background.b == 255) { /* 复杂的分支控制逻辑 */}

    }

    void debug_print() const {
        RCLCPP_INFO(rclcpp::get_logger("SimpleTestMap"), "SimpleTestMap debug_print called");
    }

    bool isInside(const Position3D& pos) const {
        return true;
    }

    Region3D getWindowBounds() const {
        return Eigen::AlignedBox3d();
    }

    Position3D getWindowCenter() const {
        return Eigen::Vector3d();
    }

    void shiftWindowTo(const Position3D& pos) {

    }

};
}

FINENAV_REGISTER_ALGO_CONFIG(finenav::SimpleTestMap, test_map)