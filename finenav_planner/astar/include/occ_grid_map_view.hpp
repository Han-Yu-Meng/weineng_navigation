// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "imap_view.hpp"
#include "occ_grid_2d_map.hpp"

namespace astar {

/**
 * @brief OccGridMapView 的处理参数
 *
 * 描述如何将原始占用值解释为规划代价，与地图数据本身无关。
 * 由调用者（builder lambda）负责提供，可来自 ROS 参数或直接赋值。
 */
struct OccGridMapViewConfig {
    double lethal_radius       = 0.10; ///< 硬膨胀半径 (m)，范围内代价=253
    double inflation_radius    = 0.50; ///< 软膨胀半径 (m)
    double cost_scaling_factor = 10.0; ///< 软膨胀指数衰减系数
};

/**
 * @brief OccGridMapView — 基于 OccGrid2DMap 的 IMapView 实现
 *
 * 持有 OccGrid2DMap 的 const 引用（由 ControlLayer 读锁保护）。
 * 在构造时读取原始地图数据并完成二值化 + 代价图构建，之后只读。
 * 地图订阅与数据注入由 MapServer<OccGrid2DMap> 负责，此类不涉及 ROS 通信。
 */
class OccGridMapView : public IMapView {
public:
    explicit OccGridMapView(const finenav::OccGrid2DMap& map,
                            const OccGridMapViewConfig&  view_config = {});

    // --- IMapView 接口 ---
    bool   isWalkable(const finenav::Position3D& p) const override;
    double getCost(const finenav::Position3D& p)    const override;
    double getStepSize()                            const override;

private:
    void binarizeMap(const nav_msgs::msg::OccupancyGrid& msg, int threshold);
    void buildCostMap(double lethal_radius, double inflation_radius, double cost_scaling_factor);
    bool worldToGrid(double x, double y, int& col, int& row) const;

    double   resolution_ = 0.05;
    uint32_t width_      = 0;
    uint32_t height_     = 0;
    double   origin_x_   = 0.0;
    double   origin_y_   = 0.0;

    cv::Mat binary_map_;
    cv::Mat cost_map_;
};

} // namespace astar

