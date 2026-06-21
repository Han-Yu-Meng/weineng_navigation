// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "occ_grid_map_view.hpp"
#include <cmath>

namespace astar {

OccGridMapView::OccGridMapView(const finenav::OccGrid2DMap& map,
                               const OccGridMapViewConfig&  view_config) {
    if (!map.hasMap()) {
        fprintf(stderr, "[OccGridMapView] WARNING: map has no data yet, all cells will be non-walkable!\n");
        return;
    }

    const auto& grid = map.grid();
    const auto& cfg  = map.config();

    resolution_ = grid.info.resolution;
    width_      = grid.info.width;
    height_     = grid.info.height;
    origin_x_   = grid.info.origin.position.x;
    origin_y_   = grid.info.origin.position.y;

    // occupancy_threshold 属于地图语义，从地图 config 读取
    binarizeMap(grid, cfg.occupancy_threshold);
    // 膨胀参数属于规划策略，从 view_config 读取
    buildCostMap(view_config.lethal_radius,
                 view_config.inflation_radius,
                 view_config.cost_scaling_factor);
}

// ── IMapView 接口 ──────────────────────────────────────────────────────────

bool OccGridMapView::isWalkable(const finenav::Position3D& p) const {
    int col, row;
    if (!worldToGrid(p.x(), p.y(), col, row)) return false;
    uint8_t cost = cost_map_.at<uint8_t>(row, col);
    return cost < 253;
}
double OccGridMapView::getCost(const finenav::Position3D& p) const {
    int col, row;
    if (!worldToGrid(p.x(), p.y(), col, row)) return 0.0;
    return static_cast<double>(cost_map_.at<uint8_t>(row, col)) / 255.0;
}

double OccGridMapView::getStepSize() const {
    return resolution_;
}

// ── 私有处理 ───────────────────────────────────────────────────────────────

void OccGridMapView::binarizeMap(const nav_msgs::msg::OccupancyGrid& msg, int threshold) {
    binary_map_ = cv::Mat(static_cast<int>(height_), static_cast<int>(width_), CV_8UC1);
    for (uint32_t row = 0; row < height_; ++row) {
        for (uint32_t col = 0; col < width_; ++col) {
            const int8_t val = msg.data[row * width_ + col];
            binary_map_.at<uint8_t>(static_cast<int>(row), static_cast<int>(col)) =
                (val < 0 || val >= threshold) ? 254 : 0;
        }
    }
}

void OccGridMapView::buildCostMap(double lethal_radius, double inflation_radius, double cost_scaling_factor) {
    // ↓ 添加
    if (binary_map_.empty()) {
        fprintf(stderr, "[OccGridMapView] buildCostMap: binary_map_ is empty, skipping!\n");
        return;
    }
    cv::Mat inverted;
    cv::subtract(cv::Scalar(254), binary_map_, inverted);

    cv::Mat dist_px;
    cv::distanceTransform(inverted, dist_px, cv::DIST_L2, 5);

    cost_map_ = cv::Mat(dist_px.rows, dist_px.cols, CV_8UC1);
    const float res    = static_cast<float>(resolution_);
    const float lethal = static_cast<float>(lethal_radius);
    const float infl   = static_cast<float>(inflation_radius);

    for (int row = 0; row < dist_px.rows; ++row) {
        for (int col = 0; col < dist_px.cols; ++col) {
            const float dist_m = dist_px.at<float>(row, col) * res;
            uint8_t cost;
            if (dist_m == 0.0f) {
                cost = 254;
            } else if (dist_m <= lethal) {
                cost = 253;
            } else if (dist_m <= infl) {
                cost = static_cast<uint8_t>(
                    253.0 * std::exp(-cost_scaling_factor * static_cast<double>(dist_m - lethal)));
            } else {
                cost = 0;
            }
            cost_map_.at<uint8_t>(row, col) = cost;
        }
    }
}

bool OccGridMapView::worldToGrid(double x, double y, int& col, int& row) const {
    if (cost_map_.empty()) {
        fprintf(stderr, "[OccGridMapView] worldToGrid: cost_map_ is EMPTY! (width=%u height=%u)\n",
                width_, height_);
        return false;
    }
    col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return col >= 0 && col < static_cast<int>(width_) &&
           row >= 0 && row < static_cast<int>(height_);
}

} // namespace astar

