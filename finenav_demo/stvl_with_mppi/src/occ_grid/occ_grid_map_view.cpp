// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "occ_grid/occ_grid_map_view.hpp"
#include <cmath>
#include <functional>
#include <cstdint>
#include <string>

namespace astar {

OccGridMapView::OccGridMapView(
    rclcpp::Node* node,
    const std::string& topic_name)
    : node_ptr_(node) {

    // 注册话题名称参数
    if (!node_ptr_->has_parameter("map_topic")) {
        node_ptr_->declare_parameter("map_topic", topic_name);
    }
    std::string actual_topic = node_ptr_->get_parameter("map_topic").as_string();

    // 注册二值化阈值参数
    if (!node_ptr_->has_parameter("occupancy_threshold")) {
        node_ptr_->declare_parameter("occupancy_threshold", 50);
    }
    // 注册膨胀半径和代价缩放因子 (Nav2 标准)
    if (!node_ptr_->has_parameter("inflation_radius")) {
        node_ptr_->declare_parameter("inflation_radius", 0.55);
    }
    if (!node_ptr_->has_parameter("cost_scaling_factor")) {
        node_ptr_->declare_parameter("cost_scaling_factor", 10.0);
    }
    // 注册硬膨胀半径 (Lethal Obstacle Inflation)
    if (!node_ptr_->has_parameter("lethal_inflation_radius")) {
        node_ptr_->declare_parameter("lethal_inflation_radius", 0.1);
    }

    // 使用 TransientLocal 以便能收到之前发布的静态地图
    auto qos = rclcpp::QoS(1).transient_local();

    sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        actual_topic, qos, std::bind(&OccGridMapView::mapCallback, this, std::placeholders::_1));
}

OccGridMapView::OccGridMapView(
    const finenav::OccGrid2DMap& map,
    int threshold,
    double lethal_radius,
    double inflation_radius,
    double cost_scaling_factor)
    : node_ptr_(nullptr) {
    if (!map.hasMap()) return;
    const auto& grid = map.grid();
    resolution_ = grid.info.resolution;
    width_      = grid.info.width;
    height_     = grid.info.height;
    origin_x_   = grid.info.origin.position.x;
    origin_y_   = grid.info.origin.position.y;
    binarizeMap(grid, threshold);
    updateCostMap(lethal_radius, inflation_radius, cost_scaling_factor);
    has_received_map_ = true;
}

void OccGridMapView::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int threshold = node_ptr_->get_parameter("occupancy_threshold").as_int();
    double inflation_radius = node_ptr_->get_parameter("inflation_radius").as_double();
    double cost_scaling_factor = node_ptr_->get_parameter("cost_scaling_factor").as_double();
    double lethal_inflation_radius = node_ptr_->get_parameter("lethal_inflation_radius").as_double();

    // 更新元数据
    resolution_ = msg->info.resolution;
    width_ = msg->info.width;
    height_ = msg->info.height;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;

    // 1. 二值化
    binarizeMap(*msg, threshold);

    // 2. 计算膨胀权重图 (包含了硬膨胀和距离变换逻辑)
    updateCostMap(lethal_inflation_radius, inflation_radius, cost_scaling_factor);

    has_received_map_ = true;
}

void OccGridMapView::binarizeMap(const nav_msgs::msg::OccupancyGrid& msg, int threshold) {
    binary_map_ = cv::Mat(height_, width_, CV_8UC1);
    for (uint32_t i = 0; i < height_; ++i) {
        for (uint32_t j = 0; j < width_; ++j) {
            int8_t val = msg.data[i * width_ + j];
            if (val < 0 || val >= threshold) {
                binary_map_.at<uint8_t>(i, j) = 254; // 占据 (Lethal)
            } else {
                binary_map_.at<uint8_t>(i, j) = 0;   // 空闲
            }
        }
    }
}

void OccGridMapView::updateCostMap(double lethal_radius, double inflation_radius, double cost_scaling_factor) {
    // 1. 准备距离变换
    // 直接基于原始 binary_map_ (0:空闲, 254:占据) 进行处理
    // Distance Transform 计算的是到 0 像素的距离，因此需逻辑反转：使障碍物像素为 0
    cv::Mat inverted_map;
    cv::subtract(cv::Scalar(254), binary_map_, inverted_map);

    cv::Mat dist_map;
    cv::distanceTransform(inverted_map, dist_map, cv::DIST_L2, 5);

    cost_map_ = cv::Mat(height_, width_, CV_8UC1);

    for (int i = 0; i < dist_map.rows; ++i) {
        for (int j = 0; j < dist_map.cols; ++j) {
            float dist = dist_map.at<float>(i, j) * static_cast<float>(resolution_);
            uint8_t cost = 0;

            // 2. 根据物理距离直接分层计算代价 (混合硬膨胀与软膨胀)
            if (dist == 0) {
                // 距离在硬膨胀半径内（包含障碍物本身），统一设为致命障碍
                cost = 254;
            } else if(dist <= lethal_radius) {
                // 硬膨胀区域：直接设为致命障碍
                cost = 253;

            }else if (dist <= inflation_radius) {
                // 软膨胀区域：为了保证跨越 lethal 边界时的平滑性，减去硬膨胀半径作为有效衰减距离
                cost = static_cast<uint8_t>(253 * std::exp(-1.0 * cost_scaling_factor * (dist - lethal_radius)));
            } else {
                // 超出膨胀范围，无额外代价
                cost = 0;
            }
            cost_map_.at<uint8_t>(i, j) = cost;
        }
    }
}

bool OccGridMapView::hasMap() const {
    return has_received_map_;
}

double OccGridMapView::getStepSize() const {
    return resolution_;
}

bool OccGridMapView::isWalkable(const finenav::Position3D& p) const {
    if (!has_received_map_) return false;

    int cx = static_cast<int>(std::floor((p.x() - origin_x_) / resolution_));
    int cy = static_cast<int>(std::floor((p.y() - origin_y_) / resolution_));

    if (cx < 0 || cx >= (int)width_ || cy < 0 || cy >= (int)height_) {
        return false;
    }

    // Nav2 标准下，代价 >= 253 (INSCRIBED) 即认为不可通行
    return cost_map_.at<uint8_t>(cy, cx) < 253;
}

double OccGridMapView::getCost(const finenav::Position3D& p) const {
    if (!has_received_map_) return 0.0;

    int cx = static_cast<int>(std::floor((p.x() - origin_x_) / resolution_));
    int cy = static_cast<int>(std::floor((p.y() - origin_y_) / resolution_));

    if (cx < 0 || cx >= (int)width_ || cy < 0 || cy >= (int)height_) {
        return 0.0;
    }

    // 将 0-255 映射到 A* 所需的额外权重
    // A* edge weight = step_cost * (1 + getCost(p))
    // Nav2 中代价 128 左右可能对应 1.0 的额外权重
    return static_cast<double>(cost_map_.at<uint8_t>(cy, cx)) / 255.0; // 归一化到 [0.0, 1.0]
}

} // namespace astar

