// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "height_analysis.hpp"

HeightAnalyzer::HeightAnalyzer(
    const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper,
    std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper)
    : node_(node)
    , passability_helper_(std::move(passability_helper))
    , costmap_helper_(std::move(costmap_helper))
{
    const auto map_ro = map_server->getLockedReadView();
    passability_image_ = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
    costmap_image_     = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
    cached_resolution_ = map_ro->getResolution();

    node_->declare_parameter<double>("height_analysis.cost_scaling_factor", 10.0);
    node_->declare_parameter<double>("height_analysis.min_obstacle_height", 0.1);
    node_->declare_parameter<double>("height_analysis.max_obstacle_height", 2.0);

    node_->get_parameter("height_analysis.cost_scaling_factor", cost_scaling_factor_);
    node_->get_parameter("height_analysis.min_obstacle_height", min_obstacle_height_);
    node_->get_parameter("height_analysis.max_obstacle_height", max_obstacle_height_);
}

void HeightAnalyzer::update(float robot_pose_z, const finenav::GridMap<Voxel>& map) {
    cached_resolution_ = map.getResolution();
    // 实时读取参数（支持动态调参）
    node_->get_parameter("height_analysis.cost_scaling_factor", cost_scaling_factor_);
    node_->get_parameter("height_analysis.min_obstacle_height", min_obstacle_height_);
    node_->get_parameter("height_analysis.max_obstacle_height", max_obstacle_height_);

    HeightAnalysis(robot_pose_z, map);
    MapInflation();
    publishMaps(map);
}

int HeightAnalyzer::getCost(const finenav::Position3D& pos,
                             const finenav::GridMap<Voxel>& map) const {
    if (map.isInside(pos)) {
        finenav::Index index = map.getIndex(pos);
        return costmap_image_.at<uchar>(
            index.y() + map.getSize().y() / 2,
            index.x() + map.getSize().x() / 2);
    }
    return 254;
}

int HeightAnalyzer::ComputeCost(int distance) const {
    const double factor = std::exp(
        -1.0 * cost_scaling_factor_ * (distance * cached_resolution_));
    return static_cast<int>(253 * factor);
}

// ── 私有：高度区间障碍分析 ────────────────────────────────────────────────

void HeightAnalyzer::HeightAnalysis(float robot_pose_z,
                                     const finenav::GridMap<Voxel>& map) {
    const int sizeX_min    = map.getMinIndex().x();
    const int sizeX_max    = map.getMaxIndex().x();
    const int sizeY_min    = map.getMinIndex().y();
    const int sizeY_max    = map.getMaxIndex().y();
    const size_t size_x    = map.getSize().x();
    const size_t size_y    = map.getSize().y();
    const size_t half_size_x = size_x / 2;
    const size_t half_size_y = size_y / 2;

    const float z_min = robot_pose_z + static_cast<float>(min_obstacle_height_);
    const float z_max = robot_pose_z + static_cast<float>(max_obstacle_height_);

    for (int x = sizeX_min; x <= sizeX_max; ++x) {
        for (int y = sizeY_min; y <= sizeY_max; ++y) {
            const std::vector<Voxel> voxels = map.getVoxelsAlongZ(x, y);

            bool is_obstacle = false;
            for (const auto& voxel : voxels) {
                if (!std::isfinite(voxel.z_height)) continue;
                if (voxel.z_height >= z_min && voxel.z_height <= z_max) {
                    is_obstacle = true;
                    break;
                }
            }

            const size_t px = static_cast<size_t>(x) + half_size_x;
            const size_t py = static_cast<size_t>(y) + half_size_y;
            passability_image_.at<uchar>(
                static_cast<int>(py), static_cast<int>(px)) =
                is_obstacle ? 254 : 0;
        }
    }
}

// ── 私有：膨胀 + 发布（与 TerrainAnalyzer 完全一致）─────────────────────

void HeightAnalyzer::MapInflation() {
    cv::Mat dist = DistanceTransform();
    for (int x = 0; x < passability_image_.cols; ++x) {
        for (int y = 0; y < passability_image_.rows; ++y) {
            if (passability_image_.at<uchar>(y, x) == 0) {
                costmap_image_.at<uchar>(y, x) =
                    static_cast<uchar>(ComputeCost(
                        static_cast<int>(dist.at<float>(y, x))));
            } else {
                costmap_image_.at<uchar>(y, x) = 254;
            }
        }
    }
}

cv::Mat HeightAnalyzer::DistanceTransform() {
    cv::Mat binary = 254 - passability_image_;
    cv::Mat dist;
    cv::distanceTransform(binary, dist, cv::DIST_L2, 3);
    return dist;
}

void HeightAnalyzer::publishMaps(const finenav::GridMap<Voxel>& map) {
    if (!node_) return;

    const size_t size_x      = map.getSize().x();
    const size_t size_y      = map.getSize().y();
    const size_t half_size_x = size_x / 2;
    const size_t half_size_y = size_y / 2;
    const auto   stamp       = node_->now();

    if (passability_helper_) {
        passability_helper_->reserve(size_x * size_y);
        for (size_t px = 0; px < size_x; ++px) {
            for (size_t py = 0; py < size_y; ++py) {
                finenav::Index idx(
                    static_cast<int>(px) - static_cast<int>(half_size_x),
                    static_cast<int>(py) - static_cast<int>(half_size_y), 0);
                if (!map.isInside(idx)) continue;
                const auto pos = map.getPosition(idx);
                const uint8_t val = passability_image_.at<uint8_t>(
                    static_cast<int>(py), static_cast<int>(px));
                const std::array<uint8_t, 3> color =
                    (val == 0) ? std::array<uint8_t,3>{0, 200, 0}
                               : std::array<uint8_t,3>{200, 0, 0};
                passability_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    0.0f, color);
            }
        }
        passability_helper_->publish(stamp);
    }

    if (costmap_helper_) {
        costmap_helper_->reserve(size_x * size_y);
        for (size_t px = 0; px < size_x; ++px) {
            for (size_t py = 0; py < size_y; ++py) {
                finenav::Index idx(
                    static_cast<int>(px) - static_cast<int>(half_size_x),
                    static_cast<int>(py) - static_cast<int>(half_size_y), 0);
                if (!map.isInside(idx)) continue;
                const auto pos = map.getPosition(idx);
                const uint8_t val = costmap_image_.at<uint8_t>(
                    static_cast<int>(py), static_cast<int>(px));
                const std::array<uint8_t, 3> color =
                    (val == 254)
                        ? std::array<uint8_t,3>{0, 0, 0}
                        : std::array<uint8_t,3>{val, static_cast<uint8_t>(255 - val), 0};
                costmap_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    0.0f, color);
            }
        }
        costmap_helper_->publish(stamp);
    }
}
