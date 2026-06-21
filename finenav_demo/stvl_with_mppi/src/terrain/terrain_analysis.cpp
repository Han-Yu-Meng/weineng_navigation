// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-27.
//
#include "terrain/terrain_analysis.hpp"
#include <chrono>

TerrainAnalyzer::TerrainAnalyzer(
    const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper,
    std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper,
    std::shared_ptr<finenav_utils::CloudPublishHelper> ground_helper)
    : node_(node)
    , passability_helper_(std::move(passability_helper))
    , costmap_helper_(std::move(costmap_helper))
    , ground_helper_(std::move(ground_helper))
{
    const auto map_ro = map_server->getLockedReadView();
    passability_image_ = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
    costmap_image_     = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
    cached_resolution_ = map_ro->getResolution();

    node_->declare_parameter<double>("terrain_analysis.cost_scaling_factor", 0.5);
    node_->declare_parameter<double>("terrain_analysis.robot_height", 1.0);
    node_->declare_parameter<double>("terrain_analysis.max_gradient", 0.8);
    node_->declare_parameter<double>("terrain_analysis.nan_fill_radius", 1.0);
    node_->declare_parameter<double>("terrain_analysis.ground_z_offset", 0.125);
    node_->declare_parameter<bool>("terrain_analysis.use_simple_z_threshold", false);
    node_->declare_parameter<double>("terrain_analysis.z_obstacle_threshold", 0.3);
    node_->declare_parameter<std::string>("terrain_analysis.global_map_topic", "/map");

    node_->get_parameter("terrain_analysis.cost_scaling_factor", cost_scaling_factor_);
    node_->get_parameter("terrain_analysis.robot_height", robot_height_);
    node_->get_parameter("terrain_analysis.max_gradient", max_gradient_);
    node_->get_parameter("terrain_analysis.nan_fill_radius", nan_fill_radius_);
    node_->get_parameter("terrain_analysis.ground_z_offset", ground_z_offset_);
    node_->get_parameter("terrain_analysis.use_simple_z_threshold", use_simple_z_threshold_);
    node_->get_parameter("terrain_analysis.z_obstacle_threshold", z_obstacle_threshold_);

    std::string global_map_topic;
    node_->get_parameter("terrain_analysis.global_map_topic", global_map_topic);

    global_map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        global_map_topic, rclcpp::QoS(1).transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            if (!global_map_) {
                global_map_ = msg;
            }
        });
}

void TerrainAnalyzer::update(float robot_pose_z, const finenav::GridMap<Voxel>& map) {
    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::duration<double, std::milli>;

    cached_resolution_ = map.getResolution();

    auto t0 = Clock::now();
    TerrainAnalysis(robot_pose_z, map);
    auto t1 = Clock::now();
    MapInflation();
    auto t2 = Clock::now();
    publishMaps(map);
    auto t3 = Clock::now();
    //
    const double ms_terrain   = Ms(t1 - t0).count();
    const double ms_inflation = Ms(t2 - t1).count();
    const double ms_publish   = Ms(t3 - t2).count();
    const double ms_total     = Ms(t3 - t0).count();
    
    auto& clk = *node_->get_clock();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), clk, 2000,
        "\n[TerrainAnalyzer] timing (map %zux%zu @ %.2fm/cell)\n"
        "  TerrainAnalysis : %7.2f ms\n"
        "  MapInflation    : %7.2f ms\n"
        "  publishMaps     : %7.2f ms\n"
        "  ─────────────────────────\n"
        "  Total           : %7.2f ms",
        map.getSize().x(), map.getSize().y(), cached_resolution_,
        ms_terrain, ms_inflation, ms_publish, ms_total);
}

int TerrainAnalyzer::getCost(const finenav::Position3D& pos, const finenav::GridMap<Voxel>& map) const {
    if (map.isInside(pos)) {
        finenav::Index index = map.getIndex(pos);
        return costmap_image_.at<uchar>(
            index.y() + map.getSize().y() / 2,
            index.x() + map.getSize().x() / 2);
    }
    return 254;
}

int TerrainAnalyzer::ComputeCost(int distance) const {
    const double factor = std::exp(-1.0 * cost_scaling_factor_ * (distance * cached_resolution_));
    return static_cast<int>(253 * factor);
}

void TerrainAnalyzer::TerrainAnalysis(float robot_pose_z, const finenav::GridMap<Voxel>& map) {
    int sizeX_min = map.getMinIndex().x();
    int sizeX_max = map.getMaxIndex().x();
    int sizeY_min = map.getMinIndex().y();
    int sizeY_max = map.getMaxIndex().y();
    size_t size_x      = map.getSize().x();
    size_t size_y      = map.getSize().y();
    size_t half_size_x = size_x / 2;
    size_t half_size_y = size_y / 2;

    ground_array_ = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);

    std::vector<Voxel> voxels_along_z;
    voxels_along_z.reserve(map.getSize().z());

    std::vector<Voxel> valid_z_values;
    valid_z_values.reserve(map.getSize().z());

    // ── Simple Z-threshold mode ──────────────────────────────────────────
    // Each (x,y) column is an obstacle if the highest voxel exceeds
    //   robot_pose_z + z_obstacle_threshold_.
    // No gradient analysis, no ground-height gap detection.
    if (use_simple_z_threshold_) {
        for (int x = sizeX_min; x <= sizeX_max; ++x) {
            for (int y = sizeY_min; y <= sizeY_max; ++y) {
                map.getVoxelsAlongZ(x, y, voxels_along_z);

                float max_z = -std::numeric_limits<float>::infinity();
                float min_z =  std::numeric_limits<float>::infinity();
                for (const auto& voxel : voxels_along_z) {
                    if (!std::isnan(voxel.z_height)) {
                        max_z = std::max(max_z, voxel.z_height);
                        min_z = std::min(min_z, voxel.z_height);
                    }
                }

                const size_t px = x + half_size_x;
                const size_t py = y + half_size_y;

                if (std::isfinite(max_z) && (max_z - robot_pose_z) > z_obstacle_threshold_) {
                    passability_image_.at<uchar>(py, px) = 254;  // obstacle
                } else {
                    passability_image_.at<uchar>(py, px) = 0;    // free
                }

                ground_array_(px, py) = std::isfinite(min_z) ? min_z : NAN;
            }
        }

        ApplyGroundMeanFilter(ground_array_, 3);

        // Fill NaN cells near the map center with robot_pose_z
        for (size_t x = 0; x < size_x; ++x) {
            for (size_t y = 0; y < size_y; ++y) {
                if (std::isnan(ground_array_(x, y))) {
                    double dist_to_center = std::hypot(
                        static_cast<double>(x) - static_cast<double>(half_size_x),
                        static_cast<double>(y) - static_cast<double>(half_size_y)) * cached_resolution_;
                    if (dist_to_center < nan_fill_radius_) {
                        ground_array_(x, y) = robot_pose_z;
                    }
                }
            }
        }

        // Fuse global occupancy grid (same as complex mode)
        if (global_map_) {
            const double map_res = global_map_->info.resolution;
            const double origin_x = global_map_->info.origin.position.x;
            const double origin_y = global_map_->info.origin.position.y;
            const int map_w = global_map_->info.width;
            const int map_h = global_map_->info.height;

            for (size_t x = 0; x < size_x; ++x) {
                for (size_t y = 0; y < size_y; ++y) {
                    finenav::Index idx(
                        static_cast<int>(x) - static_cast<int>(half_size_x),
                        static_cast<int>(y) - static_cast<int>(half_size_y),
                        0);
                    if (map.isInside(idx)) {
                        auto pos = map.getPosition(idx);
                        int gx = std::floor((pos.x() - origin_x) / map_res);
                        int gy = std::floor((pos.y() - origin_y) / map_res);
                        if (gx >= 0 && gx < map_w && gy >= 0 && gy < map_h) {
                            int g_idx = gy * map_w + gx;
                            if (global_map_->data[g_idx] == 100) {
                                passability_image_.at<uchar>(y, x) = 254;
                            }
                        }
                    }
                }
            }
        }
        return;
    }

    // ── Original complex terrain analysis ────────────────────────────────
    for (int x = sizeX_min; x <= sizeX_max; ++x) {
        for (int y = sizeY_min; y <= sizeY_max; ++y) {
            
            map.getVoxelsAlongZ(x, y, voxels_along_z);

            valid_z_values.clear();
            for (const auto& voxel : voxels_along_z) {
                if (!std::isnan(voxel.z_height)) {
                    valid_z_values.push_back(voxel);
                }
            }

            float nearest_ground = std::numeric_limits<float>::max();
            if (!valid_z_values.empty()) {
                for (size_t i = 0; i < valid_z_values.size() - 1; ++i) {
                    if(valid_z_values[i + 1].z_height - valid_z_values[i].z_height > robot_height_) {
                        if (fabs(valid_z_values[i].z_height - robot_pose_z) < fabs(nearest_ground - robot_pose_z)) {
                            nearest_ground = valid_z_values[i].z_height;
                        }
                    }
                }
                if (fabs(valid_z_values.back().z_height - robot_pose_z) < fabs(nearest_ground - robot_pose_z)) {
                    nearest_ground = valid_z_values.back().z_height;
                }
            } else {
                nearest_ground = NAN;
            }
            ground_array_(x + half_size_x, y + half_size_y) = nearest_ground;
        }
    }

    ApplyGroundMeanFilter(ground_array_, 3);

    // 将地图中心附近一定范围内仍为 NAN 的栅格填充为 robot_pose_z
    for (size_t x = 0; x < size_x; ++x) {
        for (size_t y = 0; y < size_y; ++y) {
            if (std::isnan(ground_array_(x, y))) {
                double dist_to_center = std::hypot(
                    static_cast<double>(x) - static_cast<double>(half_size_x),
                    static_cast<double>(y) - static_cast<double>(half_size_y)) * cached_resolution_;
                if (dist_to_center < nan_fill_radius_) {
                    ground_array_(x, y) = robot_pose_z;
                }
            }
        }
    }

    for (size_t x = 0; x < size_x; ++x) {
        for (size_t y = 0; y < size_y; ++y) {
            int dx[4] = {0, 1, 0, -1};
            int dy[4] = {1, 0, -1, 0};

            float terrain_value  = 0;
            float current_ground = ground_array_(x, y);
            if (!std::isnan(current_ground)) {
                for (int i = 0; i < 4; ++i) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];
                    if (nx < 0 || nx > static_cast<int>(size_x) - 1 ||
                        ny < 0 || ny > static_cast<int>(size_y) - 1) continue;
                    float neighbor_ground = ground_array_(nx, ny);
                    if (!std::isnan(neighbor_ground)) {
                        if (fabs(current_ground - neighbor_ground) > max_gradient_) {
                            terrain_value = 254;
                            break;
                        }
                    }
                }
            }
            passability_image_.at<uchar>(y, x) = terrain_value;
        }
    }

    // 融合全局占据地图 (0/100 二值图)
    if (global_map_) {
        const double map_res = global_map_->info.resolution;
        const double origin_x = global_map_->info.origin.position.x;
        const double origin_y = global_map_->info.origin.position.y;
        const int map_w = global_map_->info.width;
        const int map_h = global_map_->info.height;

        for (size_t x = 0; x < size_x; ++x) {
            for (size_t y = 0; y < size_y; ++y) {
                finenav::Index idx(
                    static_cast<int>(x) - static_cast<int>(half_size_x),
                    static_cast<int>(y) - static_cast<int>(half_size_y),
                    0);

                if (map.isInside(idx)) {
                    auto pos = map.getPosition(idx);
                    int gx = std::floor((pos.x() - origin_x) / map_res);
                    int gy = std::floor((pos.y() - origin_y) / map_res);

                    if (gx >= 0 && gx < map_w && gy >= 0 && gy < map_h) {
                        int g_idx = gy * map_w + gx;
                        if (global_map_->data[g_idx] == 100) {
                            passability_image_.at<uchar>(y, x) = 254; // 标记为不可通行
                        }
                    }
                }
            }
        }
    }
}

void TerrainAnalyzer::ApplyGroundMeanFilter(Eigen::ArrayXXf& ground_array, int kernel_size) {
    int rows = ground_array.rows();
    int cols = ground_array.cols();
    Eigen::ArrayXXf filtered_array = ground_array;
    int half_k = kernel_size / 2;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (std::isnan(ground_array(r, c))) continue;

            float sum = 0.0f;
            int count = 0;

            for (int kr = -half_k; kr <= half_k; ++kr) {
                for (int kc = -half_k; kc <= half_k; ++kc) {
                    int nr = r + kr;
                    int nc = c + kc;
                    if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                        float val = ground_array(nr, nc);
                        if (!std::isnan(val)) {
                            sum += val;
                            count++;
                        }
                    }
                }
            }

            if (count > 0) {
                filtered_array(r, c) = sum / count;
            }
        }
    }

    ground_array = filtered_array;
}

void TerrainAnalyzer::MapInflation() {
    cv::Mat distance_transform = DistanceTransform();
    for (int x = 0; x < passability_image_.cols; ++x) {
        for (int y = 0; y < passability_image_.rows; ++y) {
            if (passability_image_.at<uchar>(y, x) == 0) {
                float distance = distance_transform.at<float>(y, x);
                costmap_image_.at<uchar>(y, x) = static_cast<uchar>(ComputeCost(distance));
            } else {
                costmap_image_.at<uchar>(y, x) = 254;
            }
        }
    }
}

cv::Mat TerrainAnalyzer::DistanceTransform() {
    cv::Mat passability_binary = 254 - passability_image_;
    cv::Mat distance_transform;
    cv::distanceTransform(passability_binary, distance_transform, cv::DIST_L2, 3);
    return distance_transform;
}

void TerrainAnalyzer::publishMaps(const finenav::GridMap<Voxel>& map) {
    if (!node_) return;

    const size_t size_x      = map.getSize().x();
    const size_t size_y      = map.getSize().y();
    const size_t half_size_x = size_x / 2;
    const size_t half_size_y = size_y / 2;
    const auto   stamp       = node_->now();

    // 像素 (px, py) → 地图 Index (px - half_size_x, py - half_size_y, 0)
    // 与 TerrainAnalysis 中 ground_array_(x+half_size_x, y+half_size_y) 对应

    // ---- passability_map ----
    // 颜色编码: 0（可通行）→ 绿色; 254（障碍）→ 红色; 其余 → 灰色
    if (passability_helper_) {
        passability_helper_->reserve(size_x * size_y);
        passability_helper_->reserve(size_x * size_y);
        for (size_t px = 0; px < size_x; ++px) {
            for (size_t py = 0; py < size_y; ++py) {
                finenav::Index idx(
                    static_cast<int>(px) - static_cast<int>(half_size_x),
                    static_cast<int>(py) - static_cast<int>(half_size_y),
                    0);
                if (!map.isInside(idx)) continue;
                const auto pos = map.getPosition(idx);
                const uint8_t val = passability_image_.at<uint8_t>(
                    static_cast<int>(py), static_cast<int>(px));
                std::array<uint8_t, 3> color;
                if (val == 0) {
                    color = {0, 200, 0};    // 可通行 → 绿
                } else if (val >= 254) {
                    color = {200, 0, 0};    // 障碍 → 红
                } else {
                    color = {120, 120, 120}; // 其他 → 灰
                }
                passability_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    0.0f, color);
            }
        }
        passability_helper_->publish(stamp);
    }

    // ---- costmap_cloud ----
    // 颜色编码: cost 0（低代价）→ 绿色; cost 254（高代价/障碍）→ 红色
    if (costmap_helper_) {
        costmap_helper_->reserve(size_x * size_y);
        for (size_t px = 0; px < size_x; ++px) {
            for (size_t py = 0; py < size_y; ++py) {
                finenav::Index idx(
                    static_cast<int>(px) - static_cast<int>(half_size_x),
                    static_cast<int>(py) - static_cast<int>(half_size_y),
                    0);
                if (!map.isInside(idx)) continue;
                const auto pos = map.getPosition(idx);
                const uint8_t val = costmap_image_.at<uint8_t>(
                    static_cast<int>(py), static_cast<int>(px));
                // 绿(低代价) → 红(高代价)
                if(val==254){
                    costmap_helper_->addPoint(
                        static_cast<float>(pos.x()),
                        static_cast<float>(pos.y()),
                        0.0f, {0, 0, 0}); // 障碍 → 红
                }else{
                const std::array<uint8_t, 3> color = {val, static_cast<uint8_t>(255 - val), 0};
                costmap_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    0.0f, color);
                }
            }
        }
        costmap_helper_->publish(stamp);
    }

    // ---- ground_cloud ----
    // 每个有效 ground 格子以实际 z 高度发布，颜色按高度热力图（蓝低→红高）
    if (ground_helper_ && ground_array_.size() > 0) {
        ground_helper_->reserve(size_x * size_y);
        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
        for (size_t px = 0; px < size_x; ++px) {
            for (size_t py = 0; py < size_y; ++py) {
                float z = ground_array_(px, py);
                if (!std::isnan(z)) {
                    z_min = std::min(z_min, z);
                    z_max = std::max(z_max, z);
                }
            }
        }
        const float z_range = (z_max > z_min) ? (z_max - z_min) : 1.0f;
        // jet colormap: blue(0) → cyan → green → yellow → red(1)
        auto jet = [](float t) -> std::array<uint8_t, 3> {
            t = std::clamp(t, 0.0f, 1.0f);
            float r = std::clamp(1.5f - std::abs(4.0f * t - 3.0f), 0.0f, 1.0f);
            float g = std::clamp(1.5f - std::abs(4.0f * t - 2.0f), 0.0f, 1.0f);
            float b = std::clamp(1.5f - std::abs(4.0f * t - 1.0f), 0.0f, 1.0f);
            return {static_cast<uint8_t>(255 * r),
                    static_cast<uint8_t>(255 * g),
                    static_cast<uint8_t>(255 * b)};
        };
        for (size_t px = 0; px < size_x; ++px) {
            for (size_t py = 0; py < size_y; ++py) {
                float z = ground_array_(px, py);
                if (std::isnan(z)) continue;
                finenav::Index idx(
                    static_cast<int>(px) - static_cast<int>(half_size_x),
                    static_cast<int>(py) - static_cast<int>(half_size_y),
                    0);
                if (!map.isInside(idx)) continue;
                const auto pos = map.getPosition(idx);
                ground_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    z, jet((z - z_min) / z_range));
            }
        }
        ground_helper_->publish(stamp);
    }
}