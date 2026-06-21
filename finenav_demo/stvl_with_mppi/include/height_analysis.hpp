// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "grid_map.hpp"
#include "finenav_core/map/map_server.hpp"
#include "voxel.hpp"
#include "finenav_util/cloud_publish_helper.hpp"

#include <cmath>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief HeightAnalyzer — 基于障碍物高度区间的地图分析器
 *
 * 与 TerrainAnalyzer（基于梯度）的唯一区别是障碍判断逻辑：
 *   - TerrainAnalyzer：相邻格子高度差 > max_gradient → 障碍
 *   - HeightAnalyzer：某格存在 z_height ∈ [robot_z + min_h, robot_z + max_h] 的 Voxel → 障碍
 *
 * 接口、膨胀算法、可视化发布与 TerrainAnalyzer 完全一致。
 */
class HeightAnalyzer {
public:
    explicit HeightAnalyzer(
        const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper = nullptr,
        std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper     = nullptr);

    /** 在 MapServer 的 postUpdateHook 中调用，执行一次完整的高度分析与膨胀。 */
    void update(float robot_pose_z, const finenav::GridMap<Voxel>& map);

    /** 根据世界坐标查询代价值（与 TerrainAnalyzer::getCost 接口完全一致）。 */
    int getCost(const finenav::Position3D& pos, const finenav::GridMap<Voxel>& map) const;

    /** 将距离（栅格数）转换为代价值。 */
    int ComputeCost(int distance) const;

private:
    void HeightAnalysis(float robot_pose_z, const finenav::GridMap<Voxel>& map);
    void MapInflation();
    cv::Mat DistanceTransform();
    void publishMaps(const finenav::GridMap<Voxel>& map);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper_;
    std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper_;

    cv::Mat passability_image_;
    cv::Mat costmap_image_;

    double cost_scaling_factor_{10.0};
    double min_obstacle_height_{0.1};   ///< 相对于机器人 z 的最小障碍高度 [m]
    double max_obstacle_height_{2.0};   ///< 相对于机器人 z 的最大障碍高度 [m]
    double cached_resolution_{0.1};
};
