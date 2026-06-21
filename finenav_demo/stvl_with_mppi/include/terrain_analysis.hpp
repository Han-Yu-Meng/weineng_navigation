// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "grid_map.hpp"
#include "finenav_core/map/map_server.hpp"
#include <cmath>
#include <span>
#include <algorithm>
#include <cstring>

#include <opencv2/opencv.hpp>
#include "voxel.hpp"

#include "finenav_util/cloud_publish_helper.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>


class TerrainAnalyzer {
public:
    /**
     * @brief 构造函数。map_server 仅用于查询地图初始尺寸，不存储。
     *        passability_helper / costmap_helper 由外部（main）持有，TerrainAnalyzer
     *        仅持有非拥有指针，生命周期由调用方保证。
     */
    explicit TerrainAnalyzer(
        const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper = nullptr,
        std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper = nullptr);

    /**
     * @brief 执行一次完整的地形分析与膨胀，并发布可视化点云。
     *        必须在 MapServer 的 postUpdateHook 中调用，此时调用方已持有写锁，
     *        本函数直接使用传入的 map 引用，不会再次获取锁。
     */
    void update(float robot_pose_z, const finenav::GridMap<Voxel>& map);

    /**
     * @brief 根据世界坐标查询代价值。
     */
    int getCost(const finenav::Position3D& pos, const finenav::GridMap<Voxel>& map) const;

    /**
     * @brief 将距离（栅格数）转换为代价值，使用缓存的分辨率，无需访问地图。
     */
    int ComputeCost(int distance) const;


private:
    void ApplyGroundMeanFilter(Eigen::ArrayXXf& ground_array, int kernel_size);

    void TerrainAnalysis(float robot_pose_z, const finenav::GridMap<Voxel>& map);
    void MapInflation();
    cv::Mat DistanceTransform();
    void publishMaps(const finenav::GridMap<Voxel>& map);

    rclcpp::Node::SharedPtr node_;  // 仅用于获取时间戳
    std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper_;
    std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper_;

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;

    cv::Mat passability_image_;
    cv::Mat costmap_image_;
    double cost_scaling_factor_{1.0};
    double robot_height_{1.0};
    double max_gradient_{0.8};
    double cached_resolution_{0.1};
};
