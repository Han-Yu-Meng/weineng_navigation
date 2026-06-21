// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "imap_view.hpp"
#include "occ_grid_2d_map.hpp"

namespace astar {

/**
 * @brief OccGridMapView — 基于 ROS 2 订阅模式的 IMapView 实现
 * 负责自动监听地图话题并为 A* 规划器提供数据查询。
 */
class OccGridMapView : public IMapView {
public:
    /**
     * @param node ROS 2 节点指针
     * @param topic_name 地图话题名称 (如 "/map" 或 "/grid_map")
     */
    explicit OccGridMapView(
        rclcpp::Node* node,
        const std::string& topic_name = "map");

    explicit OccGridMapView(
        const finenav::OccGrid2DMap& map,
        int threshold = 50,
        double lethal_radius = 0.1,
        double inflation_radius = 0.55,
        double cost_scaling_factor = 10.0);

    // --- IMapView 接口重写 ---

    /// 判断世界坐标 p 是否可通行
    bool   isWalkable(const finenav::Position3D& p) const override;

    /// 获取世界坐标 p 的额外代价 (0.0-1.0)
    double getCost(const finenav::Position3D& p)    const override;

    /// 获取地图分辨率 (米/格)
    double getStepSize()                            const override;

private:
    // --- 内部处理函数 ---

    /// 是否已经收到了地图数据 (仅限内部逻辑使用)
    bool hasMap() const;

    /// 订阅回调
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief 将原始 OccupancyGrid 转换为内部二值地图 (0: 空闲, 254: 占据)
     */
    void binarizeMap(const nav_msgs::msg::OccupancyGrid& msg, int threshold);

    /**
     * @brief 对二值地图进行硬膨胀处理
     * @param lethal_radius 膨胀半径 (米)
     */
    void applyHardInflation(double lethal_radius);

    /**
     * @brief 计算并生成最终的代价地图 (Nav2 标准)
     */
    void updateCostMap(double lethal_radius, double inflation_radius, double cost_scaling_factor);

    // 订阅对象
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;

    // 地图元数据
    double resolution_ = 0.05;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    // 二值地图与代价地图
    cv::Mat binary_map_;
    cv::Mat cost_map_;

    bool has_received_map_ = false;

    rclcpp::Node* node_ptr_;
};

} // namespace astar

