// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_mppi_controller/controller.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"
#include "grid_map.hpp"
#include "height_analysis.hpp"
#include "finenav_util/cloud_publish_helper.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

using namespace mppi;
using std::chrono_literals::operator"" ms;

/**
 * @brief HeightMapView — 基于障碍物高度区间的 MPPI IMapView 实现
 *
 * 与 SimpleGridMapView（基于 TerrainAnalyzer 梯度）的唯一区别：
 *   getCost() 调用 HeightAnalyzer::getCost()，后者根据高度区间判断障碍。
 * 其余逻辑（OBB 碰撞、膨胀代价查询）完全与 SimpleGridMapView 一致。
 */
class HeightMapView : public nav2_mppi_controller::IMapView {
public:
    explicit HeightMapView(
        const finenav::GridMap<Voxel>& map,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<HeightAnalyzer> height_analyzer,
        std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper = nullptr);

    bool isTrackingUnknown() const override;
    bool considerFootprint() const override;
    bool isCollision(float x, float y, float theta) const override;
    int  getCost(const finenav::Position3D& pos)    const override;
    float getRadius()                               const override;
    float costAtPose(float x, float y, float theta) const override;
    std::string getBaseFrameID()                    const override;

private:
    std::function<bool(const finenav::Position3D&)> CollisionRule() const;
    int   ComputeCost(int distance) const;
    finenav::Pose createPose2D(double x, double y, double theta) const;

    std::shared_ptr<HeightAnalyzer> height_analyzer_;
    const finenav::GridMap<Voxel>&  map_;
    bool   is_tracking_unknown_{false};
    bool   consider_footprint_{true};

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper_;
    std::unique_ptr<finenav::OBBCollisionModel> collision_model_;
};
