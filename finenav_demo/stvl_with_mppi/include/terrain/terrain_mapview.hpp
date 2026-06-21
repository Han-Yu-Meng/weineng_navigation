// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"
#include "grid_map.hpp"
#include "terrain/terrain_analysis.hpp"
#include "finenav_util/cloud_publish_helper.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
using namespace mppi;
using std::chrono_literals::operator"" ms;

class SimpleGridMapView : public nav2_mppi_controller::IMapView {
public:
    /**
     * @param local_map_helper 可选，由外部（main）持有并配置好，用于发布局部地图点云。
     *                         传 nullptr 则不发布。
     */
    explicit SimpleGridMapView(
        const finenav::GridMap<Voxel>& map,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<TerrainAnalyzer> terrain_analyzer,
        std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper = nullptr);

    bool isTrackingUnknown() const override;
    bool considerFootprint() const override;
    bool isCollision(float x, float y, float theta) const override;
    int getCost(const finenav::Position3D& pos) const override;
    float getRadius() const override;
    float costAtPose(float x, float y, float theta) const override;
    std::string getBaseFrameID() const override;

    void publishLocalMap();

private:
    std::function<bool(const finenav::Position3D&)> CollisionRule() const;
    int ComputeCost(int distance) const;
    finenav::Pose createPose2D(double x, double y, double theta) const;

    std::shared_ptr<TerrainAnalyzer> terrain_analyzer_;
    const finenav::GridMap<Voxel>& map_;
    double inscribed_radius_;
    bool is_tracking_unknown_{false};
    bool consider_footprint_{true};
    Eigen::ArrayXXf ground_array_;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper_;

    std::unique_ptr<finenav::OBBCollisionModel> collision_model_;
};
