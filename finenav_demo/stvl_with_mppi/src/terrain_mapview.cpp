// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-27.
//

#include "terrain_mapview.hpp"


SimpleGridMapView::SimpleGridMapView(
    const finenav::GridMap<Voxel>& map,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<TerrainAnalyzer> terrain_analyzer,
    std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper)
    : map_(map)
    , node_(node)
    , terrain_analyzer_(terrain_analyzer)
    , local_map_helper_(std::move(local_map_helper))
{
    collision_model_ = std::make_unique<finenav::OBBCollisionModel>(finenav::Vector3D(1.4, 0.8, 0.3));
}

bool SimpleGridMapView::isTrackingUnknown() const {
    return is_tracking_unknown_;
}

bool SimpleGridMapView::considerFootprint() const {
    return consider_footprint_;
}

bool SimpleGridMapView::isCollision(float x, float y, float theta) const {
    int pose_cost = getCost(finenav::Position3D{x, y, 0.0});
    finenav::Pose pose = createPose2D(x, y, theta);
    int possible_collision_cost = ComputeCost(collision_model_->getOuterDiameter() / 2 / map_.getResolution());
    if (pose_cost == 254) {
        return true;
    }
    if (pose_cost >= possible_collision_cost) {
        return collision_model_->checkCollision(pose, CollisionRule());
    }
    return false;
}

int SimpleGridMapView::getCost(const finenav::Position3D& pos) const {
    return terrain_analyzer_->getCost(pos, map_);
}

float SimpleGridMapView::getRadius() const {
    return collision_model_->getOuterDiameter();
}

float SimpleGridMapView::costAtPose(float x, float y, float theta) const {
    finenav::Pose pose = createPose2D(x, y, theta);
    return collision_model_->checkCost(pose,[this](const finenav::Position3D& pos) {
        return this->getCost(pos);
    });
}

std::string SimpleGridMapView::getBaseFrameID() const {
    return "map";
}

void SimpleGridMapView::publishLocalMap() {
    if (!local_map_helper_ || !node_) return;

    const size_t size_x = map_.getSize().x();
    const size_t size_y = map_.getSize().y();
    local_map_helper_->reserve(size_x * size_y);

    for (int x = map_.getMinIndex().x(); x <= map_.getMaxIndex().x(); ++x) {
        for (int y = map_.getMinIndex().y(); y < map_.getMaxIndex().y(); ++y) {
            for (int z = map_.getMinIndex().z(); z < map_.getMaxIndex().z(); ++z) {
                finenav::Position3D pos = map_.getPosition({x, y, z});
                const float z_height = map_.atPosition(pos).z_height;
                if (!std::isfinite(z_height)) continue;
                // 用代价值编码颜色：低代价绿，高代价红
                const int cost = getCost(pos);
                const uint8_t cv = static_cast<uint8_t>(std::clamp(cost, 0, 255));
                const std::array<uint8_t, 3> color = {cv, static_cast<uint8_t>(255 - cv), 0};
                local_map_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    z_height,
                    color);
            }
        }
    }
    local_map_helper_->publish(node_->now());
}

std::function<bool(const finenav::Position3D&)> SimpleGridMapView::CollisionRule() const {
    return [this](const finenav::Position3D& pos) {
        int pose_cost = this->getCost(pos);
        switch (static_cast<unsigned char>(pose_cost)) {
            case 254:
                return true;
            case 255:
                return is_tracking_unknown_ ? false : true;
        }
        return false;
    };
}

int SimpleGridMapView::ComputeCost(int distance) const {
    return terrain_analyzer_->ComputeCost(distance);
}

finenav::Pose SimpleGridMapView::createPose2D(double x, double y, double theta) const{
    finenav::Pose pose = finenav::Pose::Identity();
    pose.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(x, y, 0.0);
    return pose;
}
