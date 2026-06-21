// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "height_mapview.hpp"

HeightMapView::HeightMapView(
    const finenav::GridMap<Voxel>& map,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<HeightAnalyzer> height_analyzer,
    std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper)
    : map_(map)
    , node_(node)
    , height_analyzer_(height_analyzer)
    , local_map_helper_(std::move(local_map_helper))
{
    collision_model_ = std::make_unique<finenav::OBBCollisionModel>(
        finenav::Vector3D(0.5, 0.5, 0.2));
}

bool HeightMapView::isTrackingUnknown() const { return is_tracking_unknown_; }
bool HeightMapView::considerFootprint() const  { return consider_footprint_; }
std::string HeightMapView::getBaseFrameID()  const { return "map"; }
float HeightMapView::getRadius() const { return collision_model_->getOuterDiameter(); }

bool HeightMapView::isCollision(float x, float y, float theta) const {
    int pose_cost = getCost(finenav::Position3D{x, y, 0.0});
    finenav::Pose pose = createPose2D(x, y, theta);
    int possible_collision_cost = ComputeCost(
        static_cast<int>(collision_model_->getOuterDiameter() / 2.0 / map_.getResolution()));
    if (pose_cost == 254) {
        return true;
    }
    if (pose_cost >= possible_collision_cost) {
        return collision_model_->checkCollision(pose, CollisionRule());
    }
    return false;
}

int HeightMapView::getCost(const finenav::Position3D& pos) const {
    return height_analyzer_->getCost(pos, map_);
}

float HeightMapView::costAtPose(float x, float y, float theta) const {
    finenav::Pose pose = createPose2D(x, y, theta);
    return static_cast<float>(collision_model_->checkCost(
        pose, [this](const finenav::Position3D& p) { return getCost(p); }));
}

std::function<bool(const finenav::Position3D&)> HeightMapView::CollisionRule() const {
    return [this](const finenav::Position3D& pos) {
        const int cost = getCost(pos);
        if (cost == 254) return true;
        if (cost == 255) return is_tracking_unknown_ ? false : true;
        return false;
    };
}

int HeightMapView::ComputeCost(int distance) const {
    return height_analyzer_->ComputeCost(distance);
}

finenav::Pose HeightMapView::createPose2D(double x, double y, double theta) const {
    finenav::Pose pose = finenav::Pose::Identity();
    pose.linear()      = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())
                             .toRotationMatrix();
    pose.translation() = Eigen::Vector3d(x, y, 0.0);
    return pose;
}
