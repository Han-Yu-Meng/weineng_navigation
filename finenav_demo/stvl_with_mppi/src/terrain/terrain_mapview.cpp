// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-27.
//

#include "terrain/terrain_mapview.hpp"


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
    // ── 碰撞模型参数：优先使用非对称模式（front/back/left/right/top/bottom）─────
    // 若 YAML 中设置了 collision_model.front，则使用非对称边界；
    // 否则回退到对称的 length/width/height 模式。
    if (node_->has_parameter("collision_model.front")) {
        // 非对称 OBB：直接指定 min/max 边界（机器人本体坐标系，X 前 Y 左 Z 上）
        const double front  = node_->get_parameter("collision_model.front").as_double();
        const double back   = node_->get_parameter("collision_model.back").as_double();
        const double left   = node_->get_parameter("collision_model.left").as_double();
        const double right  = node_->get_parameter("collision_model.right").as_double();
        const double top    = node_->get_parameter("collision_model.top").as_double();
        const double bottom = node_->get_parameter("collision_model.bottom").as_double();

        collision_model_ = std::make_unique<finenav::OBBCollisionModel>(
            finenav::Vector3D(-back, -right, -bottom),
            finenav::Vector3D(front, left, top));
    } else {
        // 对称 OBB（向后兼容）
        if (!node_->has_parameter("collision_model.length")) {
            node_->declare_parameter<double>("collision_model.length", 1.4);
            node_->declare_parameter<double>("collision_model.width",  0.8);
            node_->declare_parameter<double>("collision_model.height", 0.0);
        }
        const double length = node_->get_parameter("collision_model.length").as_double();
        const double width  = node_->get_parameter("collision_model.width").as_double();
        const double height = node_->get_parameter("collision_model.height").as_double();

        collision_model_ = std::make_unique<finenav::OBBCollisionModel>(
            finenav::Vector3D(length, width, height));
    }
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
        return collision_model_->checkCollisionEdge(pose, CollisionRule());
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
    return collision_model_->checkCostEdge(pose,[this](const finenav::Position3D& pos) {
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
