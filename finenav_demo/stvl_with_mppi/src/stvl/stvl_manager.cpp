// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "stvl/stvl_manager.hpp"

using namespace finenav;

StvlManager::StvlManager(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
    node_->declare_parameter<double>("stvl_manager.decay_time_sec", 1000000.0);
    node_->declare_parameter<double>("stvl_manager.decay_time_fov_sec", 1000000.0);
    node_->declare_parameter<double>("stvl_manager.min_range", 0.0);
    node_->declare_parameter<double>("stvl_manager.max_range", 50.0);
    node_->declare_parameter<double>("stvl_manager.min_angle", 0.0);
    node_->declare_parameter<double>("stvl_manager.max_angle", M_PI / 3.0);

    node_->get_parameter("stvl_manager.decay_time_sec", decay_time_sec_);
    node_->get_parameter("stvl_manager.decay_time_fov_sec", decay_time_fov_sec_);
    node_->get_parameter("stvl_manager.min_range", min_range_);
    node_->get_parameter("stvl_manager.max_range", max_range_);
    node_->get_parameter("stvl_manager.min_angle", min_angle_);
    node_->get_parameter("stvl_manager.max_angle", max_angle_);
}

bool StvlManager::isInFrustum(const Eigen::Isometry3d & T_map_body, const Position & p_world,
                               double min_range, double max_range,
                               double min_angle, double max_angle)
{
    const Eigen::Vector3d origin = T_map_body.translation();
    const Eigen::Vector3d v_world = p_world - origin;

    const double r = v_world.norm();
    if (r <= 0.0 || r < min_range || r > max_range) {
        return false;
    }

    const Eigen::Vector3d v_sensor = T_map_body.linear().transpose() * v_world;
    const double norm = v_sensor.norm();
    if (norm <= 0.0) {
        return false;
    }

    const double cos_theta = std::clamp(v_sensor.z() / norm, -1.0, 1.0);
    const double theta = std::acos(cos_theta);
    return theta >= min_angle && theta <= max_angle;
}

void StvlManager::PruneExpiredCells(GridMap<Voxel> & map, const Eigen::Isometry3d & T_map_body)
{
    const rclcpp::Time now = node_->get_clock()->now();

    for (auto it = dynamic_cells_.begin(); it != dynamic_cells_.end();) {
        const Position & pos = *it;

        if (!map.isInside(pos)) {
            it = dynamic_cells_.erase(it);
            continue;
        }

        Voxel & cell = map.atPosition(pos);
        if (cell.timestamp.nanoseconds() == 0) {    // TODO: 按逻辑不应出现，后续排查
            it = dynamic_cells_.erase(it);
            continue;
        }

        const double age_sec = (now - cell.timestamp).seconds();
        if (age_sec > decay_time_sec_) {
            cell = Voxel{NAN, false, rclcpp::Time{}};
            it = dynamic_cells_.erase(it);
            continue;
        }

        const bool in_fov = isInFrustum(T_map_body, pos, min_range_, max_range_, min_angle_, max_angle_);
        const double active_decay = in_fov ? decay_time_fov_sec_ : decay_time_sec_;

        if (age_sec > active_decay) {
            cell = Voxel{NAN, false, rclcpp::Time{}};
            it = dynamic_cells_.erase(it);
        } else {
            ++it;
        }
    }

}

void StvlManager::InsertPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, GridMap<Voxel> & map,
                                   const Eigen::Isometry3d & T_map_body) {
    const rclcpp::Time now = node_->get_clock()->now();
    for (const auto & pt : cloud) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        // 点云已在车体坐标系下，直接用 T_map_body 变换到 map 系
        const Eigen::Vector3d p_world = T_map_body * Eigen::Vector3d(pt.x, pt.y, pt.z);
        const Position hit_pos(p_world.x(), p_world.y(), p_world.z());
        if (!map.isInside(hit_pos)) {
            continue;
        }
        const Index idx = map.getIndex(hit_pos);
        Voxel & cell = map.at(idx);
        if (!cell.is_static) {
            cell.z_height = static_cast<float>(p_world.z());
            cell.timestamp = now;
            dynamic_cells_.insert(map.getPosition(idx));
        }
    }
}