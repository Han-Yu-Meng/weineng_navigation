// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef STVL_MANAGER_HPP_
#define STVL_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_set>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "grid_map.hpp"
#include "voxel.hpp"

namespace finenav {

struct PositionHash {
    std::size_t operator()(const Position & p) const noexcept {
        std::size_t hx = std::hash<double>{}(p.x());
        std::size_t hy = std::hash<double>{}(p.y());
        std::size_t hz = std::hash<double>{}(p.z());
        auto hash_combine = [](std::size_t seed, std::size_t v) {
            seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            return seed;
        };
        std::size_t seed = 0;
        seed = hash_combine(seed, hx);
        seed = hash_combine(seed, hy);
        seed = hash_combine(seed, hz);
        return seed;
    }
};

struct PositionEqual {
    bool operator()(const Position & a, const Position & b) const noexcept {
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

using DynamicPosSet = std::unordered_set<Position, PositionHash, PositionEqual>;

class StvlManager {
public:
    /// @param node  ROS节点
    explicit StvlManager(rclcpp::Node::SharedPtr node);

    /// @param T_map_body  当前车体在地图坐标系下的位姿，由外部定位模块提供
    void PruneExpiredCells(GridMap<Voxel> & map, const Eigen::Isometry3d & T_map_body);

    /// @param T_map_body  当前车体在地图坐标系下的位姿，用于将点云从 base_link 变换到 map 系
    void InsertPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, GridMap<Voxel> & map,
                          const Eigen::Isometry3d & T_map_body);

private:
    static bool isInFrustum(const Eigen::Isometry3d & T_map_body, const Position & p_world,
                            double min_range, double max_range,
                            double min_angle, double max_angle);

    rclcpp::Node::SharedPtr node_;

    DynamicPosSet dynamic_cells_;
    double decay_time_sec_     = 1000000.0;
    double decay_time_fov_sec_ = 1000000.0;
    // FOV parameters
    double min_range_ = 0.0;
    double max_range_ = 50.0;
    double min_angle_ = 0.0;
    double max_angle_ = M_PI / 3.0;
};

} // namespace finenav

#endif // STVL_MANAGER_HPP_
