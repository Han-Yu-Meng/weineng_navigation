// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <functional>
#include <utility>
#include <algorithm>

#include <Eigen/Core>

namespace finenav::geometry {

struct FrustumSensorPose {
    // Homogeneous transform from sensor frame to world frame: p_world = T_ws * p_sensor_h
    // Layout: [ R_ws(3x3)  t_ws(3x1) ]
    //         [   0 0 0       1     ]
    Eigen::Matrix4d T_ws{Eigen::Matrix4d::Identity()};
};

struct FrustumRange {
    double min_range{0.0};
    double max_range{0.0};
};

struct FrustumAngles {
    // Angle between ray (sensor->point) and sensor Z axis (cone axis) must be in [min_angle, max_angle].
    // Typical cone: min_angle = 0, max_angle = half_aperture.
    double min_angle{0.0};
    double max_angle{0.0};
};

struct FrustumParams3D {
    FrustumSensorPose pose{};
    FrustumRange range{};
    FrustumAngles fov{};
};

/**
 * @brief Simple 3D frustum / cone helper (header-only, map-agnostic).
 *
 * Coordinate convention:
 *  - All points are expressed in a common world/map frame.
 *  - FrustumSensorPose::origin is the sensor position in this frame.
 *  - yaw: rotation around +Z (rad), 0 along +X, positive towards +Y.
 *  - pitch: elevation angle (rad), 0 in XY plane, positive towards +Z.
 */
class Frustum3D {
  public:
    Frustum3D() = default;

    explicit Frustum3D(const FrustumParams3D & params)
        : min_range_(params.range.min_range),
          max_range_(params.range.max_range),
          min_angle_(params.fov.min_angle),
          max_angle_(params.fov.max_angle) {
        setPose(params.pose.T_ws);
    }

    void setParams(const FrustumParams3D & params) {
        min_range_ = params.range.min_range;
        max_range_ = params.range.max_range;
        min_angle_ = params.fov.min_angle;
        max_angle_ = params.fov.max_angle;
        setPose(params.pose.T_ws);
    }

    const Eigen::Vector3d & origin() const { return origin_; }

    /**
     * @brief Check if a 3D point in world frame lies inside the frustum.
     */
    bool contains(const Eigen::Vector3d & p_world) const {
        Eigen::Vector3d v = p_world - origin_;
        const double r2 = v.squaredNorm();
        if (r2 <= 0.0) {
            return false; // at origin or invalid
        }
        const double r = std::sqrt(r2);
        if (r < min_range_ || r > max_range_) {
            return false;
        }

        // Transform to sensor local frame: x' forward, y' left, z' up
        Eigen::Vector3d v_s = R_ws_.transpose() * v;

        // Cone axis is +Z' of sensor frame. Compute angle between v_s and +Z'.
        const double v_norm = v_s.norm();
        if (v_norm <= 0.0) {
            return false;
        }

        const double cos_theta = v_s.z() / v_norm;  // dot(v_s, z_axis) / |v_s|
        const double clamped_cos = std::clamp(cos_theta, -1.0, 1.0);
        const double theta = std::acos(clamped_cos);

        if (theta < min_angle_ || theta > max_angle_) {
            return false;
        }

        return true;
    }

    bool contains(double x, double y, double z) const {
        return contains(Eigen::Vector3d{x, y, z});
    }

  private:
    void setPose(const Eigen::Matrix4d & T_ws) {
        // Extract rotation and translation from homogeneous transform
        R_ws_  = T_ws.topLeftCorner<3,3>();
        origin_ = T_ws.topRightCorner<3,1>();
    }

    Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
    double min_range_{0.0};
    double max_range_{0.0};
    double min_angle_{0.0};
    double max_angle_{0.0};
    Eigen::Matrix3d R_ws_{Eigen::Matrix3d::Identity()};
};



}  // namespace finenav::geometry
