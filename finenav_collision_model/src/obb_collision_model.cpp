// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_collision_model/obb_collision_model.hpp"
#include <stdexcept>

namespace finenav {

OBBCollisionModel::OBBCollisionModel(const Vector3D& size, const Pose& local_pose, const Policy& policy)
    : OBBCollisionModel(-size / 2.0, size / 2.0, local_pose, policy) {}

OBBCollisionModel::OBBCollisionModel(const Vector3D& min, const Vector3D& max,
                                      const Pose& local_pose, const Policy& policy)
    : size_(max - min), outer_diameter_(0.0), local_pose_(local_pose), policy_(policy) {
    if (policy_.step <= 0.0) {
        throw std::invalid_argument(
            "OBBCollisionModel: policy.step must be greater than 0");
    }
    if ((max - min).minCoeff() < 0.0) {
        throw std::invalid_argument(
            "OBBCollisionModel: max must be >= min element-wise");
    }
    min_ = min;
    max_ = max;
    outer_diameter_ = (max_ - min_).norm();
}

double OBBCollisionModel::getOuterDiameter() const {
    return outer_diameter_;
}

void OBBCollisionModel::precompute(const Pose& pose) {
    cached_points_.clear();

    const double step = policy_.step;

    // 预先合并两次变换：OBB 本体系 → 机器人本体系 → 世界系
    // 避免在每个采样点重复计算 pose * local_pose_
    const Pose body_to_world = pose * local_pose_;

    // 遍历 OBB 的 6 个面（每个轴各 min/max 两个面）进行表面采样
    // 轴旋转技巧：对于固定轴 axis，u = (axis+1)%3，v = (axis+2)%3
    for (int axis = 0; axis < 3; ++axis) {
        const int u_axis = (axis + 1) % 3;
        const int v_axis = (axis + 2) % 3;

        // 当某轴尺寸为 0 时 min==max，只需采样 1 个面而非 2 个
        const double sides[2] = {min_[axis], max_[axis]};
        const int num_sides = (min_[axis] == max_[axis]) ? 1 : 2;
        for (int s = 0; s < num_sides; ++s) {
            const double side = sides[s];
            for (double u = min_[u_axis]; u <= max_[u_axis]; u += step) {
                for (double v = min_[v_axis]; v <= max_[v_axis]; v += step) {
                    Position3D local_pt;
                    local_pt[axis]   = side;
                    local_pt[u_axis] = u;
                    local_pt[v_axis] = v;
                    cached_points_.push_back(body_to_world * local_pt);
                }
                // 补全 v 方向最大边界（浮点步进可能漏掉 max_v）
                Position3D edge_v;
                edge_v[axis]   = side;
                edge_v[u_axis] = u;
                edge_v[v_axis] = max_[v_axis];
                cached_points_.push_back(body_to_world * edge_v);
            }
            // 补全 u 方向最大边界列（浮点步进可能漏掉 max_u）
            for (double v = min_[v_axis]; v <= max_[v_axis]; v += step) {
                Position3D edge_u;
                edge_u[axis]   = side;
                edge_u[u_axis] = max_[u_axis];
                edge_u[v_axis] = v;
                cached_points_.push_back(body_to_world * edge_u);
            }
            // 补全 (max_u, max_v) 角点（edge_v 和 edge_u 均未覆盖此交叉点）
            Position3D corner;
            corner[axis]   = side;
            corner[u_axis] = max_[u_axis];
            corner[v_axis] = max_[v_axis];
            cached_points_.push_back(body_to_world * corner);
        }
    }
}

void OBBCollisionModel::precomputeEdge(const Pose& pose) {
    edge_cached_points_.clear();

    const double step = policy_.step;
    const Pose body_to_world = pose * local_pose_;

    // OBB 共 12 条棱：对每个轴 a，沿 a 方向的棱由另外两轴各取 min/max 交叉组合固定
    // 轴旋转技巧：u = (a+1)%3，v = (a+2)%3
    for (int a = 0; a < 3; ++a) {
        const int u = (a + 1) % 3;
        const int v = (a + 2) % 3;

        const double u_sides[2] = {min_[u], max_[u]};
        const double v_sides[2] = {min_[v], max_[v]};
        // 当某垂直轴尺寸为 0 时 min==max，4 条棱退化为 2 条，避免重复采样
        const int num_u = (min_[u] == max_[u]) ? 1 : 2;
        const int num_v = (min_[v] == max_[v]) ? 1 : 2;

        for (int su = 0; su < num_u; ++su) {
            for (int sv = 0; sv < num_v; ++sv) {
                for (double t = min_[a]; t <= max_[a]; t += step) {
                    Position3D pt;
                    pt[a] = t;
                    pt[u] = u_sides[su];
                    pt[v] = v_sides[sv];
                    edge_cached_points_.push_back(body_to_world * pt);
                }
                // 补全终点（浮点步进可能漏掉 max_[a]）
                // 当棱长为 0 时 min_[a]==max_[a]，主循环已覆盖该点，无需再补
                if (min_[a] != max_[a]) {
                    Position3D endpoint;
                    endpoint[a] = max_[a];
                    endpoint[u] = u_sides[su];
                    endpoint[v] = v_sides[sv];
                    edge_cached_points_.push_back(body_to_world * endpoint);
                }
            }
        }
    }
}

} // namespace finenav
