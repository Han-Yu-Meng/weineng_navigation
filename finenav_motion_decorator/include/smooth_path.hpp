// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"

namespace finenav {

/**
 * @brief 无状态路径平滑装饰器，参照 Nav2 SimpleSmoother 的迭代梯度下降平滑算法。
 *
 * 核心公式（对每个内部点 i，固定首尾不动）：
 *   y_i += data_w * (x_i - y_i) + smooth_w * (y_{i-1} + y_{i+1} - 2 * y_i)
 *
 * 其中：
 *   - x_i  = 原始路径点（锚定项，防止路径偏移过远）
 *   - y_i  = 当前平滑后的路径点
 *   - data_w   = 数据权重，越大越贴近原始路径
 *   - smooth_w = 平滑权重，越大曲线越光滑
 *
 * 特性：
 *   - 仅修改 poses 中的 position（x, y），不修改 z 和 orientation
 *   - 首尾点固定不动（保证起点终点不变）
 *   - 少于 3 个点时静默跳过（无法平滑）
 *   - 支持最大迭代次数和收敛容差控制
 *   - 需要 TRAJ_POSE 已填充，否则抛出 std::runtime_error
 *
 * 用法：
 *   ctx.ref_traj | finenav::SmoothPath();                    // 默认参数
 *   ctx.ref_traj | finenav::SmoothPath(0.2, 0.3, 1e-4, 500); // 自定义参数
 */
class SmoothPath : public IMotionDecorator {
public:
    /**
     * @brief 构造路径平滑装饰器
     * @param data_weight    数据项权重，控制平滑后路径对原始路径的贴合度 (默认 0.2)
     * @param smooth_weight  平滑项权重，控制路径光滑程度 (默认 0.3)
     * @param tolerance      收敛容差，所有点单次迭代的累计位移 < 此值则停止 (默认 1e-4)
     * @param max_iterations 最大迭代次数 (默认 1000)
     */
    explicit SmoothPath(
        double data_weight    = 0.2,
        double smooth_weight  = 0.3,
        double tolerance      = 1e-4,
        int    max_iterations = 1000);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    double data_w_;
    double smooth_w_;
    double tolerance_;
    int    max_iterations_;
};

} // namespace finenav
