// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
namespace finenav {
/**
 * @brief SingleShotPolicy — 每次 Action Goal 执行一次 plan()，完成后立即 succeed。
 *
 * 适用于离线规划、测试等"请求-响应"场景。
 * planner.reset() 在 execute() 开始时调用（Episode 边界）。
 */
struct SingleShotPolicy {};
/**
 * @brief TrackingPolicy — 每次 Action Goal 以固定频率循环调用 plan()，直到取消或目标到达。
 *
 * 适用于 MPPI 等需要持续追踪的在线控制器。
 * planner.reset() 仅在 execute() 开始时调用一次；循环内各帧 plan() 共享热启动状态。
 *
 * @param frequency_hz  控制循环频率（Hz），默认 10 Hz。
 */
struct TrackingPolicy {
    double frequency_hz = 10.0;
};
} // namespace finenav
