// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include "finenav_collision_model/obb_collision_model.hpp"
#include <cmath>

using namespace finenav;

// ─── 辅助：始终返回 false 的 rule（无障碍） ───────────────────────────
auto free_rule  = [](const Position3D&) { return false; };
// 始终返回 true 的 rule（全障碍）
auto block_rule = [](const Position3D&) { return true;  };

// ─── 1. 非法参数：step <= 0 抛出异常 ─────────────────────────────────
TEST(OBBCollisionModelTest, InvalidStepThrows) {
    OBBCollisionPolicy bad_policy{0.0};
    EXPECT_THROW(OBBCollisionModel({1.0, 1.0, 1.0}, Pose::Identity(), bad_policy),
                 std::invalid_argument);

    OBBCollisionPolicy neg_policy{-0.1};
    EXPECT_THROW(OBBCollisionModel({1.0, 1.0, 1.0}, Pose::Identity(), neg_policy),
                 std::invalid_argument);
}

// ─── 2. 外径等于 OBB 空间对角线长度 ─────────────────────────────────
TEST(OBBCollisionModelTest, GetOuterDiameterReturnsBoxDiagonalLength) {
    OBBCollisionModel model({3.0, 4.0, 12.0});
    EXPECT_DOUBLE_EQ(model.getOuterDiameter(), 13.0);
}

// ─── 3. 未调用 precompute 直接 checkCollision 抛出异常 ──────────────
TEST(OBBCollisionModelTest, CheckCollisionBeforePrecomputeThrows) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    EXPECT_THROW(model.checkCollision(free_rule), std::logic_error);
}

// ─── 4. 无碰撞：rule 全返回 false ────────────────────────────────────
TEST(OBBCollisionModelTest, NoCollisionWhenFree) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    EXPECT_FALSE(model.checkCollision(pose, free_rule));
}

// ─── 5. 有碰撞：rule 全返回 true ─────────────────────────────────────
TEST(OBBCollisionModelTest, CollisionWhenBlocked) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    EXPECT_TRUE(model.checkCollision(pose, block_rule));
}

// ─── 6. 高频接口：precompute 后多次 checkCollision ───────────────────
TEST(OBBCollisionModelTest, HighFreqInterfaceReusesCache) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // 多次调用应返回相同结果
    EXPECT_FALSE(model.checkCollision(free_rule));
    EXPECT_FALSE(model.checkCollision(free_rule));
    EXPECT_TRUE(model.checkCollision(block_rule));
}

// ─── 7. 障碍在 OBB 范围内才碰撞（位置相关 rule）────────────────────
TEST(OBBCollisionModelTest, CollisionOnlyWhenObstacleIntersectsOBB) {
    // OBB: 1x1x1 m，中心在原点，半轴长 0.5m，X 面表面点 x = ±0.5
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 障碍在 x=2.0，远离 OBB → 无碰撞
    auto far_rule = [](const Position3D& p) { return p.x() > 1.9; };
    EXPECT_FALSE(model.checkCollision(pose, far_rule));

    // 障碍在 x=0.5（OBB 表面），预计有采样点触碰
    auto surface_rule = [](const Position3D& p) { return std::abs(p.x() - 0.5) < 1e-6; };
    EXPECT_TRUE(model.checkCollision(pose, surface_rule));
}

// ─── 8. OBB 带偏移的 local_pose ──────────────────────────────────────
TEST(OBBCollisionModelTest, LocalPoseOffset) {
    // OBB 在机器人本体系中偏移 (5, 0, 0)
    Pose local_pose = Pose::Identity();
    local_pose.translation() = Vector3D(5.0, 0.0, 0.0);

    OBBCollisionModel model({1.0, 1.0, 1.0}, local_pose);
    Pose robot_pose = Pose::Identity(); // 机器人在原点

    // OBB 中心在世界系 (5, 0, 0)，半轴长 0.5m，X 面表面点 x = 4.5 / 5.5
    // 障碍在原点附近 → 无碰撞
    auto near_origin = [](const Position3D& p) { return p.norm() < 0.6; };
    EXPECT_FALSE(model.checkCollision(robot_pose, near_origin));

    // 障碍在 x=5.0 附近 → 有碰撞
    auto near_offset = [](const Position3D& p) { return std::abs(p.x() - 5.0) < 0.6; };
    EXPECT_TRUE(model.checkCollision(robot_pose, near_offset));
}

// ─── 9. OBB 带旋转的 local_pose（绕 Z 轴旋转 90°）────────────────────
TEST(OBBCollisionModelTest, LocalPoseRotation) {
    // OBB 2x1x1（长轴沿 X），绕 Z 旋转 90° 后长轴变为 Y 方向
    Pose local_pose = Pose::Identity();
    local_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Vector3D::UnitZ()).toRotationMatrix();

    OBBCollisionModel model({2.0, 1.0, 1.0}, local_pose);
    Pose robot_pose = Pose::Identity();

    // 旋转后 OBB 长轴在 Y，Y 方向表面点应在 ±1.0
    // x 方向表面点应在 ±0.5（原来的 Y 半轴 = 0.5）
    auto y_surface = [](const Position3D& p) { return std::abs(std::abs(p.y()) - 1.0) < 1e-6; };
    EXPECT_TRUE(model.checkCollision(robot_pose, y_surface));

    // X 方向不应有到 ±1.0 的采样点（原长轴已旋转到 Y）
    auto x_far = [](const Position3D& p) { return std::abs(p.x()) > 0.9; };
    EXPECT_FALSE(model.checkCollision(robot_pose, x_far));
}

// ─── 10. 机器人移动后采样点跟随更新 ─────────────────────────────────
TEST(OBBCollisionModelTest, PrecomputeUpdatesWithPose) {
    OBBCollisionModel model({1.0, 1.0, 1.0});

    // 位姿1：OBB 在原点
    Pose pose1 = Pose::Identity();
    model.precompute(pose1);
    auto at_origin = [](const Position3D& p) { return p.norm() < 0.6; };
    EXPECT_TRUE(model.checkCollision(at_origin));

    // 位姿2：机器人平移到 (10, 0, 0)，OBB 中心也移到 (10, 0, 0)
    Pose pose2 = Pose::Identity();
    pose2.translation() = Vector3D(10.0, 0.0, 0.0);
    model.precompute(pose2);

    // 原点附近不再有碰撞
    EXPECT_FALSE(model.checkCollision(at_origin));

    // x=10 附近有碰撞
    auto at_new_pos = [](const Position3D& p) { return (p - Vector3D(10, 0, 0)).norm() < 0.6; };
    EXPECT_TRUE(model.checkCollision(at_new_pos));
}

// ─── 10. 边界补全：角点 (max_u, max_v) 确实在缓存中 ─────────────────
// precompute 中专门补全了三种边点，若漏掉会导致 OBB 棱/角漏检。
// OBB {1,1,1}，某面上角点为 (±0.5, ±0.5, ±0.5)，验证这些点存在。
TEST(OBBCollisionModelTest, BorderCompletionCornerPointsPresent) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // 精确命中 OBB 某个角点 (0.5, 0.5, 0.5)
    auto corner_rule = [](const Position3D& p) {
        return (p - Vector3D(0.5, 0.5, 0.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(corner_rule))
        << "Corner point (0.5,0.5,0.5) must be in cached_points_";

    // 精确命中另一角点 (-0.5, 0.5, -0.5)
    auto corner_rule2 = [](const Position3D& p) {
        return (p - Vector3D(-0.5, 0.5, -0.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(corner_rule2))
        << "Corner point (-0.5,0.5,-0.5) must be in cached_points_";
}

// ─── 11. 非对称 OBB：三轴完全不同 ───────────────────────────────────
// 使用 {0.5, 2.0, 3.0}，验证各轴表面范围均正确。
TEST(OBBCollisionModelTest, AsymmetricOBBSurfaceBounds) {
    // half: x=0.25, y=1.0, z=1.5
    OBBCollisionModel model({0.5, 2.0, 3.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // X 面：x = ±0.25，OBB 内 y∈[-1,1], z∈[-1.5,1.5]
    auto x_face = [](const Position3D& p) { return std::abs(std::abs(p.x()) - 0.25) < 1e-9; };
    EXPECT_TRUE(model.checkCollision(x_face));

    // X 方向不应有超出 ±0.25 的采样点
    auto x_out = [](const Position3D& p) { return std::abs(p.x()) > 0.26; };
    EXPECT_FALSE(model.checkCollision(x_out));

    // Y 面：y = ±1.0
    auto y_face = [](const Position3D& p) { return std::abs(std::abs(p.y()) - 1.0) < 1e-9; };
    EXPECT_TRUE(model.checkCollision(y_face));

    // Z 面角点：(±0.25, ±1.0, ±1.5) 必须存在
    auto z_corner = [](const Position3D& p) {
        return (p - Vector3D(0.25, 1.0, 1.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(z_corner));
}

// ─── 12. 旋转 + 平移组合位姿 ─────────────────────────────────────────
// 机器人位姿：绕 Z 旋转 90° 后向 X 方向平移 5m。
// OBB {2,1,1}，旋转后长轴变 Y，中心移到 (5,0,0)。
TEST(OBBCollisionModelTest, CombinedRotationAndTranslation) {
    OBBCollisionModel model({2.0, 1.0, 1.0});

    Pose robot_pose = Pose::Identity();
    robot_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Vector3D::UnitZ()).toRotationMatrix();
    robot_pose.translation() = Vector3D(5.0, 0.0, 0.0);
    model.precompute(robot_pose);

    // OBB 中心在 (5,0,0)，旋转后长轴沿 Y，所以 Y 表面在 y = ±1.0（相对中心）
    // 世界系表面点应在 (5, ±1.0, *) 附近
    auto y_surface_world = [](const Position3D& p) {
        return std::abs(p.x() - 5.0) < 1e-6 && std::abs(std::abs(p.y()) - 1.0) < 1e-6;
    };
    EXPECT_TRUE(model.checkCollision(y_surface_world))
        << "After 90deg rotation + translation, Y-face should appear at world (5, ±1, z)";

    // 原点附近不应有任何采样点
    auto near_origin = [](const Position3D& p) { return p.norm() < 3.5; };
    EXPECT_FALSE(model.checkCollision(near_origin))
        << "OBB translated to x=5 should not have points near origin";
}

// ─── 13. 缓存点数量的确定性：precompute 调用两次结果相同 ─────────────
// 验证 precompute 内部正确调用了 clear()，不会发生累积追加。
// 方法：用计数 lambda 统计遍历点数，两次 precompute 后点数应相同。
TEST(OBBCollisionModelTest, CacheCountIsDeterministic) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 第一次 precompute
    model.precompute(pose);
    int count1 = 0;
    model.checkCollision([&count1](const Position3D&) { ++count1; return false; });

    // 第二次 precompute（相同位姿）
    model.precompute(pose);
    int count2 = 0;
    model.checkCollision([&count2](const Position3D&) { ++count2; return false; });

    EXPECT_EQ(count1, count2)
        << "precompute called twice should produce same number of cached points, "
        << "got " << count1 << " vs " << count2;
    EXPECT_GT(count1, 0) << "cached_points_ should not be empty after precompute";
}

// ═══════════════════════════════════════════════════════════════════════════
// checkCost 测试
// ═══════════════════════════════════════════════════════════════════════════

// ─── 14. 未调用 precompute 直接 checkCost 抛出异常 ──────────────────────
TEST(OBBCollisionModelTest, CheckCostBeforePrecomputeThrows) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    auto cost_rule = [](const Position3D&) -> int { return 10; };
    EXPECT_THROW(model.checkCost(cost_rule), std::logic_error);
}

// ─── 15. 所有点代价相同时返回该代价 ──────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostUniformCost) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    auto uniform_cost = [](const Position3D&) -> int { return 42; };
    int max_cost = model.checkCost(pose, uniform_cost);
    EXPECT_EQ(max_cost, 42);
}

// ─── 16. 返回最大代价值 ──────────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostReturnsMaximum) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 根据 z 坐标计算代价，z 越大代价越高
    auto z_based_cost = [](const Position3D& pt) -> int {
        return static_cast<int>(pt.z() * 100);
    };

    int max_cost = model.checkCost(pose, z_based_cost);
    // OBB {1,1,1} 的 z 范围是 [-0.5, 0.5]，最大代价应该是 50
    EXPECT_EQ(max_cost, 50);
}

// ─── 17. 处理负代价值 ────────────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostHandlesNegativeValues) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 所有点返回负代价
    auto negative_cost = [](const Position3D&) -> int { return -10; };
    int max_cost = model.checkCost(pose, negative_cost);
    EXPECT_EQ(max_cost, 0) << "With all negative costs, should return 0 (initial value)";
}

// ─── 18. 高频接口：precompute 后多次 checkCost ───────────────────────────
TEST(OBBCollisionModelTest, CheckCostHighFreqInterfaceReusesCache) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // 多次调用不同的代价规则
    auto cost1 = [](const Position3D& pt) -> int { return static_cast<int>(pt.x() * 100); };
    auto cost2 = [](const Position3D& pt) -> int { return static_cast<int>(pt.y() * 100); };

    int max1 = model.checkCost(cost1);
    int max2 = model.checkCost(cost2);

    EXPECT_EQ(max1, 50);  // x 最大值 0.5
    EXPECT_EQ(max2, 50);  // y 最大值 0.5
}

// ─── 19. 基于位置的代价计算 ──────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostPositionBased) {
    OBBCollisionModel model({2.0, 2.0, 2.0});
    Pose pose = Pose::Identity();

    // 距离原点越远代价越高
    auto distance_cost = [](const Position3D& pt) -> int {
        return static_cast<int>(pt.norm() * 100);
    };

    int max_cost = model.checkCost(pose, distance_cost);
    // OBB {2,2,2} 的角点距离原点最远，约为 sqrt(3) ≈ 1.732
    EXPECT_GE(max_cost, 170);
    EXPECT_LE(max_cost, 174);
}

// ─── 20. 移动后代价计算更新 ──────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostUpdatesWithPose) {
    OBBCollisionModel model({1.0, 1.0, 1.0});

    // 位姿1：OBB 在原点
    Pose pose1 = Pose::Identity();
    auto x_cost = [](const Position3D& pt) -> int { return static_cast<int>(pt.x() * 100); };
    int cost1 = model.checkCost(pose1, x_cost);
    EXPECT_EQ(cost1, 50);  // x 最大值 0.5

    // 位姿2：机器人平移到 (10, 0, 0)
    Pose pose2 = Pose::Identity();
    pose2.translation() = Vector3D(10.0, 0.0, 0.0);
    int cost2 = model.checkCost(pose2, x_cost);
    EXPECT_EQ(cost2, 1050);  // x 最大值 10.5
}

// ─── 21. 捕获外部变量的代价规则 ──────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostWithCapturedVariables) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    double scale_factor = 200.0;
    auto scaled_cost = [scale_factor](const Position3D& pt) -> int {
        return static_cast<int>(pt.z() * scale_factor);
    };

    int max_cost = model.checkCost(pose, scaled_cost);
    EXPECT_EQ(max_cost, 100);  // z 最大值 0.5 * 200
}

// ─── 22. 零代价情况 ──────────────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostAllZero) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    auto zero_cost = [](const Position3D&) -> int { return 0; };
    int max_cost = model.checkCost(pose, zero_cost);
    EXPECT_EQ(max_cost, 0);
}

// ─── 23. 混合正负代价值 ──────────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostMixedPositiveNegative) {
    OBBCollisionModel model({2.0, 2.0, 2.0});
    Pose pose = Pose::Identity();

    // x < 0 返回负值，x >= 0 返回正值
    auto mixed_cost = [](const Position3D& pt) -> int {
        return static_cast<int>(pt.x() * 100);
    };

    int max_cost = model.checkCost(pose, mixed_cost);
    EXPECT_EQ(max_cost, 100);  // x 最大值 1.0
}

// ═══════════════════════════════════════════════════════════════════════════
// 非对称 OBB 构造函数测试 (min, max)
// ═══════════════════════════════════════════════════════════════════════════

// ─── 24. 非对称构造：外径等于 (max-min) 的范数 ────────────────────────────
TEST(OBBCollisionModelTest, AsymmetricConstructorOuterDiameter) {
    // min=(-1, -2, 0), max=(3, 1, 0) → 对角线 = (4, 3, 0), 范数=5
    OBBCollisionModel model({-1.0, -2.0, 0.0}, {3.0, 1.0, 0.0});
    EXPECT_DOUBLE_EQ(model.getOuterDiameter(), 5.0);
}

// ─── 25. 非对称 OBB 表面采样点覆盖非对称范围 ─────────────────────────────
TEST(OBBCollisionModelTest, AsymmetricBoundsSurfaceCoverage) {
    // min=(-0.2, -0.5, 0), max=(1.0, 0.5, 0)  →  模拟差速机器人：前 1.0m,
    // 后 0.2m, 左右各 0.5m
    OBBCollisionModel model({-0.2, -0.5, 0.0}, {1.0, 0.5, 0.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // X 面：前方 x=1.0，后方 x=-0.2
    auto front_face = [](const Position3D& p) {
        return std::abs(p.x() - 1.0) < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(front_face))
        << "Front face at x=1.0 should have sampling points";

    auto back_face = [](const Position3D& p) {
        return std::abs(p.x() + 0.2) < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(back_face))
        << "Back face at x=-0.2 should have sampling points";

    // X 方向不应有超出范围的采样点
    auto x_out = [](const Position3D& p) {
        return p.x() > 1.01 || p.x() < -0.21;
    };
    EXPECT_FALSE(model.checkCollision(x_out))
        << "No points should exist outside [−0.2, 1.0] in X";
}

// ─── 26. 非对称 OBB 与对称 OBB 构造的等价性验证 ──────────────────────────
TEST(OBBCollisionModelTest, AsymmetricAndSymmetricEquivalence) {
    // 对称 size={2.0, 3.0, 0.0} 等价于 min={-1,-1.5,0}, max={1,1.5,0}
    OBBCollisionModel sym({2.0, 3.0, 0.0});
    OBBCollisionModel asym({-1.0, -1.5, 0.0}, {1.0, 1.5, 0.0});

    EXPECT_DOUBLE_EQ(sym.getOuterDiameter(), asym.getOuterDiameter());

    Pose pose = Pose::Identity();
    sym.precompute(pose);
    asym.precompute(pose);

    // 两个模型对同样规则应返回相同结果
    auto corner_rule = [](const Position3D& p) {
        return (p - Vector3D(1.0, 1.5, 0.0)).norm() < 1e-9;
    };
    EXPECT_EQ(sym.checkCollision(corner_rule), asym.checkCollision(corner_rule));
}

// ─── 27. 非法边界：max < min 抛出异常 ─────────────────────────────────────
TEST(OBBCollisionModelTest, AsymmetricInvalidBoundsThrows) {
    // max.x < min.x
    EXPECT_THROW(OBBCollisionModel({2.0, 0.0, 0.0}, {1.0, 1.0, 1.0}),
                 std::invalid_argument);
    // max.y < min.y
    EXPECT_THROW(OBBCollisionModel({0.0, 2.0, 0.0}, {1.0, 1.0, 1.0}),
                 std::invalid_argument);
    // max.z < min.z
    EXPECT_THROW(OBBCollisionModel({0.0, 0.0, 2.0}, {1.0, 1.0, 1.0}),
                 std::invalid_argument);
}

// ─── 28. 非对称 OBB 带 local_pose 偏移 ────────────────────────────────────
TEST(OBBCollisionModelTest, AsymmetricWithLocalPose) {
    // 差速机器人：OBB 前 1.5m 后 0.3m，OBB 自身在 body 系下向前偏移 0.6m
    // 使 OBB 中心更靠近前方
    Pose local_pose = Pose::Identity();
    local_pose.translation() = Vector3D(0.6, 0.0, 0.0);

    OBBCollisionModel model({-0.3, -0.5, 0.0}, {1.5, 0.5, 0.0}, local_pose);
    Pose robot_pose = Pose::Identity();
    model.precompute(robot_pose);

    // OBB min/max = (-0.3, 1.5) in OBB frame
    // OBB 原点在 body 系 (0.6, 0, 0)
    // world 系: min = -0.3 + 0.6 = 0.3, max = 1.5 + 0.6 = 2.1
    auto front_world = [](const Position3D& p) {
        return std::abs(p.x() - 2.1) < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(front_world))
        << "Front face should be at x=2.1 in world frame";

    auto back_world = [](const Position3D& p) {
        return std::abs(p.x() - 0.3) < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(back_world))
        << "Back face should be at x=0.3 in world frame";
}

// ─── 29. 非对称 OBB 代价检测 ──────────────────────────────────────────────
TEST(OBBCollisionModelTest, AsymmetricCheckCost) {
    // 前 2m 后 0.5m，左右各 0.5m
    OBBCollisionModel model({-0.5, -0.5, 0.0}, {2.0, 0.5, 0.0});
    Pose pose = Pose::Identity();

    auto x_cost = [](const Position3D& pt) -> int {
        return static_cast<int>(pt.x() * 100);
    };

    int max_cost = model.checkCost(pose, x_cost);
    EXPECT_EQ(max_cost, 200);  // x 最大值 2.0 * 100 = 200
}
