// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include <cmath>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include "finenav_localizer/filter_model.hpp"
#include "finenav_localizer/finenav_localizer.hpp"

using namespace std::chrono_literals;
namespace fn = finenav;
using Model = fn::NavigationModelD;

// ============================================================================
//  NavigationModel (pure math — no ROS required)
// ============================================================================

TEST(NavigationModel, PredictNominal_ZeroVelocity_StaysAtOrigin)
{
    fn::NavStateD s;  // default: p=0, R=I, v=0, ω=0
    const auto ns = Model::predictNominal(s, 1.0);
    EXPECT_NEAR(ns.p.norm(), 0.0, 1e-12);
    EXPECT_NEAR((ns.R.matrix() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
}

TEST(NavigationModel, PredictNominal_ConstantVelocity_PositionIntegrates)
{
    fn::NavStateD s;
    s.v = {1.0, 2.0, 0.0};
    const auto ns = Model::predictNominal(s, 3.0);
    EXPECT_NEAR(ns.p.x(), 3.0, 1e-12);
    EXPECT_NEAR(ns.p.y(), 6.0, 1e-12);
    EXPECT_NEAR(ns.p.z(), 0.0, 1e-12);
}

TEST(NavigationModel, ObservePose_ZeroInnovationAtNominal)
{
    fn::NavStateD s;
    s.p = {1.0, -2.0, 0.5};
    s.R = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, 1.2});

    const auto obs = Model::observePose(s, s.p, s.R);
    EXPECT_NEAR(obs.innovation.norm(), 0.0, 1e-10);
}

TEST(NavigationModel, ObserveVelocity_ZeroInnovationAtNominal)
{
    fn::NavStateD s;
    s.v     = {1.0, 0.5, 0.0};
    s.omega = {0.0, 0.0, 0.3};

    // body-frame velocity matching the nominal
    const Eigen::Vector3d v_body = s.R.inverse() * s.v;
    const auto obs = Model::observeVelocity(s, v_body, s.omega);
    EXPECT_NEAR(obs.innovation.norm(), 0.0, 1e-9);
}

// ============================================================================
//  FineNavLocalizer integration (uses real wall-clock timer; stays < 500 ms)
// ============================================================================

class LocalizerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        node_ = rclcpp::Node::make_shared("localizer_unit_test");

        rclcpp::NodeOptions pnode_opts;
        pnode_opts.append_parameter_override("update_rate_hz",       100.0);
        pnode_opts.append_parameter_override("pose_smoothing_steps", int64_t{1});
        pnode_opts.append_parameter_override("pose_gate_dist",       20.0);
        pnode_opts.append_parameter_override("model.vel_noise_std",   0.1);
        pnode_opts.append_parameter_override("model.omega_noise_std", 0.1);

        localizer_ = std::make_unique<fn::FineNavLocalizer>(node_, pnode_opts);
    }

    /// Build a minimal PoseMsg at (x, y, yaw) with diagonal σ² covariance.
    static fn::FineNavLocalizer::PoseMsg makePose(
        double x, double y, double yaw, double pos_sigma,
        rclcpp::Time stamp)
    {
        fn::FineNavLocalizer::PoseMsg m;
        m.header.stamp    = stamp;
        m.header.frame_id = "map";
        m.pose.pose.position.x  = x;
        m.pose.pose.position.y  = y;
        m.pose.pose.position.z  = 0.0;
        m.pose.pose.orientation.w = std::cos(yaw * 0.5);
        m.pose.pose.orientation.x = 0.0;
        m.pose.pose.orientation.y = 0.0;
        m.pose.pose.orientation.z = std::sin(yaw * 0.5);
        m.pose.covariance.fill(0.0);
        const double var = pos_sigma * pos_sigma;
        m.pose.covariance[0]  = var;
        m.pose.covariance[7]  = var;
        m.pose.covariance[14] = 1e-4;
        m.pose.covariance[21] = 1e-4;
        m.pose.covariance[28] = 1e-4;
        m.pose.covariance[35] = var;
        return m;
    }

    rclcpp::Node::SharedPtr               node_;
    std::unique_ptr<fn::FineNavLocalizer> localizer_;
};

TEST_F(LocalizerTest, SetInitialPose_StateMatchesInputImmediately)
{
    constexpr double x = 3.0, y = -1.5, yaw = 0.8;
    localizer_->setInitialPose(makePose(x, y, yaw, 0.1, node_->now()));

    const fn::NavStateD s = localizer_->getState();
    EXPECT_NEAR(s.p.x(), x,   1e-9);
    EXPECT_NEAR(s.p.y(), y,   1e-9);

    const Eigen::Quaterniond q(s.R.matrix());
    const double yaw_out = std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    EXPECT_NEAR(yaw_out, yaw, 1e-9);
}

TEST_F(LocalizerTest, FeedPoseObservation_StateMovesTowardObservation)
{
    // Initialise at origin with large uncertainty so the gate passes
    localizer_->setInitialPose(makePose(0.0, 0.0, 0.0, 1.0, node_->now()));
    rclcpp::sleep_for(20ms);

    // Feed a pose observation 1 m away
    constexpr double obs_x = 1.0, obs_y = 0.0;
    localizer_->feedObservation(makePose(obs_x, obs_y, 0.0, 0.1, node_->now()));

    rclcpp::sleep_for(150ms);  // let several timer ticks run

    const fn::NavStateD s = localizer_->getState();
    const double dist_init  = std::hypot(obs_x, obs_y);      // distance from 0,0 to obs
    const double dist_after = std::hypot(s.p.x() - obs_x, s.p.y() - obs_y);
    EXPECT_LT(dist_after, dist_init);  // state closer to obs than initial (0,0)
}

// ── Test entry point ──────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

