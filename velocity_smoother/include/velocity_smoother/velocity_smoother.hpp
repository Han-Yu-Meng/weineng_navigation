// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace velocity_smoother {

/**
 * @brief Applies acceleration/deceleration limits and velocity bounds to
 *        smooth raw cmd_vel commands.
 *
 * Inspired by nav2_velocity_smoother but simplified for wheelchair use.
 * Subscribes to /cmd_vel_raw and publishes smoothed /cmd_vel.
 */
class VelocitySmoother : public rclcpp::Node {
public:
    VelocitySmoother(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ── Parameter declarations ──────────────────────────────────────────────
    void declare_parameters();

    // ── Callback for incoming raw cmd_vel ───────────────────────────────────
    void cmdVelRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ── Timer callback: applies limits and publishes smoothed cmd_vel ───────
    void timerCallback();

    // ── Helpers ─────────────────────────────────────────────────────────────
    double applyAccelLimit(double current, double target, double accel, double decel, double dt) const;
    double applyVelocityBounds(double value, double min_val, double max_val) const;
    double applyDeadband(double value, double deadband) const;

    // ── Subscriber / Publisher ──────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr smoothed_cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Latest received raw command (protected by mutex) ────────────────────
    std::mutex mutex_;
    geometry_msgs::msg::Twist last_raw_cmd_;
    rclcpp::Time last_raw_cmd_time_;
    bool command_received_{false};

    // ── Current smoothed velocity state ─────────────────────────────────────
    geometry_msgs::msg::Twist current_velocity_;

    // ── Parameters ──────────────────────────────────────────────────────────
    std::vector<double> max_velocity_;
    std::vector<double> min_velocity_;
    std::vector<double> deadband_velocity_;
    double velocity_timeout_;
    std::vector<double> max_accel_;
    std::vector<double> max_decel_;
    double control_rate_;
};

}  // namespace velocity_smoother
