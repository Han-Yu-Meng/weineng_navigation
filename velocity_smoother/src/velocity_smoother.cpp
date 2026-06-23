// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "velocity_smoother/velocity_smoother.hpp"

namespace velocity_smoother {

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions& options)
    : Node("velocity_smoother", options) {
    declare_parameters();

    // Zero-initialize current velocity
    current_velocity_.linear.x  = 0.0;
    current_velocity_.linear.y  = 0.0;
    current_velocity_.linear.z  = 0.0;
    current_velocity_.angular.x = 0.0;
    current_velocity_.angular.y = 0.0;
    current_velocity_.angular.z = 0.0;

    // Subscriber to raw cmd_vel
    raw_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_raw", 10,
        std::bind(&VelocitySmoother::cmdVelRawCallback, this, std::placeholders::_1));

    // Publisher for smoothed cmd_vel
    smoothed_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Control loop timer
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = create_wall_timer(period, std::bind(&VelocitySmoother::timerCallback, this));

    RCLCPP_INFO(get_logger(),
        "VelocitySmoother started. control_rate=%.1f Hz, "
        "max_vel=[%.2f, %.2f, %.2f], max_accel=[%.2f, %.2f, %.2f], "
        "timeout=%.1f s",
        control_rate_,
        max_velocity_[0], max_velocity_[1], max_velocity_[2],
        max_accel_[0], max_accel_[1], max_accel_[2],
        velocity_timeout_);
}

void VelocitySmoother::declare_parameters() {
    // Velocity limits (x, y, yaw)
    declare_parameter("max_velocity", std::vector<double>{2.0, 0.0, 0.5});
    declare_parameter("min_velocity", std::vector<double>{-0.50, 0.0, -0.8});
    declare_parameter("deadband_velocity", std::vector<double>{0.0, 0.0, 0.0});

    // Timeout — stop publishing if no command received within this duration
    declare_parameter("velocity_timeout", 1.0);

    // Acceleration / deceleration limits (x, y, yaw)
    declare_parameter("max_accel", std::vector<double>{0.4, 0.0, 0.4});
    declare_parameter("max_decel", std::vector<double>{-20.0, 0.0, -2.0});

    // Internal control loop frequency
    declare_parameter("control_rate", 20.0);

    // Re-read parameters (support live tuning via `ros2 param set`)
    auto cb = [this](const std::vector<rclcpp::Parameter>&) {
        max_velocity_       = get_parameter("max_velocity").as_double_array();
        min_velocity_       = get_parameter("min_velocity").as_double_array();
        deadband_velocity_  = get_parameter("deadband_velocity").as_double_array();
        velocity_timeout_   = get_parameter("velocity_timeout").as_double();
        max_accel_          = get_parameter("max_accel").as_double_array();
        max_decel_          = get_parameter("max_decel").as_double_array();
        control_rate_       = get_parameter("control_rate").as_double();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    };

    // Initial load
    max_velocity_       = get_parameter("max_velocity").as_double_array();
    min_velocity_       = get_parameter("min_velocity").as_double_array();
    deadband_velocity_  = get_parameter("deadband_velocity").as_double_array();
    velocity_timeout_   = get_parameter("velocity_timeout").as_double();
    max_accel_          = get_parameter("max_accel").as_double_array();
    max_decel_          = get_parameter("max_decel").as_double_array();
    control_rate_       = get_parameter("control_rate").as_double();

    // Register callback for live parameter updates
    [[maybe_unused]] auto cb_handle = add_on_set_parameters_callback(cb);
}

void VelocitySmoother::cmdVelRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_raw_cmd_ = *msg;
    last_raw_cmd_time_ = now();
    command_received_ = true;
}

void VelocitySmoother::timerCallback() {
    // ── Read latest target under lock ──────────────────────────────────────
    geometry_msgs::msg::Twist target;
    bool has_command;
    rclcpp::Time cmd_time;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        target = last_raw_cmd_;
        has_command = command_received_;
        cmd_time = last_raw_cmd_time_;
    }

    // ── Timeout: if no command received or last command is too old, decelerate to zero
    if (!has_command || (now() - cmd_time).seconds() > velocity_timeout_) {
        target.linear.x  = 0.0;
        target.linear.y  = 0.0;
        target.linear.z  = 0.0;
        target.angular.x = 0.0;
        target.angular.y = 0.0;
        target.angular.z = 0.0;
    }

    const double dt = 1.0 / control_rate_;

    // ── Apply limits per axis ──────────────────────────────────────────────
    // linear.x (index 0) — forward velocity
    current_velocity_.linear.x = applyAccelLimit(
        current_velocity_.linear.x, target.linear.x,
        max_accel_[0], max_decel_[0], dt);
    current_velocity_.linear.x = applyVelocityBounds(
        current_velocity_.linear.x, min_velocity_[0], max_velocity_[0]);
    current_velocity_.linear.x = applyDeadband(
        current_velocity_.linear.x, deadband_velocity_[0]);

    // linear.y (index 1) — lateral velocity (typically 0 for wheelchairs)
    current_velocity_.linear.y = applyAccelLimit(
        current_velocity_.linear.y, target.linear.y,
        max_accel_[1], max_decel_[1], dt);
    current_velocity_.linear.y = applyVelocityBounds(
        current_velocity_.linear.y, min_velocity_[1], max_velocity_[1]);
    current_velocity_.linear.y = applyDeadband(
        current_velocity_.linear.y, deadband_velocity_[1]);

    // angular.z (index 2) — yaw rate
    current_velocity_.angular.z = applyAccelLimit(
        current_velocity_.angular.z, target.angular.z,
        max_accel_[2], max_decel_[2], dt);
    current_velocity_.angular.z = applyVelocityBounds(
        current_velocity_.angular.z, min_velocity_[2], max_velocity_[2]);
    current_velocity_.angular.z = applyDeadband(
        current_velocity_.angular.z, deadband_velocity_[2]);

    // Zero out unused axes
    current_velocity_.linear.z  = 0.0;
    current_velocity_.angular.x = 0.0;
    current_velocity_.angular.y = 0.0;

    // ── Publish smoothed command ───────────────────────────────────────────
    smoothed_cmd_vel_pub_->publish(current_velocity_);
}

double VelocitySmoother::applyAccelLimit(
    double current, double target, double accel, double decel, double dt) const {
    if (accel <= 0.0 || decel >= 0.0) {
        // If accel/decel are not valid, snap to target
        return target;
    }

    const double dv = target - current;

    if (dv > 0.0) {
        // Accelerating
        const double step = accel * dt;
        return current + std::min(dv, step);
    } else if (dv < 0.0) {
        // Decelerating — decel is negative, e.g. -2.0
        const double step = std::fabs(decel) * dt;
        return current + std::max(dv, -step);
    } else {
        return current;
    }
}

double VelocitySmoother::applyVelocityBounds(
    double value, double min_val, double max_val) const {
    return std::clamp(value, min_val, max_val);
}

double VelocitySmoother::applyDeadband(double value, double deadband) const {
    if (deadband <= 0.0) {
        return value;
    }
    if (std::fabs(value) < deadband) {
        return 0.0;
    }
    return value;
}

}  // namespace velocity_smoother

// ── Main ──────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<velocity_smoother::VelocitySmoother>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
