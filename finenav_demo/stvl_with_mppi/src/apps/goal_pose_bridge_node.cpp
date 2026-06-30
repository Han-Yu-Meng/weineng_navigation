// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

#include "finenav_msgs/action/navigate_to_pose.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

namespace {

constexpr uint32_t TRAJ_POSE = 1;

geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(0.5 * yaw);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    return q;
}

} // namespace

class GoalPoseLinePathTestNode : public rclcpp::Node {
public:
    using NavigateToPose        = finenav_msgs::action::NavigateToPose;
    using GoalHandleNavigate    = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalPoseLinePathTestNode()
    : rclcpp::Node("goal_pose_bridge_node")
    {
        constexpr const char* kGoalTopic        = "/goal_pose";
        constexpr const char* kPathTopic        = "/hello_finenav/line_path";
        constexpr const char* kNavStateTopic    = "/nav_state";
        constexpr const char* kMapFrame         = "map";
        constexpr const char* kActionServer     = "/hello_finenav/navigate_to_pose";  // Navigator action server
        constexpr const char* kBehaviorTree     = "NavigateToPose";    // BT id in navigate_to_pose.xml
        constexpr const char* kMppiPlanTopic    = "/hello_finenav/mppi_layer_plan";
        constexpr const char* kNavPathTopic     = "/mppi_path";
        constexpr const char* kLinePathNavTopic = "/line_path_nav";
        constexpr int kNumPoints = 50;

        path_topic_          = kPathTopic;
        map_frame_           = kMapFrame;
        action_server_name_  = kActionServer;
        behavior_tree_       = kBehaviorTree;
        mppi_plan_topic_     = kMppiPlanTopic;
        nav_path_topic_      = kNavPathTopic;
        line_path_nav_topic_ = kLinePathNavTopic;
        num_points_          = kNumPoints;

        // 发布者
        path_pub_          = this->create_publisher<finenav_msgs::msg::Trajectory>(path_topic_, 10);
        nav_path_pub_      = this->create_publisher<nav_msgs::msg::Path>(nav_path_topic_, 10);
        line_path_nav_pub_ = this->create_publisher<nav_msgs::msg::Path>(line_path_nav_topic_, 10);

        // NavigateToPose Action 客户端（指向 Navigator）
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_server_name_);

        // 订阅 nav_state 获取机器人位姿
        nav_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            kNavStateTopic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(pose_mutex_);
                latest_pose_ = msg->pose.pose;
                has_pose_ = true;
            });

        // 订阅 /goal_pose
        goal_sub_raw_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            kGoalTopic, 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->onGoal(msg);
            });

        // 订阅 MPPI 规划结果并可视化
        mppi_plan_sub_ = this->create_subscription<finenav_msgs::msg::Trajectory>(
            mppi_plan_topic_, 10,
            std::bind(&GoalPoseLinePathTestNode::mppiPlanCallback, this, std::placeholders::_1));

        // ── 发布 /navigation_enabled 话题（transient_local），供 BT 中 NavigationControl 订阅
        nav_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/navigation_enabled", rclcpp::QoS(1).transient_local());
        {
            std_msgs::msg::Bool enabled_msg;
            enabled_msg.data = true;
            nav_enabled_pub_->publish(enabled_msg);
        }

        // ── 创建 /stop_navigation 和 /start_navigation 服务 ─────────────────
        // 在 goal_pose_bridge_node 而非 Navigator/BT 中创建，确保从启动起一直可用
        stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/stop_navigation",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                navigation_enabled_.store(false);
                {
                    std_msgs::msg::Bool msg;
                    msg.data = false;
                    nav_enabled_pub_->publish(msg);
                }
                response->success = true;
                response->message = "Navigation stopped.";

                // 取消当前活跃的导航目标（如果存在）
                if (active_goal_handle_) {
                    nav_action_client_->async_cancel_goal(active_goal_handle_);
                    RCLCPP_INFO(this->get_logger(),
                                "[goal_pose_bridge] Navigation stopped, cancelling active goal.");
                }
                RCLCPP_INFO(this->get_logger(), "[goal_pose_bridge] Navigation stopped.");
            });

        start_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/start_navigation",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                navigation_enabled_.store(true);
                {
                    std_msgs::msg::Bool msg;
                    msg.data = true;
                    nav_enabled_pub_->publish(msg);
                }
                response->success = true;
                response->message = "Navigation started.";
                RCLCPP_INFO(this->get_logger(), "[goal_pose_bridge] Navigation started.");
            });

        RCLCPP_INFO(this->get_logger(),
                    "Ready. goal_topic=%s  path_topic=%s  nav_action_server=%s  bt=%s",
                    kGoalTopic, path_topic_.c_str(),
                    action_server_name_.c_str(), behavior_tree_.c_str());
    }

private:
    void mppiPlanCallback(const finenav_msgs::msg::Trajectory::SharedPtr msg)
    {
        if (!(msg->valid_fields & TRAJ_POSE) || msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "MPPI plan has no pose data, skipping");
            return;
        }
        nav_path_pub_->publish(convertTrajectoryToPath(*msg));
    }

    nav_msgs::msg::Path convertTrajectoryToPath(const finenav_msgs::msg::Trajectory& traj)
    {
        nav_msgs::msg::Path path;
        path.header.stamp    = this->now();
        path.header.frame_id = map_frame_;
        path.poses.reserve(traj.poses.size());
        for (const auto& p : traj.poses) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose   = p;
            path.poses.push_back(ps);
        }
        return path;
    }

    void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 检查导航是否启用
        if (!navigation_enabled_.load()) {
            RCLCPP_WARN(this->get_logger(),
                        "Navigation is stopped. Call /start_navigation to enable.");
            return;
        }

        geometry_msgs::msg::Pose start_pose;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            if (!has_pose_) {
                RCLCPP_WARN(this->get_logger(),
                            "No nav_state received yet, cannot build reference path.");
                return;
            }
            start_pose = latest_pose_;
        }

        const double sx = start_pose.position.x;
        const double sy = start_pose.position.y;
        const double sz = start_pose.position.z;
        const double gx = msg->pose.position.x;
        const double gy = msg->pose.position.y;
        const double gz = msg->pose.position.z;

        RCLCPP_INFO(this->get_logger(),
                    "Goal received: start=(%.2f, %.2f) -> goal=(%.2f, %.2f)",
                    sx, sy, gx, gy);

        const double dx = gx - sx, dy = gy - sy, dz = gz - sz;
        if (std::sqrt(dx*dx + dy*dy + dz*dz) < 1e-6) {
            RCLCPP_WARN(this->get_logger(), "Start and goal coincide, skipping.");
            return;
        }

        // 1. 生成直线参考轨迹并发布到 line_path（ControlLayer prePlan 读取）
        auto ref_traj = generateStraightLineTrajectory(sx, sy, sz, gx, gy, gz);
        path_pub_->publish(ref_traj);
        line_path_nav_pub_->publish(convertTrajectoryToPath(ref_traj));
        RCLCPP_INFO(this->get_logger(),
                    "Published reference trajectory: %zu points", ref_traj.poses.size());

        // 2. 向 Navigator 发送 NavigateToPose action goal
        auto nav_goal = *msg;
        nav_goal.pose.position.z = start_pose.position.z;
        sendNavigateGoal(nav_goal);
    }

    finenav_msgs::msg::Trajectory generateStraightLineTrajectory(
        double sx, double sy, double sz,
        double gx, double gy, double gz)
    {
        auto traj = finenav_msgs::msg::Trajectory();
        traj.header.stamp    = this->now();
        traj.header.frame_id = map_frame_;
        traj.valid_fields    = TRAJ_POSE;
        traj.poses.reserve(num_points_);

        const double dx  = gx - sx;
        const double dy  = gy - sy;
        const double dz  = gz - sz;
        const double yaw = std::atan2(dy, dx);
        const auto   q   = quatFromYaw(yaw);

        for (int i = 0; i < num_points_; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(num_points_ - 1);
            geometry_msgs::msg::Pose pose;
            pose.position.x  = sx + t * dx;
            pose.position.y  = sy + t * dy;
            pose.position.z  = sz + t * dz;
            pose.orientation = q;
            traj.poses.push_back(pose);
        }
        return traj;
    }

    void sendNavigateGoal(const geometry_msgs::msg::PoseStamped& goal_pose_stamped)
    {
        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(),
                         "NavigateToPose action server '%s' not available.",
                         action_server_name_.c_str());
            return;
        }

        // 直接发送新目标，SimpleActionServer 的 preempt 机制会处理抢断。
        // 不要先 cancel 旧目标，否则 cancel 与新 goal 同时到达时，
        // terminate_all() 会把 pending 一并 abort，导致新目标被丢弃。
        NavigateToPose::Goal goal_msg;
        goal_msg.pose          = goal_pose_stamped;
        goal_msg.behavior_tree = behavior_tree_;

        RCLCPP_INFO(this->get_logger(),
                    "Sending NavigateToPose goal: (%.2f, %.2f)  bt='%s'",
                    goal_pose_stamped.pose.position.x,
                    goal_pose_stamped.pose.position.y,
                    behavior_tree_.c_str());

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        opts.goal_response_callback =
            [this](const GoalHandleNavigate::SharedPtr& gh) {
                if (!gh) {
                    RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal rejected.");
                    return;
                }
                active_goal_handle_ = gh;
                RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted.");
            };

        opts.result_callback =
            [this](const GoalHandleNavigate::WrappedResult& result) {
                active_goal_handle_ = nullptr;
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(),
                                    "✅ Navigation SUCCEEDED (IsGoalReached triggered).");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(),
                                    "Navigation CANCELED (preempted by new goal or user).");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(),
                                    "⚠️  Navigation ABORTED (plan failed or BT returned FAILURE).");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Navigation: unknown result code.");
                }
            };

        nav_action_client_->async_send_goal(goal_msg, opts);
    }

    // ---------- publishers ----------
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr line_path_nav_pub_;

    // ---------- subscribers ----------
    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr mppi_plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_raw_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_state_sub_;

    // ---------- action ----------
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    GoalHandleNavigate::SharedPtr active_goal_handle_;

    // ---------- services ----------
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;

    // ---------- navigation enabled state ----------
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_enabled_pub_;
    std::atomic<bool> navigation_enabled_{true};

    // ---------- robot pose cache ----------
    std::mutex pose_mutex_;
    geometry_msgs::msg::Pose latest_pose_{};
    bool has_pose_{false};

    // ---------- params ----------
    std::string path_topic_;
    std::string map_frame_;
    std::string action_server_name_;
    std::string behavior_tree_;
    std::string mppi_plan_topic_;
    std::string nav_path_topic_;
    std::string line_path_nav_topic_;
    int num_points_{50};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseLinePathTestNode>());
    rclcpp::shutdown();
    return 0;
}