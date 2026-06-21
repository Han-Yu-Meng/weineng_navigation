// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <string>
#include <memory>

//#include "finenav_mppi_controller/tools/path_handler.hpp"
//#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"
#include "finenav_mppi_controller/models/constraints.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

#include "rclcpp/rclcpp.hpp"


#include "finenav_interface/test_map_params.hpp"
#include "finenav_interface/test_planner_params.hpp"

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"
#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */


class MPPIController
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
    MPPIController() = default;

    static constexpr finenav::IOProfile InputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };
    static constexpr finenav::IOProfile OutputProfile = {
        .pose = true,
        .vel  = true,
        .accel= false,
        .time = false
    };

    using IMapView = nav2_mppi_controller::IMapView;
    using Context = finenav::PlanningContext<MPPIController>;
    using ConfigType = finenav_mppi_controller::Params;
    void configure(const ConfigType& params) {              //应该传入参数 因测试先去除

        registerCritic(std::make_unique<critics::ConstraintCritic>());
        registerCritic(std::make_unique<critics::CostCritic>());
        registerCritic(std::make_unique<critics::GoalAngleCritic>());
        registerCritic(std::make_unique<critics::GoalCritic>());
        // registerCritic(std::make_unique<critics::ObstaclesCritic>());
        registerCritic(std::make_unique<critics::PathAlignCritic>());
        registerCritic(std::make_unique<critics::PathAlignLegacyCritic>());
        registerCritic(std::make_unique<critics::PathAngleCritic>());
        registerCritic(std::make_unique<critics::PathFollowCritic>());
        registerCritic(std::make_unique<critics::PreferForwardCritic>());
        registerCritic(std::make_unique<critics::TwirlingCritic>());
        registerCritic(std::make_unique<critics::VelocityDeadbandCritic>());

        std::cout<<"[nav2_mppi_controller] Critics registered. Initializing optimizer..."<<std::endl;
        std::cout<<"[nav2_mppi_controller] Number of critics: "<<critics_.size()<<std::endl;

        optimizer_.initialize(
            "MPPIController",
            critics_,
            params
            );
       //TODO:初始化过程需要完善
    }

     std::optional<finenav::DataPacket<OutputProfile>>plan(const finenav::PlanningContext<MPPIController>& ctx, const IMapView& map_view) {
        using Clock = std::chrono::steady_clock;
        using Ms    = std::chrono::duration<double, std::milli>;
        const auto t_plan_start = Clock::now();

        finenav::DataPacket<OutputProfile> traj;
        // 从框架注入的 RobotState 中取出 pose 和 twist，无需 TF 查询
        const auto& robot_state = ctx.robot_state;
        auto goal_pose = ctx.goal_pose;

        nav_msgs::msg::Path path = convertDataPacketToPath(ctx.ref_traj, "map");
        this->setPlan(path);

        geometry_msgs::msg::PoseStamped start_pose_stamped;
        start_pose_stamped.pose = robot_state.pose;
        start_pose_stamped.header.stamp = rclcpp::Clock().now();
        start_pose_stamped.header.frame_id = "map";

        const auto t_optimize_start
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        = Clock::now();
        computeVelocityCommands(
            start_pose_stamped,
            robot_state.twist,  // 直接使用当前实际速度
            map_view
            );
        const double ms_optimize = Ms(Clock::now() - t_optimize_start).count();

        // 装填输出轨迹

    // 获取最优轨迹
    auto optimized_trajectory = optimizer_.getOptimizedTrajectory();
    auto control_seq = optimizer_.getControlSequence();
    // 填充轨迹点 (pose + twist)
    const size_t n = optimized_trajectory.shape(0);
    traj.poses.reserve(n);
    traj.twists.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        // pose: x, y, yaw -> quaternion
        geometry_msgs::msg::Pose pose;
        pose.position.x = optimized_trajectory(i, 0);
        pose.position.y = optimized_trajectory(i, 1);
        pose.position.z = 0.0;
        const float yaw = optimized_trajectory(i, 2);
        pose.orientation.z = std::sin(yaw * 0.5f);
        pose.orientation.w = std::cos(yaw * 0.5f);
        traj.poses.push_back(pose);

        // twist: vx, vy, wz from control sequence
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = control_seq(i, 0);
        twist.angular.z = control_seq(i, 1);
        twist.linear.y  = control_seq(i, 2);
        traj.twists.push_back(twist);
    }

    const double ms_total = Ms(Clock::now() - t_plan_start).count();
    {
        static rclcpp::Clock throttle_clk{RCL_STEADY_TIME};
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("MPPIController"), throttle_clk, 2000,
            "\n[MPPIController] plan() timing\n"
            "  evalControl (optimize) : %7.2f ms\n"
            "  total plan()           : %7.2f ms",
            ms_optimize, ms_total);
    }

    return traj;
}


  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage);

  /**
   * @brief Reset optimizer state at Episode boundary (new goal).
   *        Called by ControlLayer whenever a new Action Goal arrives.
   *        Clears control_sequence_ so the next plan() call starts from
   *        a zero warm-start, independent of any previous planning direction.
   */
  void reset() {
      optimizer_.reset();
  }

    void registerCritic (std::unique_ptr<critics::CriticFunction> critic) {
        critics_.push_back(std::move(critic));        //TODO:在初始化的时候要换成注册
    }


    template <typename DataPacketT>
    nav_msgs::msg::Path convertDataPacketToPath(
        const DataPacketT & data,
        const std::string & frame_id)
    {
        nav_msgs::msg::Path path;

        path.header.frame_id = frame_id;
        path.header.stamp = rclcpp::Clock().now();

        // ⭐ 关键：只有当有 poses 才能用
        if constexpr (requires { data.poses; }) {

            path.poses.reserve(data.poses.size());

            for (const auto & pose : data.poses) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path.header;
                ps.pose = pose;

                path.poses.push_back(ps);
            }
        }

        return path;
    }


protected:
    /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
//    void visualize(
//        nav_msgs::msg::Path transformed_plan,
//        const builtin_interfaces::msg::Time & cmd_stamp,
//        double z_height);

    /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
	// geometry_msgs::msg::TwistStamped computeVelocityCommands(
    void computeVelocityCommands(
    	const geometry_msgs::msg::PoseStamped & robot_pose,
  		const geometry_msgs::msg::Twist & robot_speed,
    	const IMapView& map_view)
	{
	#ifdef BENCHMARK_TESTING
  	auto start = std::chrono::system_clock::now();
	#endif

	//  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
	//    reset();
	//  }
//  	last_time_called_ = clock_->now();

//  	std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
//

  	// geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, path_ ,map_view);

	#ifdef BENCHMARK_TESTING
  	auto end = std::chrono::system_clock::now();
  	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  	RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
	#endif

//  	if (visualize_) {
//    	double z_height = 0.06;
//    	const std::string base_frame = map_view->getBaseFrameID();
//    	try {
//      	auto tfst = tf_buffer_->lookupTransform(
//        	map_view.getGlobalFrameID(), base_frame,  cmd.header.stamp);
//      		z_height = tfst.transform.translation.z;
//    	} catch (const tf2::TransformException & ex) {
//      	RCLCPP_WARN(logger_, "Failed to get transform %s -> %s: %s",
//                  map_view.getGlobalFrameID().c_str(), base_frame.c_str(), ex.what());
//    	}
//    	visualize(std::move(path_, cmd.header.stamp, z_height);
//  	}
  	// return cmd;
}

//  std::string name_;
//  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
//  rclcpp::Clock::SharedPtr clock_;
//  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
//  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path path_;

//  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
//  TrajectoryVisualizer trajectory_visualizer_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  bool visualize_;

  double reset_period_;
  // Last time computeVelocityCommands was called
//  rclcpp::Time last_time_called_;
};

}  // namespace nav2_mppi_controller


FINENAV_REGISTER_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)


#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
