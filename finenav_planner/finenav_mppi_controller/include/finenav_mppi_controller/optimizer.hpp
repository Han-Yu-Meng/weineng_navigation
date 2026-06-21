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

#ifndef NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
#define NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_

#pragma once

#include <string>
#include <memory>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "finenav_mppi_controller/models/optimizer_settings.hpp"
#include "finenav_mppi_controller/motion_models.hpp"
#include "finenav_mppi_controller/critic_manager.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"
#include "finenav_mppi_controller/models/path.hpp"
#include "finenav_mppi_controller/tools/noise_generator.hpp"
//#include "finenav_mppi_controller/tools/parameters_handler.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"



namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;
/**
 * @class mppi::Optimizer
 * @brief Main algorithm optimizer of the MPPI Controller
 */


class Optimizer
{
public:
  /**
    * @brief Constructor for mppi::Optimizer
    */
  	Optimizer() = default;

  /**
   * @brief Destructor for mppi::Optimizer
   */
  	~Optimizer() {shutdown();}


  /**
   * @brief Initializes optimizer on startup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
    void initialize(
        const std::string & name,
        std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
        const finenav_mppi_controller::Params & params)
  	{
  	    name_ = name;
  	    critics_ = std::move(critics);

  	    getParams(params);

  		critic_manager_.on_configure(critics_, params);
  	    noise_generator_.initialize(settings_, isHolonomic());

  	    reset();

  		std::cout<<"[nav2_mppi_controller] Optimizing initialized"<<std::endl;
  	}

  /**
   * @brief Shutdown for optimizer at process end
   */
    void shutdown()
  	{
  	    noise_generator_.shutdown();
  	}

    void getParams(const finenav_mppi_controller::Params & p)
  	{
  	    auto & s = settings_;

  	    s.model_dt        = static_cast<float>(p.model_dt);
  	    s.time_steps      = static_cast<unsigned int>(p.time_steps);
  	    s.batch_size      = static_cast<unsigned int>(p.batch_size);
  	    s.iteration_count = static_cast<unsigned int>(p.iteration_count);
  	    s.temperature     = static_cast<float>(p.temperature);
  	    s.gamma           = static_cast<float>(p.gamma);

  	    s.base_constraints.vx_max = p.vx_max;
  	    s.base_constraints.vx_min = p.vx_min;
  	    s.base_constraints.vy     = p.vy_max;
  	    s.base_constraints.wz     = p.wz_max;

  	    s.sampling_std.vx = p.vx_std;
  	    s.sampling_std.vy = p.vy_std;
  	    s.sampling_std.wz = p.wz_std;

  	    s.retry_attempt_limit = 1;

  	    s.constraints = s.base_constraints;
  	    setMotionModel(p.motion_model);

  		std::cout<<"[nav2_mppi_controller] Optimizing param prepared"<<std::endl;
  	    double controller_frequency = 1.0 / static_cast<double>(s.model_dt);
  	    setOffset(controller_frequency);
  	}

  /**
   * @brief Compute control using MPPI algorithm
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   * @return TwistStamped of the MPPI control
   */
	// geometry_msgs::msg::TwistStamped evalControl(
	void evalControl(
    	const geometry_msgs::msg::PoseStamped & robot_pose,
    	const geometry_msgs::msg::Twist & robot_speed,
    	const nav_msgs::msg::Path & plan,
    	const nav2_mppi_controller::IMapView& map_view)
    {
    prepare(robot_pose, robot_speed, plan);


    do {
      optimize(map_view);
    } while (fallback(critics_data_.fail_flag));
    // utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
    auto control = getControlFromSequenceAsTwist(plan.header.stamp, map_view);
    if (settings_.shift_control_sequence) {
      shiftControlSequence();
    }
    // return control;
}


  /**
   * @brief Get the trajectories generated in a cycle for visualization
   * @return Set of trajectories evaluated in cycle
   */
    models::Trajectories & getGeneratedTrajectories()
  	{
  	    return generated_trajectories_;
  	}

  /**
   * @brief Get the optimal trajectory for a cycle for visualization
   * @return Optimal trajectory
   */
    /**
     * @brief Get the optimal control sequence (vx, vy, wz) for each time step
     * @return xtensor of shape [time_steps, 3] with columns (vx, wz, vy) — or (vx, wz, 0) for DiffDrive
     */
    xt::xtensor<float, 2> getControlSequence() const
    {
        auto && seq = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3u});
        xt::noalias(xt::view(seq, xt::all(), 0)) = control_sequence_.vx;
        xt::noalias(xt::view(seq, xt::all(), 1)) = control_sequence_.wz;
        if (isHolonomic()) {
            xt::noalias(xt::view(seq, xt::all(), 2)) = control_sequence_.vy;
        } else {
            xt::noalias(xt::view(seq, xt::all(), 2)) = xt::zeros<float>({settings_.time_steps});
        }
        return seq;
    }

    xt::xtensor<float, 2>getOptimizedTrajectory()
    {
        auto && sequence =
          xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
        auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

        xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
        xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

        if (isHolonomic()) {
            xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
        }

        integrateStateVelocities(trajectories, sequence);
        return std::move(trajectories);
    }


  /**
   * @brief Set the maximum speed based on the speed limits callback
   * @param speed_limit Limit of the speed for use
   * @param percentage Whether the speed limit is absolute or relative
   */
    void setSpeedLimit(double speed_limit, bool percentage)
    {
        auto & s = settings_;

        // 使用局部哨兵值表示 "no speed limit"
        // 说明：nav2_costmap_2d::NO_SPEED_LIMIT 在 nav2 的实现中是用于表示“无速度限制”的常量。
        // 为了避免直接依赖外部宏/命名空间，这里使用本地常量 NO_SPEED_LIMIT_LOCAL 作为替代。
        // 可替代策略还包括使用负值（例如 -1）或 std::optional<double> 来表示未设置限制。
        constexpr double NO_SPEED_LIMIT_LOCAL = -1;

        if (speed_limit == NO_SPEED_LIMIT_LOCAL) {
            s.constraints.vx_max = s.base_constraints.vx_max;
            s.constraints.vx_min = s.base_constraints.vx_min;
            s.constraints.vy = s.base_constraints.vy;
            s.constraints.wz = s.base_constraints.wz;
        } else {
            if (percentage) {
                // Speed limit is expressed in % from maximum speed of robot
                double ratio = speed_limit / 100.0;
                s.constraints.vx_max = s.base_constraints.vx_max * ratio;
                s.constraints.vx_min = s.base_constraints.vx_min * ratio;
                s.constraints.vy = s.base_constraints.vy * ratio;
                s.constraints.wz = s.base_constraints.wz * ratio;
            } else {
                // Speed limit is expressed in absolute value
                double ratio = speed_limit / s.base_constraints.vx_max;
                s.constraints.vx_max = s.base_constraints.vx_max * ratio;
                s.constraints.vx_min = s.base_constraints.vx_min * ratio;
                s.constraints.vy = s.base_constraints.vy * ratio;
                s.constraints.wz = s.base_constraints.wz * ratio;
            }
        }
    }


  /**
   * @brief Reset the optimization problem to initial conditions
   */
    void reset()
    {
        state_.reset(settings_.batch_size, settings_.time_steps);
        control_sequence_.reset(settings_.time_steps);
        control_history_[0] = {0.0, 0.0, 0.0};
        control_history_[1] = {0.0, 0.0, 0.0};
        control_history_[2] = {0.0, 0.0, 0.0};
        control_history_[3] = {0.0, 0.0, 0.0};

        settings_.constraints = settings_.base_constraints;

        costs_ = xt::zeros<float>({settings_.batch_size});
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

        noise_generator_.reset(settings_, isHolonomic());
        RCLCPP_INFO(logger_, "Optimizer reset");
    }


protected:
  /**
   * @brief Main function to generate, score, and return trajectories
   */

	void optimize(const nav2_mppi_controller::IMapView& map_view)
	{
  	for (size_t i = 0; i < settings_.iteration_count; ++i) {
    	generateNoisedTrajectories();
    	critic_manager_.evalTrajectoriesScores(critics_data_,map_view);
    	updateControlSequence();
 		}
	}
  /**
   * @brief Prepare state information on new request for trajectory rollouts
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   */
    void prepare(
        const geometry_msgs::msg::PoseStamped & robot_pose,
        const geometry_msgs::msg::Twist & robot_speed,
        const nav_msgs::msg::Path & plan)
	{
	    state_.pose = robot_pose;
	    state_.speed = robot_speed;
	    path_ = utils::toTensor(plan);
	    costs_.fill(0);

	    critics_data_.fail_flag = false;
	    critics_data_.motion_model = motion_model_;
	    critics_data_.furthest_reached_path_point.reset();
	    critics_data_.path_pts_valid.reset();
	}



  /**
   * @brief Set the motion model of the vehicle platform
   * @param model Model string to use
   */
    void setMotionModel(const std::string & model)
    {
        if (model == "DiffDrive") {
            motion_model_ = std::make_shared<DiffDriveMotionModel>();
        } else if (model == "Omni") {
            motion_model_ = std::make_shared<OmniMotionModel>();
        } else if (model == "Ackermann") {
            motion_model_ = std::make_shared<AckermannMotionModel>(name_);
        } else {
            throw std::runtime_error(
                    std::string(
                      "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
                      "or Ackermann"));
        }
    }

  /**
   * @brief Shift the optimal control sequence after processing for
   * next iterations initial conditions after execution
   */
    void shiftControlSequence()
    {
        using namespace xt::placeholders;  // NOLINT
        control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
        control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


        xt::view(control_sequence_.vx, -1) =
          xt::view(control_sequence_.vx, -2);

        xt::view(control_sequence_.wz, -1) =
          xt::view(control_sequence_.wz, -2);


        if (isHolonomic()) {
            control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
            xt::view(control_sequence_.vy, -1) =
              xt::view(control_sequence_.vy, -2);
        }
    }

  /**
   * @brief updates generated trajectories with noised trajectories
   * from the last cycle's optimal control
   */
    void generateNoisedTrajectories()
    {
        noise_generator_.setNoisedControls(state_, control_sequence_);
        noise_generator_.generateNextNoises();
        updateStateVelocities(state_);
        integrateStateVelocities(generated_trajectories_, state_);
    }

  /**
   * @brief Apply hard vehicle constraints on control sequence
   */
    void applyControlSequenceConstraints()
    {
        auto & s = settings_;

        if (isHolonomic()) {
            control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
        }

        control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
        control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

        motion_model_->applyConstraints(control_sequence_);
    }


  /**
   * @brief  Update velocities in state
   * @param state fill state with velocities on each step
   */
    void updateStateVelocities(
      models::State & state) const
    {
        updateInitialStateVelocities(state);
        propagateStateVelocitiesFromInitials(state);
    }


  /**
   * @brief  Update initial velocity in state
   * @param state fill state
   */
    void updateInitialStateVelocities(
      models::State & state) const
    {
        xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
        xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

        if (isHolonomic()) {
            xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
        }
    }
  /**
   * @brief predict velocities in state using model
   * for time horizon equal to timesteps
   * @param state fill state
   */
    void propagateStateVelocitiesFromInitials(
      models::State & state) const
    {
        motion_model_->predict(state);
    }

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
    void integrateStateVelocities(
		xt::xtensor<float, 2> & trajectory,
		const xt::xtensor<float, 2> & sequence) const
    {
        float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

        const auto vx = xt::view(sequence, xt::all(), 0);
        const auto vy = xt::view(sequence, xt::all(), 2);
        const auto wz = xt::view(sequence, xt::all(), 1);

        auto traj_x = xt::view(trajectory, xt::all(), 0);
        auto traj_y = xt::view(trajectory, xt::all(), 1);
        auto traj_yaws = xt::view(trajectory, xt::all(), 2);

        xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

        auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
        auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

        const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

        xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
        xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
        xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
        xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

        auto && dx = xt::eval(vx * yaw_cos);
        auto && dy = xt::eval(vx * yaw_sin);

        if (isHolonomic()) {
            dx = dx - vy * yaw_sin;
            dy = dy + vy * yaw_cos;
        }

        xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
        xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
    }

	void integrateStateVelocities(
		models::Trajectories & trajectories,
		const models::State & state) const
	{
		const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

		xt::noalias(trajectories.yaws) =
            xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw;

		const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

		auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
		auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
		xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
		xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
		xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
		xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

		auto && dx = xt::eval(state.vx * yaw_cos);
		auto && dy = xt::eval(state.vx * yaw_sin);

		if (isHolonomic()) {
			dx = dx - state.vy * yaw_sin;
			dy = dy + state.vy * yaw_cos;
		}

		xt::noalias(trajectories.x) = state.pose.pose.position.x +
            xt::cumsum(dx * settings_.model_dt, 1);
		xt::noalias(trajectories.y) = state.pose.pose.position.y +
            xt::cumsum(dy * settings_.model_dt, 1);
	}


  /**
   * @brief Update control sequence with state controls weighted by costs
   * using softmax function
   */
    void updateControlSequence()
    {
        auto & s = settings_;
        auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
        auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
        xt::noalias(costs_) +=
          s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
          xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
        xt::noalias(costs_) +=
          s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
          xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

        if (isHolonomic()) {
            auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
            xt::noalias(costs_) +=
              s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
              xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
              1, immediate);
        }

        auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
        auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
        auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
        auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

        xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
        xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
        if (isHolonomic()) {
            xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
        }

        applyControlSequenceConstraints();
    }

  /**
   * @brief Convert control sequence to a twist commant
   * @param stamp Timestamp to use
   * @return TwistStamped of command to send to robot base
   */
    geometry_msgs::msg::TwistStamped getControlFromSequenceAsTwist(
        const builtin_interfaces::msg::Time & stamp, const nav2_mppi_controller::IMapView& map_view)
    {
        unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

        auto vx = control_sequence_.vx(offset);
        auto wz = control_sequence_.wz(offset);

        if (isHolonomic()) {
            auto vy = control_sequence_.vy(offset);
            return utils::toTwistStamped(vx, vy, wz, stamp, map_view.getBaseFrameID());
        }

        return utils::toTwistStamped(vx, wz, stamp, map_view.getBaseFrameID());
    }
  /**
   * @brief Whether the motion model is holonomic
   * @return Bool if holonomic to populate `y` axis of state
   */
    bool isHolonomic() const {return motion_model_->isHolonomic();}

  /**
   * @brief Using control frequence and time step size, determine if trajectory
   * offset should be used to populate initial state of the next cycle
   */
    void setOffset(double controller_frequency)
    {
        const double controller_period = 1.0 / controller_frequency;
        constexpr double eps = 1e-6;

        if ((controller_period + eps) < settings_.model_dt) {
            RCLCPP_WARN(
              logger_,
              "Controller period is less then model dt, consider setting it equal");
        } else if (abs(controller_period - settings_.model_dt) < eps) {
            RCLCPP_INFO(
              logger_,
              "Controller period is equal to model dt. Control sequence "
              "shifting is ON");
            settings_.shift_control_sequence = true;
        } else {
            throw std::runtime_error(
                    "Controller period more then model dt, set it equal to model dt");
        }
    }


  /**
   * @brief Perform fallback behavior to try to recover from a set of trajectories in collision
   * @param fail Whether the system failed to recover from
   */
    bool fallback(bool fail)
    {
        static size_t counter = 0;

        if (!fail) {
            counter = 0;
            return false;
        }

        reset();

        if (++counter > settings_.retry_attempt_limit) {
            counter = 0;
            throw std::runtime_error("Optimizer fail to compute path");
        }

        return true;
    }

protected:
//  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;

  std::shared_ptr<MotionModel> motion_model_;

//  ParametersHandler * parameters_handler_;
  CriticManager critic_manager_;
  NoiseGenerator noise_generator_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  models::OptimizerSettings settings_;

  models::State state_;
  models::ControlSequence control_sequence_;
  std::array<mppi::models::Control, 4> control_history_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  xt::xtensor<float, 1> costs_;

  CriticData critics_data_ =
  {state_, generated_trajectories_, path_, costs_, settings_.model_dt, false, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
