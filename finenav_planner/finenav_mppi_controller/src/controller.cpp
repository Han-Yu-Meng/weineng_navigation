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

#include <stdint.h>
#include <chrono>
#include <tf2/exceptions.h>
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

//template <typename MapView>
//void MPPIController<MapView>::visualize(
//  nav_msgs::msg::Path transformed_plan,
//  const builtin_interfaces::msg::Time & cmd_stamp,
//  double z_height)
//{
//  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories", z_height);
//  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory", cmd_stamp, z_height);
//  trajectory_visualizer_.visualize(std::move(transformed_plan));
//}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
    path_ = path;
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

//FINENAV_REGISTER_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)
//FINENAV_REGISTER_PLANNER_TRAITS(nav2_mppi_controller::MPPIController,
//    finenav::TraitMask::NONE,
//    finenav::TraitMask::POSE)
}  // namespace nav2_mppi_controller

//#include "pluginlib/class_list_macros.hpp"
//PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
