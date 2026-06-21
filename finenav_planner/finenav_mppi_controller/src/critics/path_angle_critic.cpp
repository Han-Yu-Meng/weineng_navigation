// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include "finenav_mppi_controller/critics/path_angle_critic.hpp"

#include <math.h>

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void PathAngleCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.PathAngleCritic;
  enabled_               = c.enabled;
  power_                 = static_cast<unsigned int>(c.cost_power);
  weight_                = static_cast<float>(c.cost_weight);
  offset_from_furthest_  = static_cast<size_t>(c.offset_from_furthest);
  threshold_to_consider_ = static_cast<float>(c.threshold_to_consider);
  max_angle_to_furthest_ = static_cast<float>(c.max_angle_to_furthest);
  forward_preference_    = c.forward_preference;

  // Determine reversing capability from top-level vx_min (physical capability)
  const float vx_min   = static_cast<float>(p.vx_min);
  reversing_allowed_    = (vx_min < 0.0f);
}

void PathAngleCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }

  if (utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

  const float goal_x = xt::view(data.path.x, offseted_idx);
  const float goal_y = xt::view(data.path.y, offseted_idx);

  if (utils::posePointAngle(
      data.state.pose.pose, goal_x, goal_y, forward_preference_) < max_angle_to_furthest_)
  {
    return;
  }

  auto yaws_between_points = xt::atan2(
    goal_y - data.trajectories.y,
    goal_x - data.trajectories.x);

  auto yaws =
    xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

  if (reversing_allowed_ && !forward_preference_) {
    const auto yaws_between_points_corrected = xt::where(
      yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
    const auto corrected_yaws = xt::abs(
      utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
    data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
      RCLCPP_DEBUG(logger_, "Scored");
  } else {
    data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
      RCLCPP_DEBUG(logger_, "Scored");
  }
}

}  // namespace mppi::critics

