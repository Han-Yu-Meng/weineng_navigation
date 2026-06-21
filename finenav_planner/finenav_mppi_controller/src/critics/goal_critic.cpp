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

#include "finenav_mppi_controller/critics/goal_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

using xt::evaluation_strategy::immediate;

void GoalCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.GoalCritic;
  enabled_               = c.enabled;
  power_                 = static_cast<unsigned int>(c.cost_power);
  weight_                = static_cast<float>(c.cost_weight);
  threshold_to_consider_ = static_cast<float>(c.threshold_to_consider);
}

void GoalCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_) {
    return;
  }

  // Only activate when robot is near the goal pose
  if (!utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
  }

  const auto goal_idx = data.path.x.shape(0) - 1;

  const auto goal_x = data.path.x(goal_idx);
  const auto goal_y = data.path.y(goal_idx);

  const auto traj_x = xt::view(data.trajectories.x, xt::all(), xt::all());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), xt::all());

  auto dists = xt::sqrt(
    xt::pow(traj_x - goal_x, 2) +
    xt::pow(traj_y - goal_y, 2));

  data.costs += xt::pow(xt::mean(dists, {1}, immediate) * weight_, power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

