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

#include "finenav_mppi_controller/critics/path_follow_critic.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void PathFollowCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.PathFollowCritic;
  enabled_               = c.enabled;
  power_                 = static_cast<unsigned int>(c.cost_power);
  weight_                = static_cast<float>(c.cost_weight);
  offset_from_furthest_  = static_cast<size_t>(c.offset_from_furthest);
  threshold_to_consider_ = static_cast<float>(c.threshold_to_consider);
}

void PathFollowCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_ || data.path.x.shape(0) < 2 ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }
  utils::setPathFurthestPointIfNotSet(data);
  utils::setPathCostsIfNotSet(data, map_view);
  const size_t path_size = data.path.x.shape(0) - 1;

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_size-1);
  // Drive to the first valid path point, in case of dynamic obstacles on path
  // we want to drive past it, not through it
  bool valid = false;
  while (!valid && offseted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offseted_idx];
    if (!valid) {
      offseted_idx++;
    }
  }

  const auto path_x = data.path.x(offseted_idx);
  const auto path_y = data.path.y(offseted_idx);

  const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
  const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

  auto dists = xt::sqrt(
    xt::pow(last_x - path_x, 2) +
    xt::pow(last_y - path_y, 2));

  data.costs += xt::pow(weight_ * std::move(dists), power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

