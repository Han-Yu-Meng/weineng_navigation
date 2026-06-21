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

#include "finenav_mppi_controller/critics/path_align_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{
using namespace nav2_mppi_controller;
using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void PathAlignCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.PathAlignCritic;
  enabled_                  = c.enabled;
  power_                    = static_cast<unsigned int>(c.cost_power);
  weight_                   = static_cast<float>(c.cost_weight);
  max_path_occupancy_ratio_ = static_cast<float>(c.max_path_occupancy_ratio);
  offset_from_furthest_     = static_cast<size_t>(c.offset_from_furthest);
  trajectory_point_step_    = c.trajectory_point_step;
  threshold_to_consider_    = static_cast<float>(c.threshold_to_consider);
  use_path_orientations_    = c.use_path_orientations;
}

void PathAlignCritic::score(CriticData & data, const IMapView& map_view)
{
  // Don't apply close to goal, let the goal critics take over

  if (!this->enabled_ )
  {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path
  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;  // up to furthest only
  if (path_segments_count < offset_from_furthest_) {
      RCLCPP_DEBUG(logger_, "Warning");
    return;
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the local path
  utils::setPathCostsIfNotSet(data, map_view);
  const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
  unsigned int invalid_ctr = 0;
  const float range = *data.furthest_reached_path_point - closest_initial_path_point;
  for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
      return;
    }
  }

  const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
  const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points
  const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));  // path points

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  // Find integrated distance in the path
  std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
  float dx = 0.0f, dy = 0.0f;
  for (unsigned int i = 1; i != path_segments_count; i++) {
    dx = P_x(i) - P_x(i - 1);
    dy = P_y(i) - P_y(i - 1);
    float curr_dist = sqrtf(dx * dx + dy * dy);
    path_integrated_distances[i] = path_integrated_distances[i - 1] + curr_dist;
  }

  float traj_integrated_distance = 0.0f;
  float summed_path_dist = 0.0f, dyaw = 0.0f;
  float num_samples = 0.0f;
  float Tx = 0.0f, Ty = 0.0f;
  size_t path_pt = 0;
  for (size_t t = 0; t < batch_size; ++t) {
    traj_integrated_distance = 0.0f;
    summed_path_dist = 0.0f;
    num_samples = 0.0f;
    path_pt = 0u;
    const auto T_x = xt::view(data.trajectories.x, t, xt::all());
    const auto T_y = xt::view(data.trajectories.y, t, xt::all());
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
      Tx = T_x(p);
      Ty = T_y(p);
      dx = Tx - T_x(p - trajectory_point_step_);
      dy = Ty - T_y(p - trajectory_point_step_);
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
      path_pt = utils::findClosestPathPt(
        path_integrated_distances, traj_integrated_distance, path_pt);

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
      if ((*data.path_pts_valid)[path_pt]) {
        dx = P_x(path_pt) - Tx;
        dy = P_y(path_pt) - Ty;
        num_samples += 1.0f;
        if (use_path_orientations_) {
          const auto T_yaw = xt::view(data.trajectories.yaws, t, xt::all());
          dyaw = angles::shortest_angular_distance(P_yaw(path_pt), T_yaw(p));
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }
    }
    if (num_samples > 0) {
      cost[t] = summed_path_dist / num_samples;
    } else {
      cost[t] = 0.0f;
    }
  }

  data.costs += xt::pow(std::move(cost) * weight_, power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

