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

#include <cmath>
#include "finenav_mppi_controller/critics/obstacles_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void ObstaclesCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.ObstaclesCritic;
  enabled_            = c.enabled;
  power_              = static_cast<unsigned int>(c.cost_power);
  repulsion_weight_   = static_cast<float>(c.repulsion_weight);
  critical_weight_    = static_cast<float>(c.critical_weight);
  consider_footprint_ = c.consider_footprint;
  collision_cost_     = static_cast<float>(c.collision_cost);
  collision_margin_distance_ = static_cast<float>(c.collision_margin_distance);
  near_goal_distance_ = static_cast<float>(c.near_goal_distance);
  inflation_scale_factor_ = static_cast<float>(c.cost_scaling_factor);
  inflation_radius_   = static_cast<float>(c.inflation_radius);

  RCLCPP_DEBUG(logger_, "Critic initialized");
}

//float ObstaclesCritic::findCircumscribedCost(
//  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
//{
//  double result = -1.0;
//  bool inflation_layer_found = false;
//
//  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
//  if (static_cast<float>(circum_radius) == ci<MapView>rcumscribed_radius_) {
//    // early return if footprint size is unchanged
//    return circumscribed_cost_;
//  }
//
//  // check if the costmap has an inflation layer
//  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
//    layer != costmap->getLayeredCostmap()->getPlugins()->end();
//    ++layer)
//  {
//    auto inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
//    if (!inflation_layer) {
//      continue;
//    }
//
//    inflation_layer_found = true;
//    const double resolution = costmap->getCostmap()->getResolution();
//    result = inflation_layer->computeCost(circum_radius / resolution);
//    auto getParam = parameters_handler_->getParamGetter(name_);
//    getParam(inflation_scale_factor_, "cost_scaling_factor", 10.0);
//    getParam(inflation_radius_, "inflation_radius", 0.55);
//  }
//
//  if (!inflation_layer_found) {
//    RCLCPP_WARN(
//      logger_,
//      "No inflation layer found in costmap configuration. "
//      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
//      "field to speed up collision checking by only checking the full footprint "
//      "when robot is within possibly-inscribed radius of an obstacle. This may "
//      "significantly slow down planning times and not avoid anything but absolute collisions!");
//  }
//
//  circumscribed_radius_ = static_cast<float>(circum_radius);
//  circumscribed_cost_ = static_cast<float>(result);
//
//  return circumscribed_cost_;
//}

float ObstaclesCritic::distanceToObstacle(const float & cost, const IMapView& map_view) const
{
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = map_view.getRadius();
  float dist_to_obj = (scale_factor * min_radius - log(cost) + log(253.0f)) / scale_factor;

  // If not footprint collision checking, the cost is using the center point cost and
  // needs the radius subtracted to obtain the closest distance to the object

  return dist_to_obj;
}


void ObstaclesCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }

//  if (consider_footprint_) {
//    // footprint may have changed since initialization if user has dynamic footprints
//    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
//  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
  }

  auto && raw_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  raw_cost.fill(0.0f);
  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0f);

  const size_t traj_len = data.trajectories.x.shape(1);
  bool all_trajectories_collide = true;
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
      bool trajectory_collide = false;
      float traj_cost = 0.0f;
      const auto & traj = data.trajectories;
      float pose_cost;

      for (size_t j = 0; j < traj_len; j++) {
          pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j),map_view);
          if (pose_cost < 1.0f) {continue;}  // In free space

          if (inCollision(pose_cost, map_view)) {
              trajectory_collide = true;
              break;
          }

          //   // Cannot process repulsion if inflation layer does not exist
          // if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
          //   continue;
          // }
          const float dist_to_obj = distanceToObstacle(pose_cost, map_view);

          // Let near-collision trajectory points be punished severely
          if (dist_to_obj < collision_margin_distance_) {
              traj_cost += (collision_margin_distance_ - dist_to_obj);
          } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
              repulsive_cost[i] += (inflation_radius_ - dist_to_obj);
          }
      }

      if (!trajectory_collide) {
          all_trajectories_collide = false;
      }
      raw_cost[i] = trajectory_collide ? collision_cost_ : traj_cost;
  }
  data.costs += xt::pow(
    (critical_weight_ * raw_cost) +
    (repulsion_weight_ * repulsive_cost / traj_len),
    power_);
  data.fail_flag = all_trajectories_collide;
}
/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */


}  // namespace mppi::critics

