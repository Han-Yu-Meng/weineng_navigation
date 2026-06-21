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

#include "finenav_mppi_controller/critics/constraint_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void ConstraintCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.ConstraintCritic;
  enabled_ = c.enabled;
  power_   = static_cast<unsigned int>(c.cost_power);
  weight_  = static_cast<float>(c.cost_weight);

  // Velocity limits come from the top-level optimizer params
  const float vx_max = static_cast<float>(p.vx_max);
  const float vy_max = static_cast<float>(p.vy_max);
  const float vx_min = static_cast<float>(p.vx_min);

  const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);

  RCLCPP_DEBUG(logger_, "Critic initialized");
}

void ConstraintCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }
  auto sgn = xt::where(data.state.vx > 0.0, 1.0, -1.0);
  auto vel_total = sgn * xt::sqrt(data.state.vx * data.state.vx + data.state.vy * data.state.vy);
  auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);
  auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);

  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    auto out_of_turning_rad_motion = xt::maximum(
      acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0);

    data.costs += xt::pow(
      xt::sum(
        (std::move(out_of_max_bounds_motion) +
        std::move(out_of_min_bounds_motion) +
        std::move(out_of_turning_rad_motion)) *
        data.model_dt, {1}, immediate) * weight_, power_);
      RCLCPP_DEBUG(logger_, "[ConstraintCritic]: Scored");

    return;
  }

  data.costs += xt::pow(
    xt::sum(
      (std::move(out_of_max_bounds_motion) +
      std::move(out_of_min_bounds_motion)) *
      data.model_dt, {1}, immediate) * weight_, power_);
    RCLCPP_DEBUG(logger_, "[ConstraintCritic]: Scored");
}

}  // namespace mppi::critics

