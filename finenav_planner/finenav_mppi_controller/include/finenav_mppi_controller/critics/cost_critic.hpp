// Copyright (c) 2023 Robocc Brice Renaudeau
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_

#include <memory>
#include <string>


#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"



namespace mppi::critics
{

/**
 * @class mppi::critics::CostCritic
 * @brief Critic objective function for avoiding obstacles using costmap's inflated cost
 */

class CostCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Point cost at pose center
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return bool if in collision
    */
    bool inCollision(float x, float y, float theta, const nav2_mppi_controller::IMapView& map_view)
    {
        return map_view.isCollision(x, y, theta);
    }


  /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @return Collision information at pose
    */
    float costAtPose(float x, float y, const nav2_mppi_controller::IMapView& map_view)
    {
        // 使用 Position3D 作为参数传入 map_view
        return map_view.getCost(Position3D{static_cast<double>(x),
                                                    static_cast<double>(y),
                                                    0.0});
    }

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
//  float findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);

protected:
  float possibly_inscribed_cost_;

//  bool consider_footprint_{true};
  float circumscribed_radius_{0};
  float circumscribed_cost_{0};
  float collision_cost_{0};
  float critical_cost_{0};
  float weight_{0};

  float near_goal_distance_;
  unsigned int power_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
