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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_

#include <memory>

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;
/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for avoiding obstacles, allowing it to deviate off
 * the planned path. This is important to tune in tandem with PathAlign to make a balance
 * between path-tracking and dynamic obstacle avoidance capabilities as desirable for a
 * particular application
 */

class ObstaclesCritic : public CriticFunction
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
    void score(CriticData & data,const nav2_mppi_controller::IMapView& map_view) override;

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Costmap cost
    * @return bool if in collision
    */
    bool inCollision(float cost, const nav2_mppi_controller::IMapView& map_view) const
    {
        bool is_tracking_unknown =
          map_view.isTrackingUnknown();

        switch (static_cast<unsigned char>(cost)) {
            // 使用具体数值替代 nav2_costmap_2d 宏
            // NO_INFORMATION = 255, LETHAL_OBSTACLE = 254, INSCRIBED_INFLATED_OBSTACLE = 253
            case 254: // LETHAL_OBSTACLE
                return true;
            case 253: // INSCRIBED_INFLATED_OBSTACLE
                return consider_footprint_ ? false : true;
            case 255: // NO_INFORMATION
                return is_tracking_unknown ? false : true;
        }

        return false;
    }

  /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return Collision information at pose
    */
    float costAtPose(float x, float y, float theta, const nav2_mppi_controller::IMapView& map_view)
    {
        float collision_cost;
        collision_cost = map_view.getCost(Position3D{static_cast<double>(x),
                                                  static_cast<double>(y),
                                                  0.0});

        if (consider_footprint_ )
        {
            collision_cost = static_cast<float>(map_view.costAtPose(
                x, y, theta));
        }

        return collision_cost;
    }

  /**
    * @brief Distance to obstacle from cost
    * @param cost Costmap cost
    * @return float Distance to the obstacle represented by cost
    */
  float distanceToObstacle(const float & cost, const IMapView& map_view) const;


  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
  // float findCircumscribedCost(const IMapView& map_view);//这个函数仅在有膨胀机制时候有作用

protected:

  bool consider_footprint_{true};
  float collision_cost_{0};
  float inflation_scale_factor_{0}, inflation_radius_{0};

  float min_radius_{0};
  float possibly_inscribed_cost_;
  float collision_margin_distance_;
  float near_goal_distance_;
  float circumscribed_cost_{0}, circumscribed_radius_{0};

  unsigned int power_{0};
  float repulsion_weight_, critical_weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
