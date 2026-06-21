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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

//#include "finenav_mppi_controller/tools/parameters_handler.hpp"
#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */

class CriticFunction
{
public:
  /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
  CriticFunction() = default;

  /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    const finenav_mppi_controller::Params & params)
  {
    parent_ = parent;
    name_ = name;
    parent_name_ = parent_name;

    initialize(params);
  }

  /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    */
  virtual void score(CriticData & data, const nav2_mppi_controller::IMapView & map_view) = 0;

  /**
    * @brief Initialize critic
    */
  virtual void initialize(const finenav_mppi_controller::Params & params) = 0;

  /**
    * @brief Get name of critic#include "finenav_control_layers/local_layer.hpp"
    */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;
  std::string name_, parent_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("CriticFunction")};

};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
