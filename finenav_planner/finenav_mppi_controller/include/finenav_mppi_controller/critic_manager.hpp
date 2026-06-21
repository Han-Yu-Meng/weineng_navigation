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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

//#include "finenav_mppi_controller/tools/parameters_handler.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"
#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/critic_function.hpp"

#include "finenav_mppi_controller/critics/constraint_critic.hpp"
#include "finenav_mppi_controller/critics/cost_critic.hpp"
#include "finenav_mppi_controller/critics/goal_angle_critic.hpp"
#include "finenav_mppi_controller/critics/goal_critic.hpp"
#include "finenav_mppi_controller/critics/obstacles_critic.hpp"
#include "finenav_mppi_controller/critics/path_align_critic.hpp"
#include "finenav_mppi_controller/critics/path_align_legacy_critic.hpp"
#include "finenav_mppi_controller/critics/path_angle_critic.hpp"
#include "finenav_mppi_controller/critics/path_follow_critic.hpp"
#include "finenav_mppi_controller/critics/prefer_forward_critic.hpp"
#include "finenav_mppi_controller/critics/twirling_critic.hpp"
#include "finenav_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */

class CriticManager
{
public:
  /**
    * @brief Constructor for mppi::CriticManager
    */
  CriticManager() = default;

  /**
    * @brief Virtual Destructor for mppi::CriticManager
    */
  virtual ~CriticManager() = default;

  /**
    * @brief Configure critic manager on bringup and load plugins
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
     std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
     const finenav_mppi_controller::Params & params);

  /**
    * @brief Score trajectories by the set of loaded critic functions
    * @param CriticData Struct of necessary information to pass to the critic functions
    */

  void evalTrajectoriesScores(CriticData & data ,const nav2_mppi_controller::IMapView& map_view) const;

protected:
  /**
    * @brief Get parameters (critics to load)
    */
  void getParams();


  /**
    * @brief Get full-name namespaced critic IDs
    */
  std::string getFullName(const std::string & name);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
