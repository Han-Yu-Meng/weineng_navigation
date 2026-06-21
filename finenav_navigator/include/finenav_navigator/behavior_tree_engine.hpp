// Copyright (c) 2026.bt_action_node.hpp
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "finenav_navigator/thirdparty/ros_node_params.hpp"
#include "finenav_navigator/thirdparty/plugins.hpp"

namespace finenav {

class BehaviorTreeEngine {
  public:
    explicit BehaviorTreeEngine(rclcpp::Node::SharedPtr node); //const std::vector<std::string> & plugin_libraries,
    virtual ~BehaviorTreeEngine() = default;

    void registerPlugins(const std::vector<std::string>& plugins);

    void registerBehaviorTrees(const std::vector<std::string>& tree_dirs);

    bool loadPlugin(BT::BehaviorTreeFactory& factory, const std::filesystem::path& file_path, BT::RosNodeParams params);

    bool loadBehaviorTree(BT::BehaviorTreeFactory& factory, const std::filesystem::path& file_path);

    BT::BehaviorTreeFactory& factory();

  private:
    // Expect "[package_name]/.../..."
    static std::pair<std::string, std::string> parseParamPath(const std::string& input);

    rclcpp::Logger logger_;
    BT::BehaviorTreeFactory factory_;
    BT::RosNodeParams ros_params_;

};


}