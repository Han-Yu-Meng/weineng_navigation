// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/condition/is_goal_reached.hpp"
#include "finenav_navigator/plugins/condition/is_stuck.hpp"
#include "behaviortree_cpp/bt_factory.h"

// Pure BT plugin (no ROS node params needed) — uses BT::PLUGIN_SYMBOL
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<finenav::IsGoalReached>("IsGoalReached");
    factory.registerNodeType<finenav::IsStuck>("IsStuck");
}

