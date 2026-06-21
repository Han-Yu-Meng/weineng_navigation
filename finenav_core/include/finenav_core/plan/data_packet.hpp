// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once
#include <cstdint>

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <builtin_interfaces/msg/duration.hpp>

namespace finenav {

struct IOProfile {
    bool pose = false;
    bool vel  = false;
    bool accel= false;
    bool time = false;
};

template <bool Enable> struct PoseMixin;
template <> struct PoseMixin<true> { std::vector<geometry_msgs::msg::Pose> poses; };
template <> struct PoseMixin<false> {};

template <bool Enable> struct VelMixin;
template <> struct VelMixin<true> { std::vector<geometry_msgs::msg::Twist> twists; };
template <> struct VelMixin<false> {};

template <bool Enable> struct AccelMixin;
template <> struct AccelMixin<true> { std::vector<geometry_msgs::msg::Accel> accels; };
template <> struct AccelMixin<false> {};

template <bool Enable> struct TimeMixin;
template <> struct TimeMixin<true> { std::vector<builtin_interfaces::msg::Duration> times; };
template <> struct TimeMixin<false> {};


struct EmptyBlock {};

// 核心容器：根据 C++20 Config 对象进行多重继承
template <IOProfile Config>
struct DataPacket :
    public PoseMixin<Config.pose>,
    public VelMixin<Config.vel>,
    public AccelMixin<Config.accel>,
    public TimeMixin<Config.time>
{};

// 类型别名 helper
template <typename PlannerT>
using InputDataT = DataPacket<PlannerT::InputProfile>;

template <typename PlannerT>
using OutputDataT = DataPacket<PlannerT::OutputProfile>;



}


