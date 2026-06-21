// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace finenav {

class NodeThread {
  public:

    explicit NodeThread(rclcpp::Node::SharedPtr node);

    explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

    NodeThread(const NodeThread&) = delete;
    NodeThread& operator=(const NodeThread&) = delete;
    NodeThread(NodeThread&&) = delete;
    NodeThread& operator=(NodeThread&&) = delete;

    ~NodeThread();

  protected:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<std::thread> thread_;
    rclcpp::Executor::SharedPtr executor_;
};

}  // namespace finenav
