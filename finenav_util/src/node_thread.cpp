#include "finenav_util/node_thread.hpp"

namespace finenav {

NodeThread::NodeThread(rclcpp::Node::SharedPtr node) : node_(node) {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    thread_ = std::make_unique<std::thread>([this]() {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) : executor_(executor) {
    thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

NodeThread::~NodeThread() {
    executor_->cancel();
    thread_->join();
}

}  // namespace finenav
