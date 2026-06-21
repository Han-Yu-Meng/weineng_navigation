// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
// This file is included at the bottom of map_server.hpp.
// It contains out-of-line template method implementations for MapServer.
// Do NOT include this file directly — include map_server.hpp instead.
#pragma once
namespace finenav {
// ==============================================================================
// Constructor
// ==============================================================================
template <MapConcept MapT>
MapServer<MapT>::MapServer(
    rclcpp::Node::SharedPtr node,
    const std::string& server_name,
    double update_rate_hz,
    FineNavLocalizer* localizer,
    double shift_rate_hz)
    : node_(node), server_name_(server_name), localizer_(localizer)
{
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(server_name_, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);
    map_instance_ = std::make_unique<MapT>();
    keep_alive(AlgoConfigurator<MapT>::load(pnode_, *map_instance_));
    // 传感器观测更新定时器
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period,
        [this]() { this->runUpdateCycle(this->pnode_->now()); });
    // 地图窗口位移定时器（可选）
    if (localizer_ != nullptr) {
        auto shift_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / shift_rate_hz));
        shift_timer_ = rclcpp::create_timer(
            pnode_, pnode_->get_clock(), shift_period,
            [this]() { this->runShiftCycle(this->pnode_->now()); });
    }
}
// ==============================================================================
// Hook Registration
// ==============================================================================
template <MapConcept MapT>
void MapServer<MapT>::registerPreUpdateHook(std::function<void(MapT& map)> hook) {
    pre_update_hooks_.push_back(std::move(hook));
}
template <MapConcept MapT>
void MapServer<MapT>::registerPostUpdateHook(std::function<void(MapT& map)> hook) {
    post_update_hooks_.push_back(std::move(hook));
}
// ==============================================================================
// Locked View Accessors
// ==============================================================================
template <MapConcept MapT>
LockedMapRO<MapT> MapServer<MapT>::getLockedReadView() {
    return LockedMapRO<MapT>(*map_instance_, rw_mutex_);
}
template <MapConcept MapT>
LockedMapRW<MapT> MapServer<MapT>::getLockedWriteView() {
    return LockedMapRW<MapT>(*map_instance_, rw_mutex_);
}
// ==============================================================================
// addUpdateSource
// ==============================================================================
template <MapConcept MapT>
template <typename MsgT>
std::function<void(MsgT&&, const rclcpp::Time&)>
MapServer<MapT>::addUpdateSource(
    const std::string& source_name,
    SourceBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    using Source = UpdateSource<MapT, MsgT>;
    typename Source::SourceBufferPolicy src_policy{
        policy.max_buffer_size,
        policy.observation_keep_time_sec,
        policy.source_timeout_sec};
    auto source = std::make_shared<Source>(source_name, src_policy, std::move(fuser));
    update_sources_.push_back(source);
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
        "MapServer added update source: " << source_name);
    std::weak_ptr<Source> weak_source = source;
    return [weak_source](MsgT&& msg, const rclcpp::Time& stamp) {
        if (auto src = weak_source.lock()) {
            src->enqueueThreadSafe(std::move(msg), stamp);
        }
    };
}
// ==============================================================================
// Internal Update Cycles
// ==============================================================================
template <MapConcept MapT>
void MapServer<MapT>::runUpdateCycle(const rclcpp::Time& now) {
    std::unique_lock<std::shared_mutex> lock(rw_mutex_);
    for (auto& hook : pre_update_hooks_)  hook(*map_instance_);
    for (auto& src  : update_sources_)   src->applyBuffered(*map_instance_, now);
    for (auto& hook : post_update_hooks_) hook(*map_instance_);
}
template <MapConcept MapT>
void MapServer<MapT>::runShiftCycle(const rclcpp::Time& now) {
    auto latest_pose = localizer_->getState().p;
    auto latest_time = localizer_->getLastUpdateTime();
    // TODO: 必须有一个原子性获取时间+状态的接口，否则存在数据竞争风险
    std::unique_lock<std::shared_mutex> lock(rw_mutex_);
    map_instance_->shiftWindowTo(latest_pose);
    RCLCPP_DEBUG_STREAM(pnode_->get_logger(),
        "MapServer shifted with pose delayed by "
        << (now - latest_time).seconds() << " seconds");
}
// ==============================================================================
// Utility
// ==============================================================================
template <MapConcept MapT>
template <typename T>
void MapServer<MapT>::keep_alive(std::shared_ptr<T> handle) {
    if (handle) held_param_handles_.push_back(handle);
}
} // namespace finenav
