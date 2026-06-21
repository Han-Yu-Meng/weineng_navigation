// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "finenav_util/algo_configurator.hpp"
#include "finenav_util/node_thread.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_core/concepts.hpp"
#include "finenav_core/map/locked_map_view.hpp"
#include "finenav_core/map/update_source.hpp"

namespace finenav {

// ==============================================================================
// MapServer Declaration
// ==============================================================================

/**
 * @brief MapServer — 管理一张地图实例的生命周期、并发访问与周期性更新。
 *
 * - 通过 addUpdateSource() 注册数据源，由内部定时器定期将缓存消息融合进地图。
 * - 通过 getLockedReadView() / getLockedWriteView() 获取 RAII 锁视图，保证线程安全。
 * - 可选接入 FineNavLocalizer 实现地图窗口跟随机器人位移。
 *
 * @tparam MapT  地图类型，必须满足 MapConcept。
 */
template <MapConcept MapT>
class MapServer {
public:
    /**
     * @brief 单个数据源的缓存与超时策略。
     */
    struct SourceBufferPolicy {
        std::size_t max_buffer_size{32};       ///< 最大缓存帧数（0 = 无限制）
        double observation_keep_time_sec{0.0}; ///< 消息最大保留时长（0 = 永不过期）
        double source_timeout_sec{0.0};        ///< 数据源超时阈值（0 = 禁用）
    };

    using PreUpdateHook  = std::function<void(MapT& map)>;
    using PostUpdateHook = std::function<void(const MapT& map)>;

    template <typename MsgT>
    using Fuser = std::function<void(const MsgT& msg, const rclcpp::Time& stamp, MapT& map)>;

    /**
     * @brief 构造 MapServer，初始化内部定时器以定期执行地图更新。
     * @param node           ROS 2 节点指针
     * @param server_name    MapServer 的名称（同时作为私有子节点名）
     * @param update_rate_hz 传感器融合定时器频率（Hz），默认 10 Hz
     * @param localizer      可选的定位器指针；非空时启动地图位移定时器
     * @param shift_rate_hz  地图位移定时器频率（Hz），默认 50 Hz
     */
    explicit MapServer(
        rclcpp::Node::SharedPtr node,
        const std::string& server_name,
        double update_rate_hz = 10.0,
        FineNavLocalizer* localizer = nullptr,
        double shift_rate_hz = 50.0);

    /** @brief 注册在每次地图更新周期**前**执行的回调。 */
    void registerPreUpdateHook(std::function<void(MapT& map)> hook);

    /** @brief 注册在每次地图更新周期**后**执行的回调。 */
    void registerPostUpdateHook(std::function<void(MapT& map)> hook);

    /**
     * @brief 获取 RAII 读视图（共享锁）。
     * @return 受读锁保护的 LockedMapRO<MapT>
     */
    LockedMapRO<MapT> getLockedReadView();

    /**
     * @brief 获取 RAII 读写视图（独占锁）。用于主动即时修改地图。
     * @return 受写锁保护的 LockedMapRW<MapT>
     */
    LockedMapRW<MapT> getLockedWriteView();

    /**
     * @brief 注册一个数据源，返回线程安全的推送闭包。
     *
     * 返回的闭包可从任意线程调用，数据将被缓存至内部队列，
     * 并在下次 runUpdateCycle() 时由 fuser 融合进地图。
     *
     * @tparam MsgT       消息类型
     * @param source_name 数据源名称（用于日志）
     * @param policy      缓存与超时策略
     * @param fuser       融合函数：(const MsgT&, MapT&) -> void
     * @return 线程安全的推送闭包：void(MsgT&&, const rclcpp::Time&)
     */
    template <typename MsgT>
    std::function<void(MsgT&&, const rclcpp::Time&)> addUpdateSource(
        const std::string& source_name, SourceBufferPolicy policy, Fuser<MsgT> fuser);

private:
    void runUpdateCycle(const rclcpp::Time& now);
    void runShiftCycle(const rclcpp::Time& now);

    template <typename T>
    void keep_alive(std::shared_ptr<T> handle);

    // ---- ROS infrastructure ----
    rclcpp::Node::SharedPtr     node_;
    std::string                 server_name_;
    rclcpp::Node::SharedPtr     pnode_;
    std::unique_ptr<NodeThread> pnode_thread_;

    // ---- Map data & lock ----
    std::unique_ptr<MapT> map_instance_;
    std::shared_mutex     rw_mutex_;

    // ---- Optional localizer for map shifting ----
    FineNavLocalizer*            localizer_{nullptr};
    rclcpp::TimerBase::SharedPtr shift_timer_;

    // ---- Hooks, sources & timers ----
    std::vector<std::function<void(MapT&)>>          pre_update_hooks_;
    std::vector<std::function<void(MapT&)>>          post_update_hooks_;
    std::vector<std::shared_ptr<IUpdateSource<MapT>>> update_sources_;
    std::vector<std::shared_ptr<void>>               held_param_handles_;
    rclcpp::TimerBase::SharedPtr                     update_timer_;
};

}  // namespace finenav

#include "finenav_core/map/map_server_impl.hpp"

