// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "finenav_core/concepts.hpp"
namespace finenav {
// Forward declaration — UpdateSource is owned by MapServer.
template <MapConcept MapT>
class MapServer;
// ==============================================================================
// IUpdateSource — type-erased interface for MapServer's source list
// ==============================================================================
template <MapConcept MapT>
class IUpdateSource {
public:
    virtual ~IUpdateSource() = default;
    virtual void applyBuffered(MapT& map, const rclcpp::Time& now) = 0;
};
// ==============================================================================
// UpdateSource — typed concrete source, holds buffered messages and a fuser
// ==============================================================================
/**
 * @brief UpdateSource — 持有单个数据源的缓存队列与融合函数。
 *
 * 线程安全：enqueueThreadSafe() 可从任意线程调用；
 * applyBuffered() 由 MapServer 的更新定时器线程调用（持有写锁）。
 *
 * @tparam MapT  目标地图类型
 * @tparam MsgT  输入消息类型
 */
template <MapConcept MapT, typename MsgT>
class UpdateSource final : public IUpdateSource<MapT> {
public:
    struct SourceBufferPolicy {
        std::size_t max_buffer_size{32};
        double observation_keep_time_sec{0.0};
        double source_timeout_sec{0.0};
    };
    using Fuser = std::function<void(const MsgT& msg, const rclcpp::Time& stamp, MapT& map)>;
    UpdateSource(const std::string& source_name, SourceBufferPolicy policy, Fuser fuser);
    /** Thread-safe: 从任意线程推入一条消息。 */
    void enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp);
    /** 在写锁保护下，将缓存消息全部融合进地图。 */
    void applyBuffered(MapT& map, const rclcpp::Time& now) override;
private:
    struct BufferedMessage {
        rclcpp::Time stamp;
        MsgT msg;
    };
    void enqueue(MsgT&& msg, const rclcpp::Time& stamp);
    void pruneExpired(const rclcpp::Time& now);
    std::string source_name_;
    SourceBufferPolicy policy_;
    Fuser fuser_;
    std::mutex mutex_;
    std::deque<BufferedMessage> buffer_;
    rclcpp::Time last_seen_stamp_{0, 0, RCL_ROS_TIME};
    bool has_seen_message_{false};
};
// ==============================================================================
// UpdateSource Implementation
// ==============================================================================
template <MapConcept MapT, typename MsgT>
UpdateSource<MapT, MsgT>::UpdateSource(
    const std::string& source_name, SourceBufferPolicy policy, Fuser fuser)
    : source_name_(source_name), policy_(policy), fuser_(std::move(fuser)) {}
template <MapConcept MapT, typename MsgT>
void UpdateSource<MapT, MsgT>::enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    enqueue(std::move(msg), stamp);
}
template <MapConcept MapT, typename MsgT>
void UpdateSource<MapT, MsgT>::applyBuffered(MapT& map, const rclcpp::Time& now) {
    std::deque<BufferedMessage> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pruneExpired(now);
        if (policy_.source_timeout_sec > 0.0 && has_seen_message_) {
            const auto elapsed = (now - last_seen_stamp_).seconds();
            if (elapsed > policy_.source_timeout_sec) {
                if (!buffer_.empty()) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("MapServer"),
                        "UpdateSource '" << source_name_ << "' has timed out ("
                        << elapsed << "s). Buffer cleared!");
                }
                buffer_.clear();
            }
        }
        pending.swap(buffer_);
    }
    if (!fuser_) return;
    if (!pending.empty()) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("MapServer"),
            "UpdateSource '" << source_name_ << "' applying "
            << pending.size() << " buffered messages.");
    }
    for (const auto& item : pending) {
        fuser_(item.msg, item.stamp, map);
    }
}
template <MapConcept MapT, typename MsgT>
void UpdateSource<MapT, MsgT>::enqueue(MsgT&& msg, const rclcpp::Time& stamp) {
    if (policy_.max_buffer_size > 0 && buffer_.size() >= policy_.max_buffer_size) {
        buffer_.pop_front();
    }
    buffer_.push_back(BufferedMessage{stamp, std::move(msg)});
    last_seen_stamp_  = stamp;
    has_seen_message_ = true;
}
template <MapConcept MapT, typename MsgT>
void UpdateSource<MapT, MsgT>::pruneExpired(const rclcpp::Time& now) {
    if (policy_.observation_keep_time_sec <= 0.0) return;
    while (!buffer_.empty()) {
        if ((now - buffer_.front().stamp).seconds() <= policy_.observation_keep_time_sec) break;
        buffer_.pop_front();
    }
}
} // namespace finenav
