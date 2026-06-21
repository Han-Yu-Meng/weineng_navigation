// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include <rclcpp/time.hpp>

#include "finenav_localizer/filter_model.hpp"

namespace finenav {

// ============================================================================
//  NavStateBuffer  —  rolling time-stamped history of NavStateD
//
//  Maintains a sliding window of (timestamp, state) pairs ordered by time.
//  Supports:
//    • push()       — append latest state, prune entries older than the window.
//    • getStateAt() — return interpolated state at an arbitrary past timestamp.
//                     Returns std::nullopt for timestamps outside [oldest, newest].
//    • getLatest()  — return the newest entry.
//    • clear()      — discard all history (e.g. on re-initialisation).
//
//  Threading
//  ─────────────────────────────────────────────────────────────────────────
//  All public methods are thread-safe via an internal std::mutex.
//
//  Interpolation
//  ─────────────────────────────────────────────────────────────────────────
//  • p, v, omega : linear interpolation.
//  • R           : SO3 geodesic interpolation (Slerp via Sophus SO3::exp/log).
// ============================================================================
class NavStateBuffer {
public:
    using SharedPtr = std::shared_ptr<NavStateBuffer>;

    struct StampedState {
        rclcpp::Time stamp;
        NavStateD    state;
    };

    /**
     * @param history_duration_sec  Length of the rolling window [s].
     *                               Entries older than (newest_stamp - window) are pruned.
     */
    explicit NavStateBuffer(double history_duration_sec = 2.0)
        : history_duration_sec_(history_duration_sec)
    {}

    // ── Write ────────────────────────────────────────────────────────────────

    /// Append a new (stamp, state) pair and prune expired entries.
    /// @note stamp must be monotonically non-decreasing; out-of-order pushes are silently ignored.
    void push(const rclcpp::Time & stamp, const NavStateD & state)
    {
        std::lock_guard<std::mutex> lk(mutex_);

        // Reject out-of-order updates.
        if (!buffer_.empty() && stamp < buffer_.back().stamp) {
            return;
        }

        buffer_.push_back({stamp, state});

        // Prune entries that fall outside the history window.
        prune(stamp);
    }

    /// Clear all history (call after setInitialPose).
    void clear()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        buffer_.clear();
    }

    // ── Read ─────────────────────────────────────────────────────────────────

    /**
     * @brief Interpolate (or clamp) the state at the given timestamp.
     *
     * Returns std::nullopt if:
     *   - The buffer is empty.
     *   - @p query_time is strictly newer than the latest entry (future queries refused).
     *   - @p query_time is strictly older than the oldest retained entry.
     *
     * If @p query_time exactly matches an entry, that entry is returned directly.
     */
    std::optional<NavStateD> getStateAt(const rclcpp::Time & query_time) const
    {
        std::lock_guard<std::mutex> lk(mutex_);

        if (buffer_.empty()) {
            return std::nullopt;
        }

        // Refuse future queries.
        if (query_time > buffer_.back().stamp) {
            return std::nullopt;
        }

        // Clamp: query older than retained window → no data.
        if (query_time < buffer_.front().stamp) {
            return std::nullopt;
        }

        // Binary search: find the first entry with stamp >= query_time.
        auto it = std::lower_bound(
            buffer_.begin(), buffer_.end(), query_time,
            [](const StampedState & s, const rclcpp::Time & t) {
                return s.stamp < t;
            });

        // Exact match.
        if (it->stamp == query_time) {
            return it->state;
        }

        // it points to the first entry AFTER query_time; interpolate between
        // (it-1) and it.
        const auto & s1 = *(it - 1);
        const auto & s2 = *it;

        const double span = (s2.stamp - s1.stamp).seconds();
        if (span <= 0.0) {
            return s1.state;  // degenerate: identical stamps
        }

        const double alpha = (query_time - s1.stamp).seconds() / span;
        return interpolate(s1.state, s2.state, alpha);
    }

    /// Return the newest (stamp, state) pair, or nullopt if buffer is empty.
    std::optional<StampedState> getLatest() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (buffer_.empty()) {
            return std::nullopt;
        }
        return buffer_.back();
    }

    /// Number of retained entries (primarily for diagnostics / tests).
    std::size_t size() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return buffer_.size();
    }

private:
    // ── Helpers ──────────────────────────────────────────────────────────────

    /// Prune entries older than (newest_stamp - history_duration_sec_).
    /// Must be called with mutex_ held.
    void prune(const rclcpp::Time & newest)
    {
        if (history_duration_sec_ <= 0.0) {
            return;
        }
        // Compute cutoff as a duration subtraction to stay in RCL_ROS_TIME.
        const rclcpp::Duration window{
            static_cast<int32_t>(static_cast<int64_t>(history_duration_sec_) / 1),
            static_cast<uint32_t>(
                (history_duration_sec_ - static_cast<int64_t>(history_duration_sec_)) * 1e9)};
        const rclcpp::Time cutoff = newest - rclcpp::Duration::from_seconds(history_duration_sec_);

        while (buffer_.size() > 1 && buffer_.front().stamp < cutoff) {
            buffer_.pop_front();
        }
    }

    /// Linearly interpolate between two NavStateD at ratio alpha in [0,1].
    static NavStateD interpolate(const NavStateD & a, const NavStateD & b, double alpha)
    {
        NavStateD out;
        out.p     = a.p     + alpha * (b.p     - a.p);
        out.v     = a.v     + alpha * (b.v     - a.v);
        out.omega = a.omega + alpha * (b.omega - a.omega);

        // SO3 geodesic interpolation: a ⊕ (alpha · log(a⁻¹ · b))
        const Sophus::SO3d delta = a.R.inverse() * b.R;
        out.R = a.R * Sophus::SO3d::exp(alpha * delta.log());

        return out;
    }

    double                   history_duration_sec_;
    mutable std::mutex       mutex_;
    std::deque<StampedState> buffer_;
};

}  // namespace finenav

