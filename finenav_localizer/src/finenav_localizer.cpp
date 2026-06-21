// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/finenav_localizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace finenav {

// ============================================================================
//  Constructor
// ============================================================================
FineNavLocalizer::FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                                   rclcpp::NodeOptions pnode_overrides)
    : parent_node_(std::move(parent_node))
    , logger_(rclcpp::get_logger("finenav.FineNavLocalizer"))
{
    // ── 创建私有子节点（参数前缀由 yaml 顶层 key "finenav_localizer" 决定）──
    pnode_overrides.append_parameter_override(
        "use_sim_time",
        parent_node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_localizer",
        parent_node_->get_name(),
        pnode_overrides);

    // ── 读取参数 ──────────────────────────────────────────────────────────────
    param_listener_ = std::make_unique<ParamListener>(pnode_);
    params_ = param_listener_->get_params();

    // ── 初始化 ESKF ──────────────────────────────────────────────────────────
    NavigationModelD::Options model_opts;
    model_opts.vel_noise_std   = params_.model.vel_noise_std;
    model_opts.omega_noise_std = params_.model.omega_noise_std;

    // accumulated_delay_times_ slot layout (steady state):
    //   slot[0] = 0.0  (always overwritten to 0 in accumulateDelayTime)
    //   slot[i] = i * dt  (i = 1 … max_delay_step-1)
    // → maxTrackedDelaySec = (max_delay_step - 1) * dt
    //
    // To guarantee maxTrackedDelaySec >= max_delay_sec we need:
    //   (max_delay_step - 1) * dt >= max_delay_sec
    //   max_delay_step >= ceil(max_delay_sec * update_rate_hz) + 1
    //
    // Minimum of 2 is enforced so that slot[1] always exists and the
    // copy_backward in accumulateDelayTime has a non-empty source range.
    //
    // ⚠ NOTE on parameter semantics:
    //   The effective rejection threshold is maxTrackedDelaySec = (max_delay_step-1) * dt,
    //   where dt = 1/update_rate_hz.  This means the smallest achievable threshold
    //   is exactly one EKF period (dt).  Observations that arrived just after the
    //   previous tick will have delay ≈ dt + transport_delay, which can slightly exceed
    //   this threshold under system jitter.
    //
    //   Rule of thumb:  max_delay_sec  ≥  2 / update_rate_hz
    //   (gives two full periods of headroom and absorbs typical transport delays)
    const double min_recommended_delay_sec = 2.0 / params_.update_rate_hz;
    if (params_.max_delay_sec < min_recommended_delay_sec) {
        RCLCPP_WARN(
            logger_,
            "[FineNavLocalizer] max_delay_sec=%.4f s is less than the recommended minimum "
            "%.4f s (= 2 / update_rate_hz).  "
            "The effective rejection threshold equals one EKF period (%.4f s), so observations "
            "that arrive just after a tick will be rejected under normal scheduling jitter.  "
            "Consider setting max_delay_sec >= %.4f s.",
            params_.max_delay_sec,
            min_recommended_delay_sec,
            1.0 / params_.update_rate_hz,
            min_recommended_delay_sec);
    }

    const int max_delay_step = static_cast<int>(std::max(
        int64_t{2},
        static_cast<int64_t>(std::ceil(params_.max_delay_sec * params_.update_rate_hz)) + 1));
    eskf_ = ESKFd(model_opts, max_delay_step);

    pose_queue_  = std::make_unique<PoseObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, params_.pose_smoothing_steps)),
        static_cast<std::size_t>(params_.max_obs_queue_size));
    twist_queue_ = std::make_unique<TwistObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, params_.twist_smoothing_steps)),
        static_cast<std::size_t>(params_.max_obs_queue_size));
    rel_pose_queue_ = std::make_unique<RelativePoseObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, params_.rel_pose_smoothing_steps)),
        static_cast<std::size_t>(params_.max_rel_pose_queue_size));

    // ── State history buffer ──────────────────────────────────────────────────
    state_buffer_ = std::make_shared<NavStateBuffer>(params_.history_duration_sec);

    // ── TF broadcaster (only if publish_tf is enabled) ───────────────────────
    if (params_.publish_tf) {
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(pnode_);
    }

    // ── 更新定时器 ────────────────────────────────────────────────────────────
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / params_.update_rate_hz));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period,
        [this]() { timerCallback(); });

    // ── 启动 spin 线程（最后执行，确保所有订阅/服务已注册）────────────────────
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    RCLCPP_INFO(
        logger_,
        "[FineNavLocalizer] Created. rate=%.1f Hz  max_delay=%.3f s  "
        "max_delay_step=%d  pose_smooth=%ld  twist_smooth=%ld",
        params_.update_rate_hz, params_.max_delay_sec,
        eskf_.maxDelayStep(),
        params_.pose_smoothing_steps, params_.twist_smoothing_steps);
}

// ============================================================================
//  setInitialPose  (thread-safe via filter_mutex_)
// ============================================================================
void FineNavLocalizer::setInitialPose(const PoseMsg & msg)
{
    auto [pos, rot, R_obs] = extractObservation(msg);

    NavStateD init_state;
    init_state.p     = pos;
    init_state.R     = rot;
    init_state.v     = Vec3d::Zero();
    init_state.omega = Vec3d::Zero();

    PMat P = PMat::Zero();
    P.block<3, 3>(0, 0) = R_obs.block<3, 3>(0, 0);              // position
    P.block<3, 3>(3, 3) = R_obs.block<3, 3>(3, 3);              // rotation
    P.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 100.0;  // linear vel (unknown)
    P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 100.0;  // angular vel (unknown)

    {
        std::lock_guard<std::mutex> lk(filter_mutex_);
        // reinitialize() resets nominal state + ALL delay slots in TimeDelayKF
        // and resets accumulated_delay_times_ to 1e15.
        eskf_.reinitialize(init_state, P);
        pose_queue_->clear();
        twist_queue_->clear();
        rel_pose_queue_->clear();
        last_predict_time_.reset();
    }

    {
        std::unique_lock<std::shared_mutex> lk(cached_state_mutex_);
        cached_state_      = init_state;
        cached_covariance_ = P;
        cached_time_       = rclcpp::Time(msg.header.stamp);
    }

    state_buffer_->clear();
    state_buffer_->push(rclcpp::Time(msg.header.stamp), init_state);

    is_initialized_.store(true, std::memory_order_release);

    RCLCPP_INFO(
        logger_,
        "[FineNavLocalizer] Initial pose set: p=[%.3f, %.3f, %.3f]",
        pos.x(), pos.y(), pos.z());
}

// ============================================================================
//  feedObservation  (thread-safe: queue has its own mutex)
// ============================================================================
void FineNavLocalizer::feedObservation(const PoseMsg & msg)
{
    pose_queue_->push(msg);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedObservation<Pose>  queued (queue=%zu)",
        pose_queue_->size());
}

void FineNavLocalizer::feedObservation(const TwistMsg & msg)
{
    twist_queue_->push(msg);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedObservation<Twist> queued (queue=%zu)",
        twist_queue_->size());
}

void FineNavLocalizer::feedRelativePose(const RelativePoseObs & obs)
{
    rel_pose_queue_->push(obs);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedRelativePose queued (queue=%zu)",
        rel_pose_queue_->size());
}

// ============================================================================
//  Getters  (shared_mutex — multiple readers allowed)
// ============================================================================
NavStateD FineNavLocalizer::getState() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    NavStateD s = cached_state_;
    // External API contract: v is expressed in the body frame.
    // Internally the ESKF propagates v in world frame; convert here.
    s.v = s.R.inverse() * s.v;
    return s;
}

FineNavLocalizer::PMat FineNavLocalizer::getCovariance() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_covariance_;
}

rclcpp::Time FineNavLocalizer::getLastUpdateTime() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_time_;
}

std::optional<NavStateD> FineNavLocalizer::getStateAt(const rclcpp::Time & query_time) const
{
    auto s = state_buffer_->getStateAt(query_time);
    if (s) {
        // Same contract as getState(): convert v from world → body frame.
        s->v = s->R.inverse() * s->v;
    }
    return s;
}

// ============================================================================
//  timerCallback  — the sole writer of eskf_
// ============================================================================
void FineNavLocalizer::timerCallback()
{
    // 1. 检查参数更新
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
    }

    const rclcpp::Time now = pnode_->now();

    // ==========================================
    // 情况 A: 透传模式 (Zero Calculation)
    // ==========================================
    if (params_.passthrough) {
        bool has_new_data = false;
        NavStateD next_state;

        // 获取锁，保护 cached_state_ 和 eskf_ (虽然不运行eskf，但为了线程安全需要同步)
        std::lock_guard<std::mutex> lk(filter_mutex_);
        {
            std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
            next_state = cached_state_; // 以旧状态为基础

            // A1. 提取最新位姿 (Pose)
            if (!pose_queue_->empty()) {
                PoseMsg latest_pose;
                while (!pose_queue_->empty()) {
                    latest_pose = pose_queue_->pop_increment_age();
                }
                auto [pos, rot, R_pose] = extractObservation(latest_pose);
                next_state.p = pos;
                next_state.R = rot;
                cached_covariance_.block<6, 6>(0, 0) = R_pose; // 更新位姿协方差
                has_new_data = true;
            }

            // A2. 提取最新速度 (Twist/Odometry)
            if (!twist_queue_->empty()) {
                TwistMsg latest_twist;
                while (!twist_queue_->empty()) {
                    latest_twist = twist_queue_->pop_increment_age();
                }
                auto [lin_v_body, ang_v_body, R_twist] = extractObservation(latest_twist);

                // 注意：Twist消息里通常是 Body Frame。
                // 内部缓存要求 v 是 World Frame，以便 getState() 统一转换。
                next_state.v = next_state.R * lin_v_body;
                next_state.omega = ang_v_body;
                cached_covariance_.block<6, 6>(6, 6) = R_twist; // 更新速度协方差
                has_new_data = true;
            }

            if (has_new_data || is_initialized_.load(std::memory_order_acquire)) {
                cached_state_ = next_state;
                cached_time_  = now;
                // 将透传状态同步给 ESKF，防止切回 NORMAL 模式时状态跳变太大
                eskf_.setState(next_state);
            }
        }

        if (is_initialized_.load(std::memory_order_acquire)) {
            state_buffer_->push(now, next_state);
            // 广播 TF (复用原有逻辑)
            if (tf_br_) {
                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = now;
                tf.header.frame_id = "map";
                tf.child_frame_id = "base_link";
                tf.transform.translation.x = next_state.p.x();
                tf.transform.translation.y = next_state.p.y();
                tf.transform.translation.z = next_state.p.z();
                const Eigen::Quaterniond q(next_state.R.matrix());
                tf.transform.rotation.w = q.w();
                tf.transform.rotation.x = q.x();
                tf.transform.rotation.y = q.y();
                tf.transform.rotation.z = q.z();
                tf_br_->sendTransform(tf);
            }
        }
        return; // 透传结束，直接返回，不执行下方 ESKF 逻辑
    }

    // ==========================================
    // 情况 B: 正常 ESKF 模式 (原逻辑)
    // ==========================================
    
    if (!is_initialized_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            logger_, *pnode_->get_clock(), 2000,
            "[FineNavLocalizer] Not initialised — call setInitialPose() first.");
        return;
    }

    // tick_time is the EKF reference epoch for this cycle.
    // All observation delay calculations use tick_time as the reference baseline.
    // Sensor stamps may be slightly later than tick_time due to concurrent scheduling;
    // this is not a "future" observation — it belongs to the current tick (delay_step = 0).
    const rclcpp::Time tick_time = pnode_->now();
    std::lock_guard<std::mutex> lk(filter_mutex_);

    // The "from future" rejection threshold: one full tick period.
    // Normal concurrent scheduling jitter never exceeds one tick period;
    // anything beyond that indicates a genuine clock fault.
    const double future_reject_threshold = -1.0 / params_.update_rate_hz;

    // ── 1. Compute dt ──────────────────────────────────────────────────────
    double dt = 1.0 / params_.update_rate_hz;  // nominal fallback (first cycle)
    if (last_predict_time_.has_value()) {
        const double meas_dt = (tick_time - *last_predict_time_).seconds();
        if (meas_dt < 0.0) {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Time jumped backwards (dt=%.6f s) — skipping cycle.",
                meas_dt);
            last_predict_time_ = tick_time;
            return;
        }
        if (meas_dt > params_.max_dt_sec) {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Large dt (%.4f s > max %.4f s) — clamped.",
                meas_dt, params_.max_dt_sec);
            dt = params_.max_dt_sec;
        } else {
            dt = meas_dt;
        }
    }
    last_predict_time_ = tick_time;

    // ── 2. Predict ─────────────────────────────────────────────────────────
    // eskf_.predict() internally propagates the nominal state, slides the
    // TimeDelayKF extended covariance, and updates accumulated_delay_times_.
    RCLCPP_DEBUG(logger_, "[FineNavLocalizer] predict  dt=%.5f s", dt);
    eskf_.predict(dt);

    // ── 3. Process pose observations ───────────────────────────────────────
    // Capture size before the loop: pop_increment_age() may re-enqueue at the
    // back, but we process only the items that were present at the start of
    // this tick.
    {
        const std::size_t n = pose_queue_->size();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] pose observations this tick: %zu", n);

        for (std::size_t i = 0u; i < n; ++i) {
            const PoseMsg pose = pose_queue_->pop_increment_age();

            double delay_time = (tick_time - rclcpp::Time(pose.header.stamp)).seconds();

            // Clamp: negative delay means sensor stamp is slightly after tick_time
            // due to concurrent scheduling — treat as delay_step = 0 (current tick).
            // Only reject if the stamp exceeds one full tick period ahead, which
            // indicates a genuine clock fault.
            if (delay_time < future_reject_threshold) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Pose obs from future (delay=%.4f s, "
                    "threshold=%.4f s) — skipping.",
                    delay_time, future_reject_threshold);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);

            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Pose obs too old (delay=%.4f s, "
                    "max tracked=%.4f s) — rejecting.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(pose, static_cast<int>(delay_step));
        }
    }

    // ── 4. Process twist observations ──────────────────────────────────────
    {
        const std::size_t n = twist_queue_->size();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] twist observations this tick: %zu", n);

        for (std::size_t i = 0u; i < n; ++i) {
            const TwistMsg twist = twist_queue_->pop_increment_age();

            double delay_time = (tick_time - rclcpp::Time(twist.header.stamp)).seconds();

            if (delay_time < future_reject_threshold) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Twist obs from future (delay=%.4f s, "
                    "threshold=%.4f s) — skipping.",
                    delay_time, future_reject_threshold);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Twist obs too old (delay=%.4f s, "
                    "max tracked=%.4f s) — rejecting.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(twist, static_cast<int>(delay_step));
        }
    }

    // ── 4.5 Process relative-pose (odometry) observations ──────────────────
    // Both stamp_to and stamp_from are used to map each epoch to its own
    // ESKF delay slot independently, avoiding linearisation error from the
    // Δt_odom offset between the two frames.
    {
        const std::size_t n = rel_pose_queue_->size();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] rel-pose observations this tick: %zu", n);

        for (std::size_t i = 0u; i < n; ++i) {
            const RelativePoseObs obs = rel_pose_queue_->pop_increment_age();

            // ── Compute delay for the "to" (newer) frame ──────────────────
            double delay_to = (tick_time - obs.stamp_to).seconds();
            if (delay_to < future_reject_threshold) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] RelPose 'to' stamp from future "
                    "(delay=%.4f s, threshold=%.4f s) — skipping.",
                    delay_to, future_reject_threshold);
                continue;
            }
            delay_to = std::max(delay_to, 0.0);

            // ── Compute delay for the "from" (older) frame ────────────────
            double delay_from = (tick_time - obs.stamp_from).seconds();
            if (delay_from < future_reject_threshold) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] RelPose 'from' stamp from future "
                    "(delay=%.4f s, threshold=%.4f s) — skipping.",
                    delay_from, future_reject_threshold);
                continue;
            }
            delay_from = std::max(delay_from, 0.0);

            const std::size_t ds_to   = eskf_.findDelayStep(delay_to);
            const std::size_t ds_from = eskf_.findDelayStep(delay_from);

            if (ds_to >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] RelPose 'to' frame too old "
                    "(delay=%.4f s) — rejecting.", delay_to);
                continue;
            }
            if (ds_from >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] RelPose 'from' frame too old "
                    "(delay=%.4f s, max=%.4f s) — rejecting.",
                    delay_from, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(obs,
                        static_cast<int>(ds_to),
                        static_cast<int>(ds_from));
        }
    }

    // ── 5. Update output cache ─────────────────────────────────────────────
    {
        std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
        cached_state_      = eskf_.getState();
        cached_covariance_ = eskf_.getCovariance();
        cached_time_       = tick_time;
    }

    // ── 5b. Append to history buffer ──────────────────────────────────────
    state_buffer_->push(tick_time, eskf_.getState());

    // ── 6. Broadcast TF: map → base_link ───────────────────────────────────
    const NavStateD & s = eskf_.getState();

    if (tf_br_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = tick_time;
        tf.header.frame_id = "map";
        tf.child_frame_id  = "base_link";

        tf.transform.translation.x = s.p.x();
        tf.transform.translation.y = s.p.y();
        tf.transform.translation.z = s.p.z();

        const Eigen::Quaterniond q(s.R.matrix());
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();

        tf_br_->sendTransform(tf);
    }

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] tick  |p|=%.4f  |v|=%.4f  |ω|=%.4f",
        s.p.norm(), s.v.norm(), s.omega.norm());

    // ── 7. Overrun detection ───────────────────────────────────────────────
    const double elapsed_sec = (pnode_->now() - tick_time).seconds();
    const double period_sec  = 1.0 / params_.update_rate_hz;
    RCLCPP_INFO_THROTTLE(logger_, *pnode_->get_clock(), 2000,
        "[FineNavLocalizer] tick complete  elapsed=%.1f ms  period=%.1f ms",
        elapsed_sec * 1e3, period_sec * 1e3);

    if (elapsed_sec > period_sec) {
        RCLCPP_WARN_THROTTLE(logger_, *pnode_->get_clock(), 2000,
            "[FineNavLocalizer] Timer overrun: callback took %.1f ms but period is %.1f ms "
            "(%.0f%% over). Consider reducing update_rate_hz or max_delay_step "
            "(max_delay_sec * update_rate_hz).",
            elapsed_sec * 1e3, period_sec * 1e3,
            (elapsed_sec / period_sec - 1.0) * 100.0);
    }
}

// ============================================================================
//  applyUpdate  — Mahalanobis gate + ESKF update with delay compensation
// ============================================================================
void FineNavLocalizer::applyUpdate(const PoseMsg & msg, int delay_step)
{
    auto [pos_obs, rot_obs, R_raw] = extractObservation(msg);

    // Scale noise covariance by smoothing_steps: makes each of the N
    // processing steps less aggressive, resulting in a smooth correction
    // spread over N ticks.
    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(params_.pose_smoothing_steps);

    // Linearise at the nominal state from the time of capture, not the
    // current nominal.  This gives the correct innovation z − h(x̂_{k−d})
    // and the correct H Jacobian for the delayed KF update.
    const auto obs = NavigationModelD::observePose(
        eskf_.getStateAtDelay(delay_step), pos_obs, rot_obs);

    // ── Mahalanobis gate ────────────────────────────────────────────────────
    // Gate uses the covariance of the delay slot that the observation maps to
    // (P(d,d)), not the current leading block P(0,0).  This gives the correct
    // innovation covariance:  S = H · P(d,d) · H^T + R_eff
    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H * eskf_.getCovarianceAtDelay(delay_step) * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);

    RCLCPP_INFO_THROTTLE(logger_, *pnode_->get_clock(), 2000, "Mahalanobis distance: %.1f", d2);

    if (d2 > params_.pose_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] applyUpdate<Pose> rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, params_.pose_gate_dist);
        return;
    }

    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] applyUpdate<Pose>   delay_step=%d  d²=%.4f  "
        "|δp|=%.4f  |δR|=%.4f",
        delay_step, d2,
        obs.innovation.head<3>().norm(), obs.innovation.tail<3>().norm());
}

void FineNavLocalizer::applyUpdate(const TwistMsg & msg, int delay_step)
{
    auto [lin_vel, ang_vel, R_raw] = extractObservation(msg);

    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(params_.twist_smoothing_steps);

    // Same as pose: linearise at the historical nominal from delay_step ticks ago.
    const auto obs = NavigationModelD::observeVelocity(
        eskf_.getStateAtDelay(delay_step), lin_vel, ang_vel);

    // ── Mahalanobis gate ────────────────────────────────────────────────────
    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H * eskf_.getCovarianceAtDelay(delay_step) * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);

    if (d2 > params_.twist_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] applyUpdate<Twist> rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, params_.twist_gate_dist);
        return;
    }

    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] applyUpdate<Twist>  delay_step=%d  d²=%.4f  "
        "|δv|=%.4f  |δω|=%.4f",
        delay_step, d2,
        obs.innovation.head<3>().norm(), obs.innovation.tail<3>().norm());
}

void FineNavLocalizer::applyUpdate(const RelativePoseObs & obs,
                                   int                     delay_step_to,
                                   int                     delay_step_from)
{
    const Eigen::Matrix<double, 6, 6> R_eff =
        obs.R_noise * static_cast<double>(params_.rel_pose_smoothing_steps);

    // Retrieve both nominal states from the ESKF using their respective
    // delay slots.  Using the correct historical slot for the "from" epoch
    // is the key fix vs. the naive approach of using only the "to" stamp.
    const NavStateD from_state = eskf_.getStateAtDelay(delay_step_from);
    const NavStateD to_state   = eskf_.getStateAtDelay(delay_step_to);

    const auto dual_obs = NavigationModelD::observeRelativePose(
        from_state, to_state, obs.delta_p_body_obs, obs.delta_R_obs);

    // ── Mahalanobis gate ─────────────────────────────────────────────────────
    // Conservative: diagonal-block approximation (ignores cross-cov P(to,from)).
    // Full S = H_to P(to,to) H_to^T + H_to P(to,from) H_from^T
    //        + H_from P(from,to) H_to^T + H_from P(from,from) H_from^T + R
    const Eigen::Matrix<double, 6, 6> S_approx =
        dual_obs.H_to   * eskf_.getCovarianceAtDelay(delay_step_to)   * dual_obs.H_to.transpose()
      + dual_obs.H_from * eskf_.getCovarianceAtDelay(delay_step_from) * dual_obs.H_from.transpose()
      + R_eff;

    const double d2 = static_cast<double>(
        dual_obs.innovation.transpose() * S_approx.inverse() * dual_obs.innovation);

    if (d2 > params_.rel_pose_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] applyUpdate<RelPose> rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, params_.rel_pose_gate_dist);
        return;
    }

    eskf_.update<6>(dual_obs.innovation,
                    dual_obs.H_to,
                    dual_obs.H_from,
                    R_eff,
                    delay_step_to,
                    delay_step_from);

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] applyUpdate<RelPose>  ds_to=%d  ds_from=%d  "
        "d²=%.4f  |δΔp|=%.4f  |δΔR|=%.4f",
        delay_step_to, delay_step_from, d2,
        dual_obs.innovation.head<3>().norm(),
        dual_obs.innovation.tail<3>().norm());
}

// ============================================================================
//  extractObservation  —  message → filter-ready quantities
// ============================================================================
std::tuple<FineNavLocalizer::Vec3d,
           FineNavLocalizer::SO3d,
           Eigen::Matrix<double, 6, 6>>
FineNavLocalizer::extractObservation(const PoseMsg & msg)
{
    Vec3d pos{
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z};

    const Eigen::Quaterniond q{
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z};
    const SO3d rot{q.normalized().toRotationMatrix()};

    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.pose.covariance[static_cast<std::size_t>(r * 6 + c)];

    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-6);

    return {pos, rot, R};
}

std::tuple<FineNavLocalizer::Vec3d,
           FineNavLocalizer::Vec3d,
           Eigen::Matrix<double, 6, 6>>
FineNavLocalizer::extractObservation(const TwistMsg & msg)
{
    Vec3d lin{
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z};

    Vec3d ang{
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z};

    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.twist.covariance[static_cast<std::size_t>(r * 6 + c)];

    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-6);

    return {lin, ang, R};
}

}  // namespace finenav


