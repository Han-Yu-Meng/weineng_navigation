// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <algorithm>
#include <deque>
#include <vector>

#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <rclcpp/logging.hpp>

#include "finenav_localizer/filter_model.hpp"
#include "finenav_localizer/kalman_filter/time_delay_kalman_filter.hpp"

namespace finenav {

// ============================================================================
//  ESKF<Model>  —  Error-State Kalman Filter with time-delay compensation
//
//  Internally, the error-state covariance is maintained inside a
//  TimeDelayKalmanFilter, which expands the state to:
//
//      x_ex = [ δx_k | δx_{k-1} | … | δx_{k-d+1} ]
//
//  This lets delayed observations update the covariance and nominal state
//  correctly via cross-covariance blocks, without storing past nominal states.
//
//  ── Template parameter `Model` must expose ─────────────────────────────────
//    typename Model::ScalarT          —  scalar type (float / double)
//    typename Model::NavStateT        —  nominal state struct
//    typename Model::ErrorVec         —  error-state column vector (kNavStateDim × 1)
//    typename Model::FMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::QMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::PMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::Options          —  noise-parameter struct
//
//    Model::predictNominal(s, dt)     —  propagate nominal state forward
//    Model::computeF(s, dt)           —  error-state transition Jacobian F
//    Model::buildQ(opts, dt)          —  discrete process-noise matrix Q
//    Model::applyErrorState(s, dx)    —  nominal ← nominal ⊕ dx
//    Model::projectCovariance(P, dx)  —  ESKF covariance reset projection
//
//  ── Delay-handling design ──────────────────────────────────────────────────
//  • Every predict(dt) call:
//      1. Saves the PRE-propagation nominal into nominal_history_ (pushed to front).
//         nominal_history_[0] = nominal one tick ago  (delay_step 1)
//         nominal_history_[d-1] = nominal d ticks ago (delay_step d)
//      2. Propagates the nominal state forward.
//      3. Slides the TimeDelayKF extended covariance and accumulates delay times.
//
//  • findDelayStep(delay_sec) converts a wall-clock delay to the closest
//    integer delay slot index to pass to update(..., delay_step).
//
//  • getStateAtDelay(d) returns nominal_history_[d-1], which is the nominal
//    state at the time of capture of a delay_step=d observation.
//    This is critical for computing correct innovations and Jacobians.
//
//  • update<ObsDim>(..., delay_step) calls updateWithDelay internally.
//    After the KF update:
//      1. dx_0 ← getLatestX()  (block 0 = current-time error correction)
//      2. nominal ← applyErrorState(nominal, dx_0)
//      3. For each historical slot i ≥ 1:
//           dx_i ← getXAtSlot(i)  (past error correction from cross-covariance)
//           nominal_history_[i−1] ← applyErrorState(history[i−1], dx_i)
//         This keeps the nominal history consistent with the TDKF posterior,
//         so future delayed observations compute correct innovations.
//      4. P(0,0) ← projectCovariance(P(0,0), dx_0)   [leading block only]
//      5. resetState() — zero all error-state slots in tdkf_
//
//  ── Thread safety ──────────────────────────────────────────────────────────
//  Not thread-safe.  All calls must be serialized by the owner
//  (FineNavLocalizer's timerCallback holds filter_mutex_).
//
//  This file is header-only (templates).
// ============================================================================
template <typename Model>
class ESKF {
public:
    using S         = typename Model::ScalarT;
    using NavStateT = typename Model::NavStateT;
    using ErrorVec  = typename Model::ErrorVec;
    using FMat      = typename Model::FMat;
    using QMat      = typename Model::QMat;
    using PMat      = typename Model::PMat;

    static constexpr int kDim = kNavStateDim;  ///< 12

    // ---- Construction -----------------------------------------------------

    /// @param model_opts      Process-noise parameters forwarded to Model::buildQ.
    /// @param max_delay_step  Number of delay slots (≥ 1).
    ///                        Typically ceil(max_delay_sec * update_rate_hz).
    ///                        Observations older than this many ticks are rejected.
    /// @param init_cov        Initial state covariance P₀.
    explicit ESKF(typename Model::Options model_opts   = {},
                  int                    max_delay_step = 50,
                  PMat                   init_cov       = PMat::Identity() * static_cast<S>(1e-2))
        : model_opts_(std::move(model_opts))
        , accumulated_delay_times_(static_cast<std::size_t>(max_delay_step), 1.0e15)
        , logger_(rclcpp::get_logger("finenav.ESKF"))
    {
        const Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.init(x0, init_cov.template cast<double>(), max_delay_step);
    }

    // ---- Predict step -----------------------------------------------------
    //
    //  Nominal-state propagation + error-state covariance propagation.
    //
    //  Since the error state is always reset to zero after every update(),
    //  x_next passed to predictWithDelay is the zero vector.  Only the
    //  covariance (and its cross-correlation blocks) advances in time.
    //
    //  After predict(), accumulateDelayTime(dt) updates accumulated_delay_times_
    //  so that findDelayStep() returns meaningful indices.
    void predict(S dt)
    {
        if (dt <= static_cast<S>(0.0)) {
            RCLCPP_WARN(logger_,
                "[ESKF] predict() called with non-positive dt=%.6f — skipping.",
                static_cast<double>(dt));
            return;
        }

        // Compute F and Q from the current nominal state *before* propagating.
        const FMat F = Model::computeF(nominal_, static_cast<S>(dt));
        const QMat Q = Model::buildQ(model_opts_, static_cast<S>(dt));

        // ── Save nominal into history BEFORE propagation ────────────────────
        //  nominal_history_[0] will become delay_step=1 (one tick ago).
        //  The oldest slot beyond max_delay_step is discarded.
        nominal_history_.push_front(nominal_);
        if (static_cast<int>(nominal_history_.size()) >= tdkf_.maxDelayStep()) {
            nominal_history_.pop_back();
        }

        // Propagate nominal state.
        nominal_ = Model::predictNominal(nominal_, static_cast<S>(dt));

        // Propagate error-state covariance via time-delay extended structure.
        // x_next is zero: error state is always reset after each update().
        const Eigen::MatrixXd x_next = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.predictWithDelay(x_next,
                               F.template cast<double>(),
                               Q.template cast<double>());

        // Track elapsed time in each delay slot.
        accumulateDelayTime(static_cast<double>(dt));

        RCLCPP_DEBUG(logger_,
            "[ESKF] predict  dt=%.4f  |p|=%.3f  |v|=%.3f  |ω|=%.3f",
            static_cast<double>(dt),
            static_cast<double>(nominal_.p.norm()),
            static_cast<double>(nominal_.v.norm()),
            static_cast<double>(nominal_.omega.norm()));
    }

    // ---- Dual-epoch observation update  (relative-pose / odometry) --------
    //
    //  For a measurement whose observation function spans TWO state epochs
    //  (e.g. odometry: z = T_from^{-1} · T_to), the Jacobian decomposes into
    //  two blocks:
    //    H_to   — ∂z/∂δx at the "to"   epoch (delay_step_to   ticks ago)
    //    H_from — ∂z/∂δx at the "from" epoch (delay_step_from ticks ago)
    //
    //  Both blocks are placed in the extended C_ex passed to the TDKF, so the
    //  cross-covariance P(slot_to, slot_from) is naturally exploited.
    //
    //  @param innovation     z − h(x̂)                        (ObsDim × 1)
    //  @param H_to           Jacobian for the "to"   epoch    (ObsDim × kDim)
    //  @param H_from         Jacobian for the "from" epoch    (ObsDim × kDim)
    //  @param R_noise        Observation-noise covariance     (ObsDim × ObsDim)
    //  @param delay_step_to  Delay slot for the "to"   frame  (≥ 0)
    //  @param delay_step_from Delay slot for the "from" frame (≥ delay_step_to)
    template <int ObsDim>
    bool update(const Eigen::Matrix<S, ObsDim, 1>      & innovation,
                const Eigen::Matrix<S, ObsDim, kDim>   & H_to,
                const Eigen::Matrix<S, ObsDim, kDim>   & H_from,
                const Eigen::Matrix<S, ObsDim, ObsDim> & R_noise,
                int                                      delay_step_to,
                int                                      delay_step_from)
    {
        // ── 1. Dual-slot KF update ──────────────────────────────────────────
        if (!tdkf_.updateWithTwoSlots(
                innovation.template cast<double>(),
                H_to.template cast<double>(),   delay_step_to,
                H_from.template cast<double>(), delay_step_from,
                R_noise.template cast<double>())) {
            return false;
        }

        // ─��� 2–5. Identical post-update logic as single-block update ─────────
        const ErrorVec dx     = tdkf_.getLatestX().template cast<S>();
        const S        dx_norm = dx.norm();

        nominal_ = Model::applyErrorState(nominal_, dx);

        const int n_hist = std::min(
            tdkf_.maxDelayStep() - 1,
            static_cast<int>(nominal_history_.size()));
        for (int i = 0; i < n_hist; ++i) {
            const ErrorVec dx_i = tdkf_.getXAtSlot(i + 1).template cast<S>();
            if (dx_i.squaredNorm() > static_cast<S>(1e-20)) {
                nominal_history_[static_cast<std::size_t>(i)] =
                    Model::applyErrorState(
                        nominal_history_[static_cast<std::size_t>(i)], dx_i);
            }
        }

        const PMat P_lead      = tdkf_.getLatestP().template cast<S>();
        const PMat P_projected = Model::projectCovariance(P_lead, dx);
        tdkf_.setLeadingCovariance(P_projected.template cast<double>());
        tdkf_.resetState();

        RCLCPP_DEBUG(logger_,
            "[ESKF] dual update  ObsDim=%d  slot_to=%d  slot_from=%d  "
            "|innov|=%.4f  |dx|=%.4f",
            ObsDim, delay_step_to, delay_step_from,
            static_cast<double>(innovation.norm()),
            static_cast<double>(dx_norm));
        return true;
    }

    // ---- Generic observation update  (compile-time observation dimension) -
    //
    //  @tparam ObsDim    Observation vector length.
    //  @param innovation  z − h(x̂)                       (ObsDim × 1)
    //  @param H           Observation Jacobian             (ObsDim × kDim)
    //  @param R_noise     Observation-noise covariance     (ObsDim × ObsDim)
    //  @param delay_step  Delay slot (0 = no delay, from findDelayStep()).
    //                     Default 0 is backward-compatible with callers that
    //                     do not yet pass a delay.
    //
    //  Returns false if updateWithDelay rejects the observation (e.g.
    //  delay_step ≥ max_delay_step) or if the KF gain is degenerate.
    template <int ObsDim>
    bool update(const Eigen::Matrix<S, ObsDim, 1>      & innovation,
                const Eigen::Matrix<S, ObsDim, kDim>   & H,
                const Eigen::Matrix<S, ObsDim, ObsDim> & R_noise,
                int                                      delay_step = 0)
    {
        // ── 1. Delayed KF update ────────────────────────────────────────────
        if (!tdkf_.updateWithDelay(innovation.template cast<double>(),
                                   H.template cast<double>(),
                                   R_noise.template cast<double>(),
                                   delay_step)) {
            return false;
        }

        // ── 2. Extract error-state correction from the leading block ────────
        const ErrorVec dx = tdkf_.getLatestX().template cast<S>();
        const S dx_norm   = dx.norm();  // capture for debug log

        // ── 3. Apply error corrections to ALL nominal states ────────────────
        //
        //  dx_0  (slot 0): correction for current nominal.
        //  dx_i  (slot i, i ≥ 1): correction for nominal_history_[i−1].
        //    These arise from the cross-covariance blocks that propagate the
        //    measurement's information back through the delay.  Applying them
        //    keeps the history consistent with the TDKF posterior, so that a
        //    future observation at the same delay slot sees the correct
        //    innovation (z − h(corrected_historical_nominal)) and does not
        //    receive an inflated correction from a stale linearisation point.
        nominal_ = Model::applyErrorState(nominal_, dx);

        const int n_hist = std::min(
            tdkf_.maxDelayStep() - 1,
            static_cast<int>(nominal_history_.size()));
        for (int i = 0; i < n_hist; ++i) {
            // TDKF slot index (i+1) ↔ nominal_history_[i]
            const ErrorVec dx_i =
                tdkf_.getXAtSlot(i + 1).template cast<S>();
            // Only bother if the correction is non-trivial (avoids unnecessary
            // SO3 exponentials when the cross-covariance contribution is zero).
            if (dx_i.squaredNorm() > static_cast<S>(1e-20)) {
                nominal_history_[static_cast<std::size_t>(i)] =
                    Model::applyErrorState(nominal_history_[static_cast<std::size_t>(i)],
                                          dx_i);
            }
        }

        // ── 4. ESKF covariance projection on the leading block ──────────────
        //  P(0,0) ← G · P(0,0) · Gᵀ   where G = I − 0.5·[δθ]×  (SO3 block)
        //  Cross-correlation blocks P(i,0) / P(0,j) for i,j ≥ 1 are not
        //  explicitly projected because G ≈ I for small δθ (second-order).
        const PMat P_lead      = tdkf_.getLatestP().template cast<S>();
        const PMat P_projected = Model::projectCovariance(P_lead, dx);
        tdkf_.setLeadingCovariance(P_projected.template cast<double>());

        // ── 5. Reset all error-state slots to zero (ESKF reset step) ────────
        tdkf_.resetState();

        RCLCPP_DEBUG(logger_,
            "[ESKF] update  ObsDim=%d  delay_step=%d  |innov|=%.4f  |dx|=%.4f",
            ObsDim, delay_step,
            static_cast<double>(innovation.norm()),
            static_cast<double>(dx_norm));
        return true;
    }

    // ---- Delay-step helpers (mirrors autoware EKFModule) ------------------

    /// Find the delay-slot index whose accumulated time is closest to
    /// delay_time_sec.  Returns accumulated_delay_times_.size() if
    /// delay_time_sec exceeds the maximum tracked delay (caller should
    /// reject the observation).
    [[nodiscard]] std::size_t findDelayStep(double delay_time_sec) const
    {
        // If larger than the oldest slot, signal rejection.
        if (delay_time_sec > accumulated_delay_times_.back()) {
            return accumulated_delay_times_.size();
        }

        auto lower = std::lower_bound(
            accumulated_delay_times_.begin(),
            accumulated_delay_times_.end(),
            delay_time_sec);

        if (lower == accumulated_delay_times_.begin()) {
            return 0u;
        }
        if (lower == accumulated_delay_times_.end()) {
            return accumulated_delay_times_.size() - 1u;
        }

        // Pick whichever neighbour is closer.
        const auto prev = lower - 1;
        const bool closer_to_prev =
            (delay_time_sec - *prev) < (*lower - delay_time_sec);

        return closer_to_prev
            ? static_cast<std::size_t>(std::distance(accumulated_delay_times_.begin(), prev))
            : static_cast<std::size_t>(std::distance(accumulated_delay_times_.begin(), lower));
    }

    /// Maximum delay time currently tracked (seconds).
    /// Observations older than this should be discarded.
    [[nodiscard]] double maxTrackedDelaySec() const
    {
        return accumulated_delay_times_.empty() ? 0.0 : accumulated_delay_times_.back();
    }

    // ---- Accessors --------------------------------------------------------
    [[nodiscard]] const NavStateT & getState() const { return nominal_; }

    /// Get the nominal state at a specific delay step.
    ///   delay_step = 0  →  current nominal (same as getState())
    ///   delay_step = d  →  nominal from d ticks ago (= nominal_history_[d−1])
    ///
    /// Use this to compute the innovation for a delayed observation:
    ///   innov = z − h(getStateAtDelay(delay_step))
    ///
    /// Falls back to the oldest available entry if d exceeds history depth.
    [[nodiscard]] const NavStateT & getStateAtDelay(int delay_step) const
    {
        if (delay_step <= 0 || nominal_history_.empty()) {
            return nominal_;
        }
        const auto idx = static_cast<std::size_t>(delay_step - 1);
        if (idx < nominal_history_.size()) {
            return nominal_history_[idx];
        }
        return nominal_history_.back();  // oldest available
    }

    /// Returns the leading block of the extended covariance — i.e. the
    /// covariance of the CURRENT (latest) error-state estimate.
    [[nodiscard]] PMat getCovariance() const
    {
        return tdkf_.getLatestP().template cast<S>();
    }

    /// Returns the covariance block for a specific delay slot.
    ///   delay_step = 0  →  current covariance (same as getCovariance())
    ///   delay_step = d  →  P(d,d): covariance of the error-state d ticks ago
    ///
    /// Use this for a correct Mahalanobis gate on a delayed observation:
    ///   S = H · getCovarianceAtDelay(delay_step) · H^T + R
    [[nodiscard]] PMat getCovarianceAtDelay(int delay_step) const
    {
        return tdkf_.getPAtSlot(delay_step).template cast<S>();
    }

    // ---- Mutators ---------------------------------------------------------

    void setState(const NavStateT & state) { nominal_ = state; }
    void setState(NavStateT       && state) { nominal_ = std::move(state); }

    /// Overwrites the full extended covariance (all delay slots) with P.
    /// Also resets accumulated_delay_times_ to 1e15 and clears nominal history.
    /// Use this for a full initialisation / reinit after setInitialPose.
    void setCovariance(const PMat & P)
    {
        const Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.init(x0, P.template cast<double>(), tdkf_.maxDelayStep());
        accumulated_delay_times_.assign(
            static_cast<std::size_t>(tdkf_.maxDelayStep()), 1.0e15);
        nominal_history_.clear();
    }

    /// Convenience: reset nominal state AND full extended covariance atomically.
    void reinitialize(const NavStateT & state, const PMat & P)
    {
        nominal_ = state;
        setCovariance(P);  // also clears nominal_history_
    }

    [[nodiscard]] const typename Model::Options & getModelOptions() const { return model_opts_; }
    void setModelOptions(const typename Model::Options & opts)             { model_opts_ = opts; }

    [[nodiscard]] int maxDelayStep() const { return tdkf_.maxDelayStep(); }

private:
    // ---- Internal helpers -------------------------------------------------

    /// Shift accumulated_delay_times_ one slot to the right, insert 0.0 at
    /// slot 0, and add dt to all existing slots.  Mirrors autoware's
    /// EKFModule::accumulate_delay_time().
    void accumulateDelayTime(double dt)
    {
        std::copy_backward(
            accumulated_delay_times_.begin(),
            accumulated_delay_times_.end() - 1,
            accumulated_delay_times_.end());

        accumulated_delay_times_.front() = 0.0;
        for (std::size_t i = 1u; i < accumulated_delay_times_.size(); ++i) {
            accumulated_delay_times_[i] += dt;
        }
    }

    // ---- Data members -----------------------------------------------------
    NavStateT               nominal_{};        ///< nominal (best-estimate) state
    /// Circular history of pre-propagation nominal states.
    /// nominal_history_[0] = nominal one tick ago  (delay_step = 1)
    /// nominal_history_[d-1] = nominal d ticks ago (delay_step = d)
    /// Size is bounded to maxDelayStep()-1 by predict().
    std::deque<NavStateT, Eigen::aligned_allocator<NavStateT>>   nominal_history_;
    TimeDelayKalmanFilter   tdkf_;             ///< extended error-state covariance
    typename Model::Options model_opts_;       ///< process-noise parameters
    std::vector<double>     accumulated_delay_times_;  ///< slot → elapsed time [s]
    rclcpp::Logger          logger_;
};

// ---- Convenience aliases --------------------------------------------------
using ESKFd = ESKF<NavigationModel<double>>;
using ESKFf = ESKF<NavigationModel<float>>;

}  // namespace finenav

