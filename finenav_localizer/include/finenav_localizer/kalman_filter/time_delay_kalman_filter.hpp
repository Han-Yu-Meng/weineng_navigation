// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_localizer/kalman_filter/kalman_filter.hpp"

#include <iostream>
#include <utility>

namespace finenav {

// ============================================================================
//  TimeDelayKalmanFilter  —  KF with state-expansion for delayed observations
//
//  Extends KalmanFilter by maintaining an augmented state vector:
//
//      x_ex = [ x_k | x_{k-1} | ... | x_{k-d+1} ]   (dim_x * max_delay_step)
//
//  The expanded state transition matrix has the structure:
//
//      A_ex = [ A   0   0 … ]      Q_ex = [ Q   0   0 … ]
//             [ I   0   0 … ]             [ 0   0   0 … ]
//             [ 0   I   0 … ]             [ 0   0   0 … ]
//             [ …   …   … … ]             [ …   …   … … ]
//
//  An observation arriving with delay_step d is associated with x_{k-d} and
//  uses the measurement matrix:
//
//      C_ex = [ 0 | … | C | … | 0 ]   (C at column block d)
//
//  This lets the standard KF update simultaneously correct x_k (via the
//  cross-covariance blocks) and x_{k-d} (directly).
//
//  ── Usage ─────────────────────────────────────────────────────────────────
//  1. Call init(x0, P0, max_delay_step) once.
//  2. Every filter tick:  predictWithDelay(x_next, F, Q)
//  3. For each observation with known delay_step:
//       updateWithDelay(y, C, R, delay_step)
//  4. getLatestX() / getLatestP() return the leading (current-time) block.
//  5. After ESKF error-state reset, call resetState() to zero x_ex.
//  6. After ESKF covariance projection, call setLeadingCovariance(P_proj).
// ============================================================================
class TimeDelayKalmanFilter : public KalmanFilter {
public:
    TimeDelayKalmanFilter() = default;

    // ---- Initialization ---------------------------------------------------

    /// @param x              Initial state vector        (dim_x × 1)
    /// @param P0             Initial covariance           (dim_x × dim_x)
    /// @param max_delay_step Maximum number of delay slots (≥ 1)
    ///
    /// All delay slots are seeded with the same (x, P0) so that the filter
    /// is consistent from the first tick.
    void init(const Eigen::MatrixXd & x,
              const Eigen::MatrixXd & P0,
              int                    max_delay_step)
    {
        max_delay_step_ = max_delay_step;
        dim_x_          = static_cast<int>(x.rows());
        dim_x_ex_       = dim_x_ * max_delay_step_;

        // Fill all delay slots with the same initial state / covariance.
        x_ = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
        P_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

        for (int i = 0; i < max_delay_step_; ++i) {
            x_.block(i * dim_x_, 0,          dim_x_, 1)      = x;
            P_.block(i * dim_x_, i * dim_x_, dim_x_, dim_x_) = P0;
        }
    }

    // ---- Accessors for the leading (current-time) block ------------------
    [[nodiscard]] Eigen::MatrixXd getLatestX() const
    {
        return x_.block(0, 0, dim_x_, 1);
    }

    [[nodiscard]] Eigen::MatrixXd getLatestP() const
    {
        return P_.block(0, 0, dim_x_, dim_x_);
    }

    // ---- ESKF helpers ----------------------------------------------------

    /// Zero out the entire extended error-state vector.
    /// Called after ESKF applies the error correction to the nominal state.
    void resetState()
    {
        x_.setZero();
    }

    /// Overwrite the leading (0,0) covariance block only.
    /// Called after ESKF covariance projection (P ← G·P·Gᵀ).
    void setLeadingCovariance(const Eigen::MatrixXd & P_new)
    {
        P_.block(0, 0, dim_x_, dim_x_) = P_new;
    }

    /// Read the error-state vector at a specific delay slot.
    /// slot 0 = current time (same as getLatestX()), slot d = d ticks ago.
    [[nodiscard]] Eigen::MatrixXd getXAtSlot(int slot) const
    {
        return x_.block(slot * dim_x_, 0, dim_x_, 1);
    }

    /// Read the covariance block at a specific delay slot.
    /// slot 0 = current time (same as getLatestP()), slot d = d ticks ago.
    /// Returns P(d,d) — the covariance of the error-state at that delay slot.
    /// Clamps to the last valid slot if slot >= max_delay_step_.
    [[nodiscard]] Eigen::MatrixXd getPAtSlot(int slot) const
    {
        const int s = std::min(slot, max_delay_step_ - 1);
        return P_.block(s * dim_x_, s * dim_x_, dim_x_, dim_x_);
    }

    // ---- Time-delay predict ----------------------------------------------

    /// Slide all slots one step forward in time, then update covariance.
    ///
    /// Extended state slide:
    ///   x_ex ← [ x_next | x_k | x_{k-1} | … | x_{k-d+2} ]
    ///           (oldest slot x_{k-d+1} is discarded)
    ///
    /// Extended covariance (structured propagation, see class comment):
    ///   P(0,0) ← A·P(0,0)·Aᵀ + Q
    ///   P(0,j) ← A·P(0,j-1)     for j ≥ 1
    ///   P(i,0) ← P(i-1,0)·Aᵀ   for i ≥ 1
    ///   P(i,j) ← P(i-1,j-1)     for i,j ≥ 1
    ///
    /// @param x_next  Next nominal error state (zero in ESKF, since error
    ///                state is always reset after every update).
    /// @param A       Error-state transition Jacobian F  (dim_x × dim_x).
    /// @param Q       Process-noise covariance           (dim_x × dim_x).
    bool predictWithDelay(const Eigen::MatrixXd & x_next,
                          const Eigen::MatrixXd & A,
                          const Eigen::MatrixXd & Q)
    {
        const int d_dim_x = dim_x_ex_ - dim_x_;  // (max_delay_step - 1) * dim_x

        // ── Slide state vector ──────────────────────────────────────────────
        // x_tmp = [x_next; x_[0 .. d_dim_x-1]]
        Eigen::MatrixXd x_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
        x_tmp.block(0,      0, dim_x_,  1) = x_next;
        x_tmp.block(dim_x_, 0, d_dim_x, 1) = x_.block(0, 0, d_dim_x, 1);
        x_ = std::move(x_tmp);

        // ── Update extended covariance ─────────────────────────────────────
        //
        //  Symbolically (using block notation [i,j]):
        //    P_tmp[0,0] = A * P[0,0] * A' + Q
        //    P_tmp[0, 1..] = A * P[0, 0..d-2]    (first row of new cross-cov)
        //    P_tmp[1.., 0] = P[0..d-2, 0] * A'   (first col of new cross-cov)
        //    P_tmp[1.., 1..] = P[0..d-2, 0..d-2] (inner block = old leading sub)
        //
        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

        P_tmp.block(0,      0,      dim_x_,  dim_x_)  =
            A * P_.block(0, 0, dim_x_, dim_x_) * A.transpose() + Q;

        P_tmp.block(0,      dim_x_, dim_x_,  d_dim_x) =
            A * P_.block(0, 0, dim_x_, d_dim_x);

        P_tmp.block(dim_x_, 0,      d_dim_x, dim_x_)  =
            P_.block(0, 0, d_dim_x, dim_x_) * A.transpose();

        P_tmp.block(dim_x_, dim_x_, d_dim_x, d_dim_x) =
            P_.block(0, 0, d_dim_x, d_dim_x);

        P_ = std::move(P_tmp);
        return true;
    }

    // ---- Dual-slot measurement update ------------------------------------

    /// Apply a measurement whose observation function spans two state epochs:
    ///   C_slot_a acts on error state at slot_a  (e.g. the "to"   epoch)
    ///   C_slot_b acts on error state at slot_b  (e.g. the "from" epoch)
    ///
    /// Builds the full expanded C_ex with non-zero blocks at both slots,
    /// then calls the standard KF update.  When slot_a == slot_b the
    /// two blocks are summed at the same column (correct degenerate case).
    ///
    /// @param y       Measurement vector          (dim_y × 1)
    /// @param C_a     Measurement Jacobian for slot_a  (dim_y × dim_x)
    /// @param slot_a  Delay slot for C_a  (0 = current tick)
    /// @param C_b     Measurement Jacobian for slot_b  (dim_y × dim_x)
    /// @param slot_b  Delay slot for C_b
    /// @param R       Measurement noise cov.     (dim_y × dim_y)
    bool updateWithTwoSlots(const Eigen::MatrixXd & y,
                            const Eigen::MatrixXd & C_a,
                            int                    slot_a,
                            const Eigen::MatrixXd & C_b,
                            int                    slot_b,
                            const Eigen::MatrixXd & R)
    {
        if (slot_a >= max_delay_step_ || slot_b >= max_delay_step_) {
            std::cerr << "[TimeDelayKF] updateWithTwoSlots: slot " << slot_a
                      << " or " << slot_b
                      << " >= max_delay_step=" << max_delay_step_ << " — ignoring.\n";
            return false;
        }
        const int dim_y = static_cast<int>(y.rows());
        Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
        C_ex.block(0, dim_x_ * slot_a, dim_y, dim_x_) += C_a;
        C_ex.block(0, dim_x_ * slot_b, dim_y, dim_x_) += C_b;
        return update(y, C_ex * x_, C_ex, R);
    }

    // ---- Delayed measurement update ---------------------------------------

    /// Apply a measurement that was captured delay_step ticks ago.
    ///
    /// Internally builds an expanded measurement matrix:
    ///   C_ex = [ 0 | … | C | … | 0 ]   (C at column block delay_step)
    ///
    /// The standard KF update then simultaneously corrects:
    ///   • x_{k-delay_step}  directly (via C)
    ///   • x_k, x_{k-1}, …  indirectly (via cross-covariance blocks)
    ///
    /// @param y           Measurement vector        (dim_y × 1)
    /// @param C           Measurement Jacobian      (dim_y × dim_x)
    /// @param R           Measurement noise cov.    (dim_y × dim_y)
    /// @param delay_step  0 = no delay (current tick), 1 = one tick old, …
    bool updateWithDelay(const Eigen::MatrixXd & y,
                         const Eigen::MatrixXd & C,
                         const Eigen::MatrixXd & R,
                         int                    delay_step)
    {
        if (delay_step >= max_delay_step_) {
            std::cerr << "[TimeDelayKF] delay_step=" << delay_step
                      << " >= max_delay_step=" << max_delay_step_
                      << " — ignoring update.\n";
            return false;
        }

        const int dim_y = static_cast<int>(y.rows());

        // Build expanded C: C_ex[:, delay_step*dim_x : (delay_step+1)*dim_x] = C
        Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
        C_ex.block(0, dim_x_ * delay_step, dim_y, dim_x_) = C;

        // y_pred = C_ex * x_ (note: in ESKF x_ is always ≈ 0)
        return update(y, C_ex * x_, C_ex, R);
    }

    // ---- Introspection ----------------------------------------------------
    [[nodiscard]] int maxDelayStep() const { return max_delay_step_; }
    [[nodiscard]] int dimX()         const { return dim_x_; }
    [[nodiscard]] int dimXEx()       const { return dim_x_ex_; }

private:
    int max_delay_step_ = 1;
    int dim_x_          = 0;
    int dim_x_ex_       = 0;
};

}  // namespace finenav

