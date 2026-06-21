// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>

namespace finenav {

// ============================================================================
//  KalmanFilter  —  standard linear discrete-time KF (dynamic matrices)
//
//  Prediction:  x' = x_next,   P' = A·P·Aᵀ + Q
//  Update:      K  = P·Cᵀ·(C·P·Cᵀ + R)⁻¹
//               x' = x + K·(y − y_pred)
//               P' = (I − K·C)·P·(I − K·C)ᵀ + K·R·Kᵀ   (Joseph form)
//
//  All matrices use double (Eigen::MatrixXd).  The class is header-only.
// ============================================================================
class KalmanFilter {
public:
    KalmanFilter()          = default;
    virtual ~KalmanFilter() = default;

    // ---- Initialization ---------------------------------------------------

    /// Minimal init: only x₀ and P₀ (A/B/C/Q/R must be set separately if needed).
    bool init(const Eigen::MatrixXd & x, const Eigen::MatrixXd & P0)
    {
        if (x.size() == 0 || P0.size() == 0) { return false; }
        x_ = x;
        P_ = P0;
        return true;
    }

    // ---- Prediction step -------------------------------------------------

    /// @param x_next  Pre-computed next state  (caller's responsibility to
    ///                apply the non-linear model before calling predict).
    /// @param A       State-transition Jacobian  (dim_x × dim_x).
    /// @param Q       Process-noise covariance   (dim_x × dim_x).
    bool predict(const Eigen::MatrixXd & x_next,
                 const Eigen::MatrixXd & A,
                 const Eigen::MatrixXd & Q)
    {
        if (x_.rows() != x_next.rows() ||
            A.cols()  != P_.rows()     ||
            Q.rows()  != Q.cols()      ||
            A.rows()  != Q.cols()) {
            return false;
        }
        x_ = x_next;
        P_ = A * P_ * A.transpose() + Q;
        return true;
    }

    // ---- Update step ------------------------------------------------------

    /// Full update with explicit prediction y_pred (useful when h(x) is
    /// non-linear — caller linearises and passes y_pred = h(x̂)).
    /// Uses Joseph form for numerical stability.
    bool update(const Eigen::MatrixXd & y,
                const Eigen::MatrixXd & y_pred,
                const Eigen::MatrixXd & C,
                const Eigen::MatrixXd & R)
    {
        if (P_.cols() != C.cols() ||
            R.rows() != R.cols()  ||
            R.rows() != C.rows()  ||
            y.rows() != y_pred.rows() ||
            y.rows() != C.rows()) {
            return false;
        }

        const Eigen::MatrixXd PCT = P_ * C.transpose();
        const Eigen::MatrixXd K   = PCT * (R + C * PCT).inverse();

        if (K.array().isNaN().any() || K.array().isInf().any()) {
            return false;
        }

        x_ = x_ + K * (y - y_pred);

        // Joseph form: P = (I-KC)P(I-KC)' + KRK'
        const Eigen::MatrixXd I_KC =
            Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) - K * C;
        P_ = I_KC * P_ * I_KC.transpose() + K * R * K.transpose();

        return true;
    }

    /// Simplified update: y_pred = C·x̂  (linear observation model).
    bool update(const Eigen::MatrixXd & y,
                const Eigen::MatrixXd & C,
                const Eigen::MatrixXd & R)
    {
        if (C.cols() != x_.rows()) { return false; }
        return update(y, C * x_, C, R);
    }

    // ---- Accessors --------------------------------------------------------
    [[nodiscard]] Eigen::MatrixXd getX() const { return x_; }
    [[nodiscard]] Eigen::MatrixXd getP() const { return P_; }
    [[nodiscard]] double getXelement(int i) const { return x_(i); }

protected:
    Eigen::MatrixXd x_;   ///< state vector  (dim_x × 1)
    Eigen::MatrixXd P_;   ///< covariance    (dim_x × dim_x)
};

}  // namespace finenav

