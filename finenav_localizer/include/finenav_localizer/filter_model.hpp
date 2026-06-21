// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace finenav {

// ============================================================================
//  Dimension of the error-state vector: [δp(3) | δθ(3) | δv(3) | δω(3)]
// ============================================================================
inline constexpr int kNavStateDim = 12;

// ============================================================================
//  NavState<S>  —  nominal state for the [p, R, v, ω] navigation model
//
//  All quantities are expressed in the *world* frame unless noted otherwise.
//  Angular velocity ω is the body-frame angular rate (same convention as IMU).
// ============================================================================
template <typename S = double>
struct NavState {
    // Sophus::SO3<S> internally holds an Eigen::Quaternion<S> which may
    // require 16-byte (SSE) or 32-byte (AVX) alignment.  When NavState is
    // stored in heap containers (std::deque, std::vector) the allocator must
    // honour this requirement; the macro below ensures it does.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SO3T  = Sophus::SO3<S>;
    using Vec3T = Eigen::Matrix<S, 3, 1>;

    Vec3T p     = Vec3T::Zero();   ///< position  (world frame)
    SO3T  R;                       ///< rotation  world ← body  (identity by default)
    Vec3T v     = Vec3T::Zero();   ///< linear velocity  (world frame)
    Vec3T omega = Vec3T::Zero();   ///< angular velocity (body frame)

    NavState() = default;
    NavState(Vec3T position, SO3T rotation, Vec3T velocity, Vec3T ang_vel)
        : p(std::move(position))
        , R(std::move(rotation))
        , v(std::move(velocity))
        , omega(std::move(ang_vel))
    {}
};

using NavStateF = NavState<float>;
using NavStateD = NavState<double>;

// ============================================================================
//  NavigationModel<S>  —  constant-velocity dead-reckoning model
//
//  Nominal-state propagation over dt:
//      p  ←  p + v · dt
//      R  ←  R · Exp(ω · dt)
//      v  ←  v               (no external acceleration)
//      ω  ←  ω               (no external angular acceleration)
//
//  Error-state layout (right-perturbation, 12-dim):
//      [ δp(3) | δθ(3) | δv(3) | δω(3) ]
//
//  The class exposes only static methods so that it acts as a pure
//  "model descriptor" consumed by ESKF<Model>.
// ============================================================================
template <typename S = double>
class NavigationModel {
public:
    // ---- Exported type aliases (required by ESKF) --------------------------
    using ScalarT   = S;
    using SO3T      = Sophus::SO3<S>;
    using Vec3T     = Eigen::Matrix<S, 3, 1>;
    using Mat3T     = Eigen::Matrix<S, 3, 3>;
    using NavStateT = NavState<S>;
    using ErrorVec  = Eigen::Matrix<S, kNavStateDim, 1>;
    using FMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< state-transition Jacobian
    using QMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< process-noise covariance
    using PMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< state covariance

    // ---- Process-noise configuration --------------------------------------
    struct Options { // Since we use a contant-velocity model, process error comes from variation of velocity
        S vel_noise_std   = static_cast<S>(0.10);  ///< lin-velocity noise σ         [m/s/√s]
        S omega_noise_std = static_cast<S>(0.10);  ///< ang-velocity noise σ         [rad/s/√s]
    };

    // ---- Nominal-state propagation ----------------------------------------
    [[nodiscard]]
    static NavStateT predictNominal(const NavStateT& s, S dt) {
        NavStateT ns;
        ns.p     = s.p + s.v * dt;
        ns.R     = s.R * SO3T::exp(s.omega * dt);
        ns.v     = s.v;
        ns.omega = s.omega;
        return ns;
    }

    // ---- Error-state transition Jacobian  F  (12×12)  ---------------------
    //
    //  Derived from first-order perturbation of the nominal dynamics:
    //    δp' = δp + δv·dt
    //    δθ' ≈ Exp(−ω·dt)·δθ + I·dt·δω     (right-perturbation model;
    //                                         Jr(ω·dt) ≈ I for small ω·dt)
    //    δv' = δv
    //    δω' = δω
    //
    [[nodiscard]]
    static FMat computeF(const NavStateT& s, S dt) {
        FMat F = FMat::Identity();

        // ∂δp / ∂δv
        F.template block<3, 3>(0, 6) = Mat3T::Identity() * dt;

        // ∂δθ / ∂δθ  —  Exp(−ω·dt): tangent transported by the rotation increment
        F.template block<3, 3>(3, 3) = SO3T::exp(-s.omega * dt).matrix();

        // ∂δθ / ∂δω  —  first-order: Jr(ω·dt)·dt ≈ I·dt
        F.template block<3, 3>(3, 9) = Mat3T::Identity() * dt;

        return F;
    }

    // ---- Discrete process-noise covariance  Q  (12×12) -------------------
    [[nodiscard]]
    static QMat buildQ(const Options& opts, S dt) {
        QMat Q = QMat::Zero();
        const S dt2 = dt * dt;

        auto diag3 = [](S sigma_sq_dt2) -> Mat3T {
            return Mat3T::Identity() * sigma_sq_dt2;
        };

        Q.template block<3, 3>(0, 0) = Mat3T::Zero(); // no direct position noise in constant-velocity model
        Q.template block<3, 3>(3, 3) = Mat3T::Zero(); // no direct rotation noise in constant-velocity model
        Q.template block<3, 3>(6, 6) = diag3(opts.vel_noise_std   * opts.vel_noise_std   * dt);
        Q.template block<3, 3>(9, 9) = diag3(opts.omega_noise_std * opts.omega_noise_std * dt);

        return Q;
    }

    // ---- Apply error-state correction to nominal state --------------------
    //  After correction the caller is responsible for zeroing dx.
    [[nodiscard]]
    static NavStateT applyErrorState(const NavStateT& nominal, const ErrorVec& dx) {
        NavStateT corrected;
        corrected.p     = nominal.p     + dx.template segment<3>(0);
        corrected.R     = nominal.R     * SO3T::exp(dx.template segment<3>(3));
        corrected.v     = nominal.v     + dx.template segment<3>(6);
        corrected.omega = nominal.omega + dx.template segment<3>(9);
        return corrected;
    }

    // ---- Covariance projection after error-state correction ---------------
    //  P ← J·P·J^T,   J = I − 0.5·[δθ]×  in the (3,3) rotation block.
    //  Ref: ESKF textbook eq. (3.63).
    [[nodiscard]]
    static PMat projectCovariance(const PMat& P, const ErrorVec& dx) {
        FMat J = FMat::Identity();
        J.template block<3, 3>(3, 3) =
            Mat3T::Identity() - static_cast<S>(0.5) * SO3T::hat(dx.template segment<3>(3));
        return J * P * J.transpose();
    }

    /// Typed aggregate returned by every observation helper.
    template <int Dim>
    struct ObsData {
        Eigen::Matrix<S, Dim, kNavStateDim> H;           ///< observation Jacobian
        Eigen::Matrix<S, Dim, 1>            innovation;  ///< z − h(x̂)
    };

    /// Typed aggregate returned by dual-epoch observation helpers.
    template <int Dim>
    struct DualObsData {
        Eigen::Matrix<S, Dim, kNavStateDim> H_to;        ///< Jacobian w.r.t. "to"   epoch error state
        Eigen::Matrix<S, Dim, kNavStateDim> H_from;      ///< Jacobian w.r.t. "from" epoch error state
        Eigen::Matrix<S, Dim, 1>            innovation;
    };

    // ---- Observe 6-DOF pose  [p_world, R_world←body]  --------------------
    //
    //  h_p(x) = p,    ∂h_p/∂δp = I₃
    //  h_θ(x) = δθ,   ∂h_θ/∂δθ = I₃   (linearised via log map)
    //
    //  H (6 × 12):
    //    [ I₃   0    0   0 ]   ← position rows
    //    [ 0    I₃   0   0 ]   ← rotation rows
    [[nodiscard]]
    static ObsData<6> observePose(const NavStateT& nominal,
                                  const Vec3T&     pos_obs,
                                  const SO3T&      rot_obs)
    {
        Eigen::Matrix<S, 6, kNavStateDim> H = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        H.template block<3, 3>(0, 0) = Mat3T::Identity();  // ∂z_p / ∂δp
        H.template block<3, 3>(3, 3) = Mat3T::Identity();  // ∂z_θ / ∂δθ

        Eigen::Matrix<S, 6, 1> innov;
        innov.template head<3>() = pos_obs - nominal.p;
        innov.template tail<3>() = (nominal.R.inverse() * rot_obs).log();

        return {H, innov};
    }

    // ---- Observe body-frame twist  [v_body, ω_body]  ----------------------
    //
    //  Both velocity and angular velocity are in the *body* frame.
    //  State stores v in world frame, so the observation function is:
    //    h_v(x) = R^T · v
    //    h_ω(x) = ω
    //
    //  First-order Jacobian (right-perturbation  R = R̂·Exp(δθ)):
    //    ∂h_v / ∂δθ = [R̂^T · v̂]×   (skew of nominal body-frame velocity)
    //    ∂h_v / ∂δv = R̂^T
    //    ∂h_ω / ∂δω = I₃
    //
    //  H (6 × 12):
    //    [ 0   [R̂ᵀv̂]×   R̂ᵀ   0  ]   ← v_body rows
    //    [ 0   0          0    I₃ ]   ← ω_body rows
    [[nodiscard]]
    static ObsData<6> observeVelocity(const NavStateT& nominal,
                                      const Vec3T&     lin_vel_obs_body,
                                      const Vec3T&     ang_vel_obs_body)
    {
        const Mat3T Rt         = nominal.R.inverse().matrix();  // R̂^T
        const Vec3T v_body_hat = Rt * nominal.v;                // R̂^T · v̂

        Eigen::Matrix<S, 6, kNavStateDim> H = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        H.template block<3, 3>(0, 3) = SO3T::hat(v_body_hat);  // ∂v_body / ∂δθ
        H.template block<3, 3>(0, 6) = Rt;                     // ∂v_body / ∂δv
        H.template block<3, 3>(3, 9) = Mat3T::Identity();       // ∂ω_body / ∂δω

        Eigen::Matrix<S, 6, 1> innov;
        innov.template head<3>() = lin_vel_obs_body - v_body_hat;
        innov.template tail<3>() = ang_vel_obs_body - nominal.omega;

        return {H, innov};
    }
    // ---- Observe relative pose [Δp_body, ΔR] spanning two epochs  ----------
    //
    //  Implements the Stochastic-Cloning dual-epoch observation model.
    //  Both the "from" and "to" nominal states are supplied by the caller
    //  (retrieved from eskf_.getStateAtDelay(delay_step_from/to)).
    //
    //  from_state : nominal at the "from" epoch  (T_k,     delay slot d_from)
    //  to_state   : nominal at the "to"   epoch  (T_{k+Δt}, delay slot d_to)
    //
    //  Measurement:
    //    z_p = R_from^{-1} · (p_to − p_from)   [body-frame displacement]
    //    z_R = R_from^{-1} · R_to               [relative rotation]
    //
    //  Predicted:
    //    ẑ_p = R̂_from^{-1} · Δp̂   where  Δp̂ = p̂_to − p̂_from
    //    ẑ_R = ΔR̂               where  ΔR̂  = R̂_from^{-1} · R̂_to
    //
    //  H_to   (6 × 12) — linearised w.r.t. "to" epoch error [δp, δθ, δv, δω]:
    //    [ R̂_from^{-1}   0   0   0 ]   ← Δp rows
    //    [ 0              I   0   0 ]   ← ΔR rows
    //
    //  H_from (6 × 12) — linearised w.r.t. "from" epoch error [δp, δθ, δv, δω]:
    //    [ −R̂_from^{-1}   [R̂_from^{-1}·Δp̂]×   0   0 ]   ← Δp rows
    //    [  0              −ΔR̂^T                0   0 ]   ← ΔR rows
    //
    //  R_noise must be a DISPLACEMENT covariance (m² / rad²), not velocity.
    [[nodiscard]]
    static DualObsData<6> observeRelativePose(const NavStateT& from_state,
                                              const NavStateT& to_state,
                                              const Vec3T&     delta_p_body_obs,
                                              const SO3T&      delta_R_obs)
    {
        const Mat3T Rf_inv     = from_state.R.inverse().matrix();          // R̂_from^{-1}
        const Vec3T delta_p_hat = to_state.p - from_state.p;              // Δp̂ (world frame)
        const SO3T  delta_R_hat = from_state.R.inverse() * to_state.R;    // ΔR̂

        // ── H_to ─────────────────────────────────────────────────────────────
        Eigen::Matrix<S, 6, kNavStateDim> Hto = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        Hto.template block<3, 3>(0, 0) = Rf_inv;              // ∂Δp / ∂δp_to
        Hto.template block<3, 3>(3, 3) = Mat3T::Identity();   // ∂ΔR / ∂δθ_to

        // ── H_from ───────────────────────────────────────────────────────────
        Eigen::Matrix<S, 6, kNavStateDim> Hfrom = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        Hfrom.template block<3, 3>(0, 0) = -Rf_inv;                              // ∂Δp / ∂δp_from
        Hfrom.template block<3, 3>(0, 3) = SO3T::hat(Rf_inv * delta_p_hat);      // ∂Δp / ∂δθ_from  ([R̂_from^{-1}Δp̂]×)
        Hfrom.template block<3, 3>(3, 3) = -delta_R_hat.matrix().transpose();    // ∂ΔR / ∂δθ_from  (−ΔR̂^T)

        // ── Innovation ───────────────────────────────────────────────────────
        Eigen::Matrix<S, 6, 1> innov;
        innov.template head<3>() = delta_p_body_obs - Rf_inv * delta_p_hat;
        innov.template tail<3>() = (delta_R_hat.inverse() * delta_R_obs).log();

        return {Hto, Hfrom, innov};
    }
};

using NavigationModelF = NavigationModel<float>;
using NavigationModelD = NavigationModel<double>;

}  // namespace finenav
