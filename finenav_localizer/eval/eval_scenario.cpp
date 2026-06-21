// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//

// Scenarios (select via cfg::kScenario):
//   1 — Uniform circular motion, all ideal            [implemented]
//   2 — Uniform circular, Session-B stress events     [stub]
//   3 — High-dynamic Lissajous figure-8, ideal        [stub]
//
// CSV schema (extended — all 12 state dims + non-2D covariance diagonals):
//   t | gt_{x,y,yaw,vx,vy,wz} | est_{x,y,yaw,vx,vy,wz} |
//   est_{z,vz,wx,wy} | cov_{pz,rx,ry,vz,wx,wy} |
//   obs_{x,y,yaw,vx_body,wz_body} | has_pose_obs | has_twist_obs | scenario

#include "eval_config.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

#include "finenav_localizer/finenav_localizer.hpp"

using namespace std::chrono_literals;
namespace fn = finenav;

static_assert(cfg::kUsePose || cfg::kUseTwist,
    "cfg: at least one of kUsePose / kUseTwist must be true.");

// ═════════════════════════════════════════════════════════════════════════════
//  Data types
// ═════════════════════════════════════════════════════════════════════════════

/// Ground-truth state for 2-D planar motion.
struct GtState {
    double x{}, y{}, yaw{};
    double vx_world{}, vy_world{};  ///< world-frame linear velocity  [m/s]
    double vx_body{};               ///< body-frame forward speed      [m/s]
    double wz{};                    ///< body-frame yaw rate           [rad/s]
};

/// One logged row — extended schema covers all 12 state dims.
/// Non-2D covariance diagonals let us spot divergence in unused dimensions.
struct Row {
    double t{};

    // Ground truth (6 quantities)
    double gt_x{}, gt_y{}, gt_yaw{};
    double gt_vx{}, gt_vy{}, gt_wz{};

    // Estimate — primary 2-D dims (visualised as time-series)
    double est_x{}, est_y{}, est_yaw{};
    double est_vx{}, est_vy{}, est_wz{};   ///< world v_x, v_y;  body ω_z

    // Estimate — non-2D dims
    double est_z{}, est_vz{}, est_wx{}, est_wy{};

    // Error-state covariance diagonal for non-2D dims
    //   state layout: [ δp(0-2) | δθ(3-5) | δv(6-8) | δω(9-11) ]
    double cov_pz{};   ///< P[2,2]
    double cov_rx{};   ///< P[3,3]
    double cov_ry{};   ///< P[4,4]
    double cov_vz{};   ///< P[8,8]
    double cov_wx{};   ///< P[9,9]
    double cov_wy{};   ///< P[10,10]

    // Observations  (NaN = absent this tick)
    double obs_x{}, obs_y{}, obs_yaw{};       ///< pose
    double obs_vx_body{}, obs_wz_body{};      ///< twist (body frame)
    bool   has_pose_obs{false};
    bool   has_twist_obs{false};

    const char* scenario{""};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Helper utilities  (file-internal linkage)
// ═════════════════════════════════════════════════════════════════════════════

static double noisy(double v, double sigma, std::mt19937& rng)
{
    return sigma > 0.0
        ? v + std::normal_distribution<double>(0.0, sigma)(rng)
        : v;
}

static double yawFromState(const fn::NavStateD& s)
{
    const Eigen::Quaterniond q(s.R.matrix());
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// Assemble pnode parameter overrides from cfg:: constants.
static rclcpp::NodeOptions makeOpts()
{
    rclcpp::NodeOptions o;
    o.append_parameter_override("update_rate_hz",        cfg::kUpdateRateHz);
    o.append_parameter_override("max_delay_sec",         cfg::kMaxDelaySec);
    o.append_parameter_override("pose_smoothing_steps",  int64_t{cfg::kPoseSmoothingSteps});
    o.append_parameter_override("twist_smoothing_steps", int64_t{cfg::kTwistSmoothingSteps});
    o.append_parameter_override("pose_gate_dist",        cfg::kPoseGateDist);
    o.append_parameter_override("twist_gate_dist",       cfg::kTwistGateDist);
    o.append_parameter_override("max_dt_sec",            cfg::kMaxDtSec);
    o.append_parameter_override("model.vel_noise_std",   cfg::kVelNoiseStd);
    o.append_parameter_override("model.omega_noise_std", cfg::kOmegaNoiseStd);
    return o;
}

/// Initialise the filter at the exact GT pose with tight uncertainty.
/// Velocity starts at zero with high covariance (filter learns from obs).
static void initLocalizer(fn::FineNavLocalizer& loc, const GtState& g,
                           const rclcpp::Node::SharedPtr& node)
{
    fn::FineNavLocalizer::PoseMsg m;
    m.header.stamp          = node->now();
    m.header.frame_id       = "map";
    m.pose.pose.position.x  = g.x;
    m.pose.pose.position.y  = g.y;
    m.pose.pose.position.z  = 0.0;
    m.pose.pose.orientation.w = std::cos(g.yaw * 0.5);
    m.pose.pose.orientation.x = 0.0;
    m.pose.pose.orientation.y = 0.0;
    m.pose.pose.orientation.z = std::sin(g.yaw * 0.5);
    m.pose.covariance.fill(0.0);
    m.pose.covariance[0]  = 1e-4;   // x
    m.pose.covariance[7]  = 1e-4;   // y
    m.pose.covariance[35] = 1e-4;   // yaw
    loc.setInitialPose(m);
}

/// Build a noisy PoseWithCovarianceStamped from GT state.
static fn::FineNavLocalizer::PoseMsg buildPose(
    const GtState& g, double p_sigma, double yaw_sigma,
    const rclcpp::Time& stamp, std::mt19937& rng)
{
    const double yaw = noisy(g.yaw, yaw_sigma, rng);

    fn::FineNavLocalizer::PoseMsg m;
    m.header.stamp = (cfg::kPoseDelaySec > 0.0)
        ? stamp - rclcpp::Duration::from_seconds(cfg::kPoseDelaySec)
        : stamp;
    m.header.frame_id       = "map";
    m.pose.pose.position.x  = noisy(g.x, p_sigma, rng);
    m.pose.pose.position.y  = noisy(g.y, p_sigma, rng);
    m.pose.pose.position.z  = 0.0;
    m.pose.pose.orientation.w = std::cos(yaw * 0.5);
    m.pose.pose.orientation.x = 0.0;
    m.pose.pose.orientation.y = 0.0;
    m.pose.pose.orientation.z = std::sin(yaw * 0.5);
    m.pose.covariance.fill(0.0);
    m.pose.covariance[0]  = std::max(p_sigma   * p_sigma,   1e-6);  // x
    m.pose.covariance[7]  = std::max(p_sigma   * p_sigma,   1e-6);  // y
    m.pose.covariance[14] = 1e-4;                                    // z
    m.pose.covariance[21] = 1e-4;                                    // roll
    m.pose.covariance[28] = 1e-4;                                    // pitch
    m.pose.covariance[35] = std::max(yaw_sigma * yaw_sigma, 1e-6);  // yaw
    return m;
}

/// Build a noisy TwistWithCovarianceStamped from GT state (body frame).
static fn::FineNavLocalizer::TwistMsg buildTwist(
    const GtState& g, double v_sigma, double w_sigma,
    const rclcpp::Time& stamp, std::mt19937& rng)
{
    fn::FineNavLocalizer::TwistMsg m;
    m.header.stamp = (cfg::kTwistDelaySec > 0.0)
        ? stamp - rclcpp::Duration::from_seconds(cfg::kTwistDelaySec)
        : stamp;
    m.header.frame_id       = "base_link";
    m.twist.twist.linear.x  = noisy(g.vx_body, v_sigma, rng);
    m.twist.twist.linear.y  = 0.0;
    m.twist.twist.linear.z  = 0.0;
    m.twist.twist.angular.x = 0.0;
    m.twist.twist.angular.y = 0.0;
    m.twist.twist.angular.z = noisy(g.wz, w_sigma, rng);
    m.twist.covariance.fill(0.0);
    m.twist.covariance[0]  = std::max(v_sigma * v_sigma, 1e-6);  // vx
    m.twist.covariance[7]  = 1e-4;                               // vy ~ 0
    m.twist.covariance[14] = 1e-4;                               // vz ~ 0
    m.twist.covariance[21] = 1e-4;                               // ωx ~ 0
    m.twist.covariance[28] = 1e-4;                               // ωy ~ 0
    m.twist.covariance[35] = std::max(w_sigma * w_sigma, 1e-6);  // wz
    return m;
}

/// Fill all estimate fields of a Row from the current filter state.
static void fillEstimate(Row& row, fn::FineNavLocalizer& loc)
{
    const fn::NavStateD              est = loc.getState();
    const fn::NavigationModelD::PMat P   = loc.getCovariance();

    // Primary 2-D dims.
    // getState() returns v in body frame; convert back to world frame so that
    // est_vx/vy are directly comparable with the world-frame GT columns.
    const Eigen::Vector3d v_world = est.R * est.v;
    row.est_x   = est.p.x();
    row.est_y   = est.p.y();
    row.est_yaw = yawFromState(est);
    row.est_vx  = v_world.x();   ///< world-frame vx  (matches gt_vx)
    row.est_vy  = v_world.y();   ///< world-frame vy  (matches gt_vy)
    row.est_wz  = est.omega.z();

    // Non-2D state values
    row.est_z   = est.p.z();
    row.est_vz  = est.v.z();
    row.est_wx  = est.omega.x();
    row.est_wy  = est.omega.y();

    // Non-2D covariance diagonals
    //   state layout: [ δp(0-2) | δθ(3-5) | δv(6-8) | δω(9-11) ]
    row.cov_pz = P(2,  2);
    row.cov_rx = P(3,  3);
    row.cov_ry = P(4,  4);
    row.cov_vz = P(8,  8);
    row.cov_wx = P(9,  9);
    row.cov_wy = P(10, 10);
}

/// Write all rows to cfg::kCsvPath.
static void writeCSV(const std::vector<Row>& rows)
{
    const std::string path = cfg::kCsvPath;
    std::ofstream f(path);
    if (!f) throw std::runtime_error("[writeCSV] Cannot open: " + path);

    f << "t,"
         "gt_x,gt_y,gt_yaw,gt_vx,gt_vy,gt_wz,"
         "est_x,est_y,est_yaw,est_vx,est_vy,est_wz,"
         "est_z,est_vz,est_wx,est_wy,"
         "cov_pz,cov_rx,cov_ry,cov_vz,cov_wx,cov_wy,"
         "obs_x,obs_y,obs_yaw,obs_vx_body,obs_wz_body,"
         "has_pose_obs,has_twist_obs,"
         "scenario\n";

    // Format a double: empty string for NaN (CSV-safe)
    auto F = [](double v) -> std::string {
        return std::isnan(v) ? "" : std::to_string(v);
    };

    for (const auto& r : rows) {
        f << r.t << ","
          << r.gt_x    << "," << r.gt_y    << "," << r.gt_yaw  << ","
          << r.gt_vx   << "," << r.gt_vy   << "," << r.gt_wz   << ","
          << r.est_x   << "," << r.est_y   << "," << r.est_yaw << ","
          << r.est_vx  << "," << r.est_vy  << "," << r.est_wz  << ","
          << r.est_z   << "," << r.est_vz  << ","
          << r.est_wx  << "," << r.est_wy  << ","
          << r.cov_pz  << "," << r.cov_rx  << "," << r.cov_ry  << ","
          << r.cov_vz  << "," << r.cov_wx  << "," << r.cov_wy  << ","
          << F(r.obs_x) << "," << F(r.obs_y) << "," << F(r.obs_yaw) << ","
          << F(r.obs_vx_body) << "," << F(r.obs_wz_body) << ","
          << (r.has_pose_obs  ? 1 : 0) << ","
          << (r.has_twist_obs ? 1 : 0) << ","
          << r.scenario << "\n";
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Scenario 1 — Uniform circular motion, all ideal
//
//  Ground truth (CCW circle):
//    x(t) =  R·cos(ω·t)           y(t) =  R·sin(ω·t)
//    yaw(t) = ω·t + π/2           (tangent direction)
//    vx_world = −R·ω·sin(ω·t)     vy_world = R·ω·cos(ω·t)
//    vx_body  =  R·ω  (const)     wz = ω  (const)
//
//  Observations (all at nominal noise — no outliers, no drop-outs):
//    Pose  @ cfg::kCirclePoseHz   (if cfg::kUsePose)
//    Twist @ cfg::kCircleTwistHz  (if cfg::kUseTwist)
// ═════════════════════════════════════════════════════════════════════════════
static void runCircleIdeal(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(),
        "═══ Scenario 1: Uniform circular, all ideal  "
        "(pose=%s  twist=%s) ═══",
        cfg::kUsePose  ? "ON" : "OFF",
        cfg::kUseTwist ? "ON" : "OFF");

    const double R = cfg::kCircleRadius;
    const double W = cfg::kCircleOmega;

    auto gt = [R, W](double t) -> GtState {
        const double th = W * t;
        GtState g;
        g.x        =  R * std::cos(th);
        g.y        =  R * std::sin(th);
        g.yaw      =  th + M_PI / 2.0;           // tangent (CCW)
        g.vx_world = -R * W * std::sin(th);
        g.vy_world =  R * W * std::cos(th);
        g.vx_body  =  R * W;                     // constant
        g.wz       =  W;
        return g;
    };

    fn::FineNavLocalizer localizer(node, makeOpts());
    initLocalizer(localizer, gt(0.0), node);
    rclcpp::sleep_for(40ms);   // let timer thread start

    constexpr double kDt    = 1.0 / cfg::kUpdateRateHz;
    constexpr double kTotal = cfg::kCircleDuration;
    const double     kNaN   = std::numeric_limits<double>::quiet_NaN();

    std::mt19937     rng(42);
    std::vector<Row> log;
    log.reserve(static_cast<std::size_t>(kTotal / kDt * 1.1));

    double t            = 0.0;
    double last_pose_t  = -999.0;
    double last_twist_t = -999.0;

    while (t < kTotal) {
        const GtState      g   = gt(t);
        const rclcpp::Time now = node->now();

        Row row;
        row.t             = t;
        row.gt_x          = g.x;
        row.gt_y          = g.y;
        row.gt_yaw        = g.yaw;
        row.gt_vx         = g.vx_world;
        row.gt_vy         = g.vy_world;
        row.gt_wz         = g.wz;
        row.obs_x         = kNaN;
        row.obs_y         = kNaN;
        row.obs_yaw       = kNaN;
        row.obs_vx_body   = kNaN;
        row.obs_wz_body   = kNaN;
        row.has_pose_obs  = false;
        row.has_twist_obs = false;
        row.scenario      = "CircleIdeal";

        // ── Pose observation ─────────────────────────────────────────────────
        if constexpr (cfg::kUsePose) {
            if ((t - last_pose_t) >= (1.0 / cfg::kCirclePoseHz)) {
                last_pose_t      = t;
                const auto pm    = buildPose(g, cfg::kPoseSigmaM,
                                             cfg::kPoseYawSigmaRad, now, rng);
                localizer.feedObservation(pm);
                row.has_pose_obs = true;
                row.obs_x        = pm.pose.pose.position.x;
                row.obs_y        = pm.pose.pose.position.y;
                row.obs_yaw      = std::atan2(
                    2.0 * pm.pose.pose.orientation.w * pm.pose.pose.orientation.z,
                    1.0 - 2.0 * pm.pose.pose.orientation.z
                               * pm.pose.pose.orientation.z);
            }
        }

        // ── Twist observation ────────────────────────────────────────────────
        if constexpr (cfg::kUseTwist) {
            if ((t - last_twist_t) >= (1.0 / cfg::kCircleTwistHz)) {
                last_twist_t      = t;
                const auto tm     = buildTwist(g, cfg::kTwistVelSigma,
                                               cfg::kTwistOmegaSigma, now, rng);
                localizer.feedObservation(tm);
                row.has_twist_obs = true;
                row.obs_vx_body   = tm.twist.twist.linear.x;
                row.obs_wz_body   = tm.twist.twist.angular.z;
            }
        }

        // ── Wait one filter tick, then snapshot ──────────────────────────────
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(kDt)));

        fillEstimate(row, localizer);
        log.push_back(row);
        t += kDt;
    }

    writeCSV(log);
    RCLCPP_INFO(node->get_logger(),
        "[Scenario 1] %zu rows written → %s", log.size(), cfg::kCsvPath);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Scenario 2 — Uniform circular, Session-B stress events
//
//  GT: identical circle to Scenario 1 (R, ω from cfg::).
//
//  Phase structure  (cfg::kCircleDuration must be ≥ 10 s):
//    0– 2 s  Normal    pose @ cfg::kCirclePoseHz,  no outliers
//    2– 4 s  LowFreq   pose @ 1 Hz,                no outliers
//    4– 6 s  HighFreq  pose @ 30 Hz,               no outliers
//    6– 8 s  ObsJump   pose @ cfg::kCirclePoseHz,  30 % large-noise outliers
//                        (outlier: p_sigma=2 m, yaw_sigma=0.8 rad)
//    8–10 s  PoseJump  pose @ cfg::kCirclePoseHz,  nominal noise PLUS a
//                        persistent step bias (cfg::kPoseJumpDx/Dy/Dyaw).
//                        All obs after t=8 s are: GT + step + Gaussian noise.
//                        The filter does not know about the bias.
//
//  cfg::kUseTwist is respected in all phases (no twist-forced override).
// ═════════════════════════════════════════════════════════════════════════════
static void runCircleStress(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(),
        "═══ Scenario 2: Uniform circular, stress events  "
        "(pose=%s  twist=%s) ═══",
        cfg::kUsePose  ? "ON" : "OFF",
        cfg::kUseTwist ? "ON" : "OFF");

    const double R = cfg::kCircleRadius;
    const double W = cfg::kCircleOmega;

    auto gt = [R, W](double t) -> GtState {
        const double th = W * t;
        GtState g;
        g.x        =  R * std::cos(th);
        g.y        =  R * std::sin(th);
        g.yaw      =  th + M_PI / 2.0;
        g.vx_world = -R * W * std::sin(th);
        g.vy_world =  R * W * std::cos(th);
        g.vx_body  =  R * W;
        g.wz       =  W;
        return g;
    };

    // Phase descriptors.
    // pose_active : whether pose is fed (ANDed with cfg::kUsePose)
    // outlier_frac: fraction of pose obs that are gross outliers
    // bias_*      : persistent step offset added to every pose obs in this phase
    //               (simulates a sudden sensor bias — filter does not see it)
    struct Phase {
        double      t0, t1;
        const char* label;
        bool        pose_active;
        double      pose_hz;
        double      outlier_frac;
        double      bias_x, bias_y, bias_yaw;  ///< step offset on top of GT
    };
    static const Phase kPhases[] = {
        {0.0, 2.0,  "Normal",   true,  cfg::kCirclePoseHz, 0.00, 0.0, 0.0, 0.0},
        {2.0, 4.0,  "LowFreq",  true,  1.0,                0.00, 0.0, 0.0, 0.0},
        {4.0, 6.0,  "HighFreq", true,  30.0,               0.00, 0.0, 0.0, 0.0},
        {6.0, 8.0,  "ObsJump",  true,  cfg::kCirclePoseHz, 0.30, 0.0, 0.0, 0.0},
        {8.0, 10.0, "PoseJump", true,  cfg::kCirclePoseHz, 0.00,
            cfg::kPoseJumpDx, cfg::kPoseJumpDy, cfg::kPoseJumpDyaw},
    };
    auto find_phase = [&](double t) -> const Phase& {
        for (const auto& ph : kPhases)
            if (t >= ph.t0 && t < ph.t1) return ph;
        return kPhases[std::size(kPhases) - 1];
    };

    fn::FineNavLocalizer localizer(node, makeOpts());
    initLocalizer(localizer, gt(0.0), node);
    rclcpp::sleep_for(40ms);

    constexpr double kDt    = 1.0 / cfg::kUpdateRateHz;
    constexpr double kTotal = cfg::kCircleDuration;
    const double     kNaN   = std::numeric_limits<double>::quiet_NaN();

    std::mt19937     rng(42);
    std::vector<Row> log;
    log.reserve(static_cast<std::size_t>(kTotal / kDt * 1.1));

    double t            = 0.0;
    double last_pose_t  = -999.0;
    double last_twist_t = -999.0;

    while (t < kTotal) {
        const Phase&       ph  = find_phase(t);
        const GtState      g   = gt(t);
        const rclcpp::Time now = node->now();

        Row row;
        row.t             = t;
        row.gt_x          = g.x;
        row.gt_y          = g.y;
        row.gt_yaw        = g.yaw;
        row.gt_vx         = g.vx_world;
        row.gt_vy         = g.vy_world;
        row.gt_wz         = g.wz;
        row.obs_x         = kNaN;
        row.obs_y         = kNaN;
        row.obs_yaw       = kNaN;
        row.obs_vx_body   = kNaN;
        row.obs_wz_body   = kNaN;
        row.has_pose_obs  = false;
        row.has_twist_obs = false;
        row.scenario      = ph.label;

        // ── Pose observation ─────────────────────────────────────────────────
        const bool do_pose = cfg::kUsePose && ph.pose_active && ph.pose_hz > 0.0;
        if (do_pose && (t - last_pose_t) >= (1.0 / ph.pose_hz)) {
            last_pose_t = t;

            const bool   is_outlier =
                (ph.outlier_frac > 0.0) &&
                std::uniform_real_distribution<double>(0.0, 1.0)(rng) < ph.outlier_frac;
            const double p_sig   = is_outlier ? 2.0 : cfg::kPoseSigmaM;
            const double yaw_sig = is_outlier ? 0.8 : cfg::kPoseYawSigmaRad;

            // Apply persistent step bias to the observed pose (GT + bias + noise).
            // The filter's covariance (R in the message) still reflects nominal
            // noise — the filter has no knowledge of the bias.
            GtState g_obs = g;
            g_obs.x   += ph.bias_x;
            g_obs.y   += ph.bias_y;
            g_obs.yaw += ph.bias_yaw;

            const auto pm = buildPose(g_obs, p_sig, yaw_sig, now, rng);
            localizer.feedObservation(pm);
            row.has_pose_obs = true;
            row.obs_x        = pm.pose.pose.position.x;
            row.obs_y        = pm.pose.pose.position.y;
            row.obs_yaw      = std::atan2(
                2.0 * pm.pose.pose.orientation.w * pm.pose.pose.orientation.z,
                1.0 - 2.0 * pm.pose.pose.orientation.z
                           * pm.pose.pose.orientation.z);
        }

        // ── Twist observation (cfg::kUseTwist respected in all phases) ───────
        if (cfg::kUseTwist && (t - last_twist_t) >= (1.0 / cfg::kCircleTwistHz)) {
            last_twist_t      = t;
            const auto tm     = buildTwist(g, cfg::kTwistVelSigma,
                                           cfg::kTwistOmegaSigma, now, rng);
            localizer.feedObservation(tm);
            row.has_twist_obs = true;
            row.obs_vx_body   = tm.twist.twist.linear.x;
            row.obs_wz_body   = tm.twist.twist.angular.z;
        }

        // ── Wait one filter tick, then snapshot ──────────────────────────────
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(kDt)));

        fillEstimate(row, localizer);
        log.push_back(row);
        t += kDt;
    }

    writeCSV(log);
    RCLCPP_INFO(node->get_logger(),
        "[Scenario 2] %zu rows written → %s", log.size(), cfg::kCsvPath);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Scenario 3 — High-dynamic Lissajous figure-8, all ideal
//
//  Ground truth:
//    x(t) = A·sin(2ω·t)       y(t) = A·sin(ω·t)
//
//  Exact analytic derivatives:
//    vx_world = 2A·ω·cos(2ω·t)         vy_world = A·ω·cos(ω·t)
//    yaw  = atan2(vy_world, vx_world)   (robot heading aligns with velocity)
//    wz   = (vx · vy' − vy · vx') / speed²
//         = 2ω·(2·cos(ω·t)·sin(2ω·t) − cos(2ω·t)·sin(ω·t))
//             / (4·cos²(2ω·t) + cos²(ω·t))
//    vx_body = speed = ‖v_world‖   (heading-aligned, vy_body ≡ 0)
//
//  Two complete figure-8 cycles in kFig8Duration = 20 s  (ω = π/5).
//  All observations at nominal noise — no outliers, no drop-outs.
// ═════════════════════════════════════════════════════════════════════════════
static void runHighDynIdeal(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(),
        "═══ Scenario 3: High-dynamic Lissajous figure-8, ideal  "
        "(pose=%s  twist=%s) ═══",
        cfg::kUsePose  ? "ON" : "OFF",
        cfg::kUseTwist ? "ON" : "OFF");

    const double A = cfg::kFig8Amplitude;
    const double W = cfg::kFig8Omega;

    auto gt = [A, W](double t) -> GtState {
        const double wt      = W * t;
        const double cos_wt  = std::cos(wt);
        const double sin_wt  = std::sin(wt);
        const double cos_2wt = std::cos(2.0 * wt);
        const double sin_2wt = std::sin(2.0 * wt);

        const double vx_w = 2.0 * A * W * cos_2wt;
        const double vy_w = A * W * cos_wt;

        // wz = (vx · (dvy/dt) − vy · (dvx/dt)) / speed²
        //   dvx/dt = −4AW²·sin(2Wt)
        //   dvy/dt = −AW²·sin(Wt)
        const double speed_sq = vx_w * vx_w + vy_w * vy_w;
        const double d_vx = -4.0 * A * W * W * sin_2wt;
        const double d_vy =       -A * W * W * sin_wt;
        const double wz   = (speed_sq > 1e-12)
            ? (vx_w * d_vy - vy_w * d_vx) / speed_sq
            : 0.0;

        GtState g;
        g.x        = A * sin_2wt;
        g.y        = A * sin_wt;
        g.yaw      = std::atan2(vy_w, vx_w);
        g.vx_world = vx_w;
        g.vy_world = vy_w;
        g.vx_body  = std::sqrt(speed_sq);   // heading aligned → vy_body = 0
        g.wz       = wz;
        return g;
    };

    fn::FineNavLocalizer localizer(node, makeOpts());
    initLocalizer(localizer, gt(0.0), node);
    rclcpp::sleep_for(40ms);

    constexpr double kDt    = 1.0 / cfg::kUpdateRateHz;
    constexpr double kTotal = cfg::kFig8Duration;
    const double     kNaN   = std::numeric_limits<double>::quiet_NaN();

    std::mt19937     rng(42);
    std::vector<Row> log;
    log.reserve(static_cast<std::size_t>(kTotal / kDt * 1.1));

    double t            = 0.0;
    double last_pose_t  = -999.0;
    double last_twist_t = -999.0;

    while (t < kTotal) {
        const GtState      g   = gt(t);
        const rclcpp::Time now = node->now();

        Row row;
        row.t             = t;
        row.gt_x          = g.x;
        row.gt_y          = g.y;
        row.gt_yaw        = g.yaw;
        row.gt_vx         = g.vx_world;
        row.gt_vy         = g.vy_world;
        row.gt_wz         = g.wz;
        row.obs_x         = kNaN;
        row.obs_y         = kNaN;
        row.obs_yaw       = kNaN;
        row.obs_vx_body   = kNaN;
        row.obs_wz_body   = kNaN;
        row.has_pose_obs  = false;
        row.has_twist_obs = false;
        row.scenario      = "HighDynIdeal";

        // ── Pose observation ─────────────────────────────────────────────────
        if constexpr (cfg::kUsePose) {
            if ((t - last_pose_t) >= (1.0 / cfg::kFig8PoseHz)) {
                last_pose_t      = t;
                const auto pm    = buildPose(g, cfg::kPoseSigmaM,
                                             cfg::kPoseYawSigmaRad, now, rng);
                localizer.feedObservation(pm);
                row.has_pose_obs = true;
                row.obs_x        = pm.pose.pose.position.x;
                row.obs_y        = pm.pose.pose.position.y;
                row.obs_yaw      = std::atan2(
                    2.0 * pm.pose.pose.orientation.w * pm.pose.pose.orientation.z,
                    1.0 - 2.0 * pm.pose.pose.orientation.z
                               * pm.pose.pose.orientation.z);
            }
        }

        // ── Twist observation ────────────────────────────────────────────────
        if constexpr (cfg::kUseTwist) {
            if ((t - last_twist_t) >= (1.0 / cfg::kFig8TwistHz)) {
                last_twist_t      = t;
                const auto tm     = buildTwist(g, cfg::kTwistVelSigma,
                                               cfg::kTwistOmegaSigma, now, rng);
                localizer.feedObservation(tm);
                row.has_twist_obs = true;
                row.obs_vx_body   = tm.twist.twist.linear.x;
                row.obs_wz_body   = tm.twist.twist.angular.z;
            }
        }

        // ── Wait one filter tick, then snapshot ──────────────────────────────
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(kDt)));

        fillEstimate(row, localizer);
        log.push_back(row);
        t += kDt;
    }

    writeCSV(log);
    RCLCPP_INFO(node->get_logger(),
        "[Scenario 3] %zu rows written → %s", log.size(), cfg::kCsvPath);
}

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_finenav_localizer");

    RCLCPP_INFO(node->get_logger(),
        "[test_finenav_localizer]  scenario=%d  pose=%s  twist=%s  → %s",
        cfg::kScenario,
        cfg::kUsePose  ? "ON" : "OFF",
        cfg::kUseTwist ? "ON" : "OFF",
        cfg::kCsvPath);

    switch (cfg::kScenario) {
        case 1:  runCircleIdeal(node);   break;
        case 2:  runCircleStress(node);  break;
        case 3:  runHighDynIdeal(node);  break;
        default:
            RCLCPP_ERROR(node->get_logger(),
                "Unknown cfg::kScenario=%d  (valid: 1, 2, 3).", cfg::kScenario);
            rclcpp::shutdown();
            return 1;
    }

    RCLCPP_INFO(node->get_logger(),
        "\n[Done]  Visualise with:\n"
        "  python3 finenav_localizer/eval/plot.py %s",
        cfg::kCsvPath);

    rclcpp::shutdown();
    return 0;
}
