// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  FineNavLocalizer — Scenario Test Configuration                         ║
// ║                                                                          ║
// ║  Edit ONLY this file to change test behaviour between runs.             ║
// ║  Recompile after any change.                                            ║
// ╚══════════════════════════════════════════════════════════════════════════╝

namespace cfg {

// ── 1. Scenario selection ─────────────────────────────────────────────────────
//   1 = Uniform circular motion, all ideal
//   2 = Uniform circular, Session-B stress events
//   3 = High-dynamic Lissajous figure-8, ideal
constexpr int kScenario = 2;

// ── 2. Observation mode ───────────────────────────────────────────────────────
//   At least one of kUsePose / kUseTwist must be true.
//   kUsePose  : feed PoseWithCovarianceStamped  (position + orientation)
//   kUseTwist : feed TwistWithCovarianceStamped (body-frame v + ω)
constexpr bool kUsePose  = true;
constexpr bool kUseTwist = true;

// ── 3. Output CSV path (one file per run) ─────────────────────────────────────
//   Overwrite freely — only one CSV is ever produced per run.
constexpr const char* kCsvPath = "/tmp/finenav_localizer_test.csv";

// ── 4. FineNavLocalizer filter parameters ────────────────────────────────────
constexpr double kUpdateRateHz        = 50.0;  ///< timer frequency      [Hz]
constexpr double kMaxDelaySec         = 0.3;   ///< max observation delay [s]
constexpr int    kPoseSmoothingSteps  = 10;     ///< pose obs reuse count
constexpr int    kTwistSmoothingSteps = 1;     ///< twist obs reuse count
constexpr double kPoseGateDist        = 1e9;   ///< Mahalanobis χ²(6) gate
constexpr double kTwistGateDist       = 1e9;
constexpr double kMaxDtSec            = 0.5;   ///< dt clamp              [s]
constexpr double kVelNoiseStd         = 0.5;   ///< process noise σ_v    [m/s/√s]
constexpr double kOmegaNoiseStd       = 0.1;   ///< process noise σ_ω  [rad/s/√s]

// ── 5. Observation noise ──────────────────────────────────────────────────────
constexpr double kPoseSigmaM      = 0.05;   ///< position σ             [m]
constexpr double kPoseYawSigmaRad = 0.02;   ///< yaw σ                  [rad]
constexpr double kTwistVelSigma   = 0.05;   ///< body-frame linear v σ  [m/s]
constexpr double kTwistOmegaSigma = 0.01;   ///< body-frame angular v σ [rad/s]

// ── 6. Observation delivery delay ────────────────────────────────────────────
//   Back-dates every header.stamp by a fixed amount to simulate sensor pipeline
//   latency (e.g. camera processing, network, serialisation).
//   The ESKF time-delay compensator (findDelayStep / updateWithDelay) corrects
//   for this automatically — no manual tuning at the filter level required.
//
//   Constraint: value must be < kMaxDelaySec, otherwise observations are
//   rejected as "too old" by timerCallback.
//   Set to 0.0 to disable (stamp = current wall time, no latency).
constexpr double kPoseDelaySec  = 0.005;   ///< fixed pose  pipeline delay [s]
constexpr double kTwistDelaySec = 0.005;   ///< fixed twist pipeline delay [s]

// ── 7. Scenario-specific simulation parameters ────────────────────────────────

// Scenarios 1 & 2 — uniform circle
constexpr double kCircleRadius   = 3.0;                        ///< [m]
constexpr double kCircleOmega    = 3.14159265358979323846/5.0; ///< ω [rad/s]
constexpr double kCircleDuration = 10.0;                       ///< sim time [s]
constexpr double kCirclePoseHz   = 10.0;   ///< pose observation rate  [Hz]
constexpr double kCircleTwistHz  = 20.0;   ///< twist observation rate [Hz]

// Scenario 2 — phase 5 (PoseJump): persistent step bias added to every pose obs.
// After t=8 s all pose observations carry this constant offset on top of Gaussian
// noise.  The filter does not know about the bias; observe how fast it tracks.
// If initial observations are rejected by the Mahalanobis gate (d² > kPoseGateDist),
// increase kPoseGateDist or reduce the step magnitude.
constexpr double kPoseJumpDx   = 2.0;    ///< step offset in x    [m]
constexpr double kPoseJumpDy   = 2.0;    ///< step offset in y    [m]
constexpr double kPoseJumpDyaw = 0.85;   ///< step offset in yaw  [rad]

// Scenario 3 — Lissajous figure-8  (x=A·sin(2ωt), y=A·sin(ωt))
constexpr double kFig8Amplitude = 3.0;
constexpr double kFig8Omega     = 3.14159265358979323846/5.0;
constexpr double kFig8Duration  = 20.0;
constexpr double kFig8PoseHz    = 10.0;
constexpr double kFig8TwistHz   = 20.0;

} // namespace cfg

