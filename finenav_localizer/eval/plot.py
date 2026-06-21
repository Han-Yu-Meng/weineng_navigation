#!/usr/bin/env python3
"""Visualise FineNavLocalizer scenario-test output.

Usage:
    python3 eval/plot.py [CSV_PATH]

Default: /tmp/finenav_localizer_test.csv   (mirrors cfg::kCsvPath)

Extended CSV schema produced by test_finenav_localizer:
  t | gt_{x,y,yaw,vx,vy,wz} | est_{x,y,yaw,vx,vy,wz}
  | est_{z,vz,wx,wy} | cov_{pz,rx,ry,vz,wx,wy}
  | obs_{x,y,yaw,vx_body,wz_body} | has_pose_obs | has_twist_obs | scenario

Panel layout (3 rows × 3 cols):
  Row 0: 2-D trajectory        | x(t)    | y(t)
  Row 1: yaw(t)                | vx(t)   | vy(t)
  Row 2: wz(t)                 | non-2D covariance diagonals (spans 2 cols)

Observations are overlaid as scatter markers where present.
Scenario phases are shaded on every time-series panel.
Non-2D covariance diagonals (cov_pz, cov_rx, cov_ry, cov_vz, cov_wx, cov_wy)
are shown on a shared log-scale panel — runaway growth signals divergence in
the unused state dimensions of a 2-D scenario.

Requires: numpy, pandas, matplotlib ≥ 3.5
"""

#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import math
import os
import sys

import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

# ── Default CSV path (mirrors cfg::kCsvPath in localizer_test_config.hpp) ─────
DEFAULT_CSV = "/tmp/finenav_localizer_test.csv"

# ── Scenario → colour (tab10).  Add new labels here when more scenarios land. ─
_KNOWN_SCENARIOS = [
    "CircleIdeal",                                            # Scenario 1
    "Normal", "LowFreq", "HighFreq", "ObsJump", "PoseJump",  # Scenario 2 phases
    "HighDynIdeal",                                           # Scenario 3
]
_PAL = plt.cm.tab10(np.linspace(0, 0.9, len(_KNOWN_SCENARIOS)))
_SC_COLOR = {sc: tuple(_PAL[i]) for i, sc in enumerate(_KNOWN_SCENARIOS)}


def _c(sc: str) -> tuple:
    """Colour for a scenario label (grey fallback for unknown labels)."""
    return _SC_COLOR.get(sc, (0.45, 0.45, 0.45, 1.0))


def _wrap(series):
    """Wrap angle series to (−π, π]."""
    return series.apply(lambda a: (a + math.pi) % (2 * math.pi) - math.pi)


# ─────────────────────────────────────────────────────────────────────────────
def _shade_phases(ax, df: pd.DataFrame, scenarios: list) -> None:
    """Draw semi-transparent phase bands and phase-boundary tick lines."""
    for sc in scenarios:
        m = df["scenario"] == sc
        if not m.any():
            continue
        t0 = df.loc[m, "t"].min()
        t1 = df.loc[m, "t"].max()
        ax.axvspan(t0, t1, alpha=0.08, color=_c(sc), zorder=0)
        ax.axvline(t0, color=_c(sc), lw=0.7, ls=":", alpha=0.6, zorder=1)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Time [s]", fontsize=8)


# ─────────────────────────────────────────────────────────────────────────────
def _ts(ax, t, gt_vals, est_vals, ylabel: str, title: str,
        obs_t=None, obs_vals=None, obs_label: str = "Obs",
        scenarios=None, df=None) -> None:
    """Standard time-series panel: GT line, estimate line, optional obs scatter."""
    ax.plot(t, gt_vals,  "k-",  lw=1.8, label="GT",  zorder=3)
    ax.plot(t, est_vals, "b--", lw=1.3, label="Est", zorder=4)
    if obs_t is not None and obs_vals is not None and len(obs_t) > 0:
        ax.scatter(obs_t, obs_vals,
                   color="tomato", s=16, zorder=6, alpha=0.80, label=obs_label)
    ax.set_ylabel(ylabel, fontsize=8)
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=6.5, loc="upper right")
    if scenarios is not None and df is not None:
        _shade_phases(ax, df, scenarios)


# ─────────────────────────────────────────────────────────────────────────────
def plot_csv(csv_path: str) -> None:
    df = pd.read_csv(csv_path)

    # Cast potentially-empty observation columns to float (NaN where absent)
    for col in ["obs_x", "obs_y", "obs_yaw", "obs_vx_body", "obs_wz_body"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df["has_pose_obs"]  = df["has_pose_obs"].astype(bool)
    df["has_twist_obs"] = df["has_twist_obs"].astype(bool)

    scenarios = list(df["scenario"].unique())
    t = df["t"].to_numpy()

    # ── Figure & GridSpec ────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 11))
    fig.suptitle(
        f"FineNavLocalizer  ▸  {os.path.basename(csv_path)}",
        fontsize=13, fontweight="bold", y=0.998)

    gs = gridspec.GridSpec(
        3, 3, figure=fig,
        hspace=0.55, wspace=0.38,
        left=0.055, right=0.975,
        top=0.955,  bottom=0.065)

    ax_traj = fig.add_subplot(gs[0, 0])
    ax_x    = fig.add_subplot(gs[0, 1])
    ax_y    = fig.add_subplot(gs[0, 2])
    ax_yaw  = fig.add_subplot(gs[1, 0])
    ax_vx   = fig.add_subplot(gs[1, 1])
    ax_vy   = fig.add_subplot(gs[1, 2])
    ax_wz   = fig.add_subplot(gs[2, 0])
    ax_cov  = fig.add_subplot(gs[2, 1:])   # spans columns 1 and 2

    # ── Convenience: filtered obs arrays ────────────────────────────────────
    pose_m  = df["has_pose_obs"]  & df["obs_x"].notna()
    twist_m = df["has_twist_obs"] & df["obs_wz_body"].notna()

    pose_t   = df.loc[pose_m,  "t"].to_numpy()
    twist_t  = df.loc[twist_m, "t"].to_numpy()

    # ── Panel 1: 2-D Trajectory ───────────────────────────────────────────────
    ax_traj.plot(df["gt_x"],  df["gt_y"],  "k-",  lw=2.0, zorder=3, label="GT")
    ax_traj.plot(df["est_x"], df["est_y"], "b--", lw=1.5, zorder=4, label="Est")
    ax_traj.plot(df["gt_x"].iloc[0], df["gt_y"].iloc[0],
                 "k^", ms=9, zorder=7, clip_on=False)

    for sc in scenarios:
        m = df["scenario"] == sc
        ax_traj.plot(df.loc[m, "est_x"], df.loc[m, "est_y"],
                     color=_c(sc), lw=2.2, alpha=0.80, zorder=5)
        pm = pose_m & m
        if pm.any():
            ax_traj.scatter(df.loc[pm, "obs_x"], df.loc[pm, "obs_y"],
                            color=_c(sc), s=10, zorder=6, alpha=0.55)

    ax_traj.set_aspect("equal")
    ax_traj.set_xlabel("X [m]", fontsize=8)
    ax_traj.set_ylabel("Y [m]", fontsize=8)
    ax_traj.set_title("2-D Trajectory", fontsize=9)
    ax_traj.grid(True, alpha=0.3)

    # Scenario patches + GT/Est entries in the traj legend
    traj_handles = [
        Line2D([0], [0], color="k", lw=2.0,            label="GT"),
        Line2D([0], [0], color="b", lw=1.5, ls="--",   label="Est"),
    ]
    for sc in scenarios:
        traj_handles.append(mpatches.Patch(color=_c(sc), alpha=0.75, label=sc))
    ax_traj.legend(handles=traj_handles, fontsize=6.5, loc="best")

    # ── Panel 2: x(t) ────────────────────────────────────────────────────────
    _ts(ax_x, t,
        df["gt_x"].to_numpy(), df["est_x"].to_numpy(),
        "x [m]", "x  (world frame)",
        obs_t=pose_t, obs_vals=df.loc[pose_m, "obs_x"].to_numpy(),
        obs_label="Pose obs",
        scenarios=scenarios, df=df)

    # ── Panel 3: y(t) ────────────────────────────────────────────────────────
    _ts(ax_y, t,
        df["gt_y"].to_numpy(), df["est_y"].to_numpy(),
        "y [m]", "y  (world frame)",
        obs_t=pose_t, obs_vals=df.loc[pose_m, "obs_y"].to_numpy(),
        obs_label="Pose obs",
        scenarios=scenarios, df=df)

    # ── Panel 4: yaw(t) ───────────────────────────────────────────────────────
    _ts(ax_yaw, t,
        _wrap(df["gt_yaw"]).to_numpy(), _wrap(df["est_yaw"]).to_numpy(),
        "yaw [rad]", "yaw  (world frame)",
        obs_t=pose_t if df["has_pose_obs"].any() and df["obs_yaw"].notna().any() else None,
        obs_vals=_wrap(df.loc[pose_m, "obs_yaw"]).to_numpy() if pose_m.any() else None,
        obs_label="Pose obs",
        scenarios=scenarios, df=df)

    # ── Panel 5: vx(t)  — world frame  ───────────────────────────────────────
    _ts(ax_vx, t,
        df["gt_vx"].to_numpy(), df["est_vx"].to_numpy(),
        "vx [m/s]", "vx  (world frame, velocity highlight)",
        scenarios=scenarios, df=df)

    # ── Panel 6: vy(t)  — world frame ────────────────────────────────────────
    _ts(ax_vy, t,
        df["gt_vy"].to_numpy(), df["est_vy"].to_numpy(),
        "vy [m/s]", "vy  (world frame, velocity highlight)",
        scenarios=scenarios, df=df)

    # ── Panel 7: wz(t)  — body frame ─────────────────────────────────────────
    _ts(ax_wz, t,
        df["gt_wz"].to_numpy(), df["est_wz"].to_numpy(),
        "ωz [rad/s]", "ωz  (body frame, velocity highlight)",
        obs_t=twist_t if twist_m.any() else None,
        obs_vals=df.loc[twist_m, "obs_wz_body"].to_numpy() if twist_m.any() else None,
        obs_label="Twist obs",
        scenarios=scenarios, df=df)

    # ── Panel 8: Non-2D covariance diagonals ──────────────────────────────────
    # state layout: [ δp(0-2) | δθ(3-5) | δv(6-8) | δω(9-11) ]
    # 2-D motion uses p_x/y, θ_z, v_x/y, ω_z.
    # The six unused dims are p_z, θ_x, θ_y, v_z, ω_x, ω_y (indices 2,3,4,8,9,10).
    COV_SPEC = [
        ("cov_pz", "p_z   (P[2,2])",   "#1f77b4"),
        ("cov_rx", "θ_x   (P[3,3])",   "#ff7f0e"),
        ("cov_ry", "θ_y   (P[4,4])",   "#2ca02c"),
        ("cov_vz", "v_z   (P[8,8])",   "#d62728"),
        ("cov_wx", "ω_x   (P[9,9])",   "#9467bd"),
        ("cov_wy", "ω_y  (P[10,10])",  "#8c564b"),
    ]
    for col, label, color in COV_SPEC:
        vals = np.maximum(df[col].to_numpy(), 1e-15)   # guard against log(0)
        ax_cov.plot(t, vals, label=label, color=color, lw=1.25, alpha=0.90)

    ax_cov.set_yscale("log")
    ax_cov.set_ylabel("Covariance  [unit²]", fontsize=8)
    ax_cov.set_title(
        "Non-2D error-state covariance diagonals\n"
        "p_z | θ_x | θ_y | v_z | ω_x | ω_y — runaway growth = divergence",
        fontsize=9)
    ax_cov.legend(fontsize=7.5, loc="upper right", ncol=2)
    _shade_phases(ax_cov, df, scenarios)

    # ── Save ──────────────────────────────────────────────────────────────────
    out_png = csv_path.replace(".csv", ".png")
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[plot_localizer]  {len(df)} rows  →  {out_png}")
    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────
def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    if not os.path.exists(path):
        print(f"[ERROR] CSV not found: {path}")
        print("  Run the test first:")
        print("    ros2 run finenav_core test_finenav_localizer")
        sys.exit(1)
    plot_csv(path)


if __name__ == "__main__":
    main()

