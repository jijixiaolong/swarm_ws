#!/usr/bin/env python3
"""
Swarm flight data visualization – focused on control forces, distances, and payload tracking.
"""

import os
import warnings
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
warnings.filterwarnings('ignore')
warnings.filterwarnings('ignore', message='No artists with labels found to put in legend.*')
from scipy.spatial.transform import Rotation

import sys
import argparse


def _to_mpl_value(value):
    if isinstance(value, (pd.Series, pd.Index)):
        return value.to_numpy()
    return value


def _patch_axes_method(name):
    from matplotlib.axes import Axes
    original = getattr(Axes, name)

    def wrapper(self, *args, **kwargs):
        args = tuple(_to_mpl_value(arg) for arg in args)
        kwargs = {key: _to_mpl_value(val) for key, val in kwargs.items()}
        return original(self, *args, **kwargs)

    setattr(Axes, name, wrapper)


for _axes_method in ("plot", "scatter", "step", "fill_between"):
    _patch_axes_method(_axes_method)

CSV_DIR = None
OUT_DIR = None
GLOBAL_TIME_ORIGIN_S = None
TIME_WINDOW_START_S = 0.0
TIME_WINDOW_END_S = None
TIME_REBASE_TO_WINDOW = True

UAV_IDS    = [1, 2, 3]
UAV_COLORS = {1: "#E74C3C", 2: "#2ECC71", 3: "#3498DB"}
UAV_LABELS = {1: "UAV-1", 2: "UAV-2", 3: "UAV-3"}
PAYLOAD_COLOR = "#F39C12"

XYZ_COLORS = ["#E74C3C", "#2ECC71", "#3498DB"]

# ── helpers ────────────────────────────────────────────────────────────────────
def load(name):
    path = os.path.join(CSV_DIR, name)
    if not os.path.exists(path):
        return None
    df = pd.read_csv(path)
    if "_bag_time_s" in df.columns:
        t0 = GLOBAL_TIME_ORIGIN_S if GLOBAL_TIME_ORIGIN_S is not None else df["_bag_time_s"].iloc[0]
        df["t"] = df["_bag_time_s"] - t0
        if TIME_WINDOW_START_S > 0.0:
            df = df.loc[df["t"] >= TIME_WINDOW_START_S].reset_index(drop=True)
        if TIME_WINDOW_END_S is not None:
            df = df.loc[df["t"] <= TIME_WINDOW_END_S].reset_index(drop=True)
        if df.empty:
            return None
        if TIME_REBASE_TO_WINDOW and TIME_WINDOW_START_S > 0.0:
            df["t"] = df["t"] - TIME_WINDOW_START_S
    return df


def detect_global_time_origin():
    if not CSV_DIR or not os.path.isdir(CSV_DIR):
        return None
    first_samples = []
    for name in sorted(os.listdir(CSV_DIR)):
        if not name.endswith(".csv"):
            continue
        path = os.path.join(CSV_DIR, name)
        try:
            df = pd.read_csv(path, usecols=["_bag_time_s"], nrows=1)
        except Exception:
            continue
        if not df.empty:
            first_samples.append(float(df["_bag_time_s"].iloc[0]))
    return min(first_samples) if first_samples else None


def detect_common_valid_start_bag_time():
    starts = []
    required_columns = ["_bag_time_s", "uav_position.x", "uav_position.y", "uav_position.z"]
    for uid in UAV_IDS:
        path = os.path.join(CSV_DIR, f"px4_{uid}_fsmpx4_fsm_debug.csv")
        if not os.path.exists(path):
            continue
        try:
            df = pd.read_csv(path, usecols=required_columns)
        except ValueError:
            continue
        position_mag = (
            df["uav_position.x"].abs()
            + df["uav_position.y"].abs()
            + df["uav_position.z"].abs()
        )
        valid_mask = position_mag > 1e-6
        if valid_mask.any():
            starts.append(float(df.loc[valid_mask, "_bag_time_s"].iloc[0]))
    return max(starts) if starts else None


def detect_cmd_ctrl_start_bag_time():
    starts = []
    required_columns = ["_bag_time_s", "fsm_state_name"]
    for uid in UAV_IDS:
        path = os.path.join(CSV_DIR, f"px4_{uid}_fsmpx4_fsm_debug.csv")
        if not os.path.exists(path):
            continue
        try:
            df = pd.read_csv(path, usecols=required_columns)
        except ValueError:
            continue
        cmd_ctrl = df["fsm_state_name"] == "CMD_CTRL"
        if cmd_ctrl.any():
            starts.append(float(df.loc[cmd_ctrl, "_bag_time_s"].iloc[0]))
    if starts:
        return max(starts)
    planner_path = os.path.join(CSV_DIR, "px4_1_swarm_planner_debug.csv")
    if os.path.exists(planner_path):
        try:
            df = pd.read_csv(planner_path, usecols=["_bag_time_s"], nrows=1)
        except ValueError:
            return None
        if not df.empty:
            return float(df["_bag_time_s"].iloc[0])
    return None


def configure_time_window(window_mode):
    global GLOBAL_TIME_ORIGIN_S, TIME_WINDOW_START_S, TIME_WINDOW_END_S
    GLOBAL_TIME_ORIGIN_S = detect_global_time_origin()
    TIME_WINDOW_START_S = 0.0
    TIME_WINDOW_END_S = None
    if GLOBAL_TIME_ORIGIN_S is None or window_mode == "full":
        return TIME_WINDOW_START_S
    detector = {
        "valid": detect_common_valid_start_bag_time,
        "cmd_ctrl": detect_cmd_ctrl_start_bag_time,
    }.get(window_mode)
    if detector is None:
        return TIME_WINDOW_START_S
    start_bag_time = detector()
    if start_bag_time is None and window_mode == "cmd_ctrl":
        start_bag_time = detect_common_valid_start_bag_time()
    if start_bag_time is not None:
        TIME_WINDOW_START_S = max(0.0, start_bag_time - GLOBAL_TIME_ORIGIN_S)
    return TIME_WINDOW_START_S


def load_uav_kinematics(uid):
    df = load(f"px4_{uid}_fsmpx4_fsm_debug.csv")
    if df is not None and has_columns(
        df,
        "uav_position.x", "uav_position.y", "uav_position.z",
        "uav_velocity.x", "uav_velocity.y", "uav_velocity.z",
    ):
        return df.rename(columns={
            "uav_position.x": "x", "uav_position.y": "y", "uav_position.z": "z",
            "uav_velocity.x": "vx", "uav_velocity.y": "vy", "uav_velocity.z": "vz",
        })
    return load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")


def savefig(fig, name, dpi=150):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)
    print(f"  saved: {name}")


def has_columns(df, *columns):
    return all(column in df.columns for column in columns)


def virtual_node_position(dp, node_idx):
    return (
        dp[f"virtual_positions_ned[{node_idx}].x"].values,
        dp[f"virtual_positions_ned[{node_idx}].y"].values,
        dp[f"virtual_positions_ned[{node_idx}].z"].values,
    )


def clear_png_outputs():
    if not OUT_DIR or not os.path.isdir(OUT_DIR):
        return
    for name in os.listdir(OUT_DIR):
        if name.endswith(".png"):
            os.unlink(os.path.join(OUT_DIR, name))


def _plot_xyz_force(ax, dp, key, label=None):
    """Plot x/y/z components of a vector field on ax. Returns True if data found."""
    if not has_columns(dp, f"{key}.x", f"{key}.y", f"{key}.z"):
        ax.text(0.5, 0.5, "n/a", ha='center', va='center',
                transform=ax.transAxes, fontsize=8, color='gray')
        ax.grid(True, alpha=0.3)
        return False
    for comp, col in zip(["x", "y", "z"], XYZ_COLORS):
        ax.plot(dp["t"], dp[f"{key}.{comp}"], color=col, lw=1.0,
                label=comp if label is None else None)
    ax.axhline(0, color='k', lw=0.4, ls='--')
    ax.grid(True, alpha=0.3)
    return True


def _uav_force_grid(rows_keys, rows_labels, title, filename):
    """
    Generic 3-col (UAV) × N-row (force key) grid.
    rows_keys: list of column-name prefixes, e.g. ["spring_force", "damping_force"]
    rows_labels: matching y-axis labels
    """
    n_rows = len(rows_keys)
    fig, axes = plt.subplots(n_rows, 3, figsize=(16, 3.2 * n_rows), sharex=True)
    axes = np.atleast_2d(axes)

    for col, uid in enumerate(UAV_IDS):
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        axes[0, col].set_title(UAV_LABELS[uid], fontweight='bold', fontsize=11)
        axes[-1, col].set_xlabel("Time (s)")
        for row, (key, ylabel) in enumerate(zip(rows_keys, rows_labels)):
            ax = axes[row, col]
            if dp is not None:
                _plot_xyz_force(ax, dp, key)
            else:
                ax.text(0.5, 0.5, "no data", ha='center', va='center',
                        transform=ax.transAxes, fontsize=8, color='gray')
                ax.grid(True, alpha=0.3)
            if col == 0:
                ax.set_ylabel(ylabel, fontsize=9)

    # Legend on top-left subplot
    axes[0, 0].plot([], [], color=XYZ_COLORS[0], lw=1.2, label='x')
    axes[0, 0].plot([], [], color=XYZ_COLORS[1], lw=1.2, label='y')
    axes[0, 0].plot([], [], color=XYZ_COLORS[2], lw=1.2, label='z')
    axes[0, 0].legend(fontsize=8, loc='upper right')

    fig.suptitle(title, fontsize=13, fontweight='bold')
    fig.tight_layout()
    savefig(fig, filename)


# ── 1. Trajectory overview ─────────────────────────────────────────────────────
def plot_overview():
    print("[1] Trajectory overview")
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    ax_xy, ax_alt = axes

    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None:
            continue
        ax_xy.plot(df["x"], df["y"], color=UAV_COLORS[uid], lw=1.4, label=UAV_LABELS[uid])
        ax_xy.scatter(df["x"].iloc[0],  df["y"].iloc[0],  color=UAV_COLORS[uid], marker='o', s=50, zorder=6)
        ax_xy.scatter(df["x"].iloc[-1], df["y"].iloc[-1], color=UAV_COLORS[uid], marker='*', s=100, zorder=6)
        ax_alt.plot(df["t"], -df["z"], color=UAV_COLORS[uid], lw=1.4, label=UAV_LABELS[uid])

    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is not None:
        ax_xy.plot(dp["payload_position_ned.x"], dp["payload_position_ned.y"],
                   color=PAYLOAD_COLOR, lw=1.8, ls='--', label="Payload")
        ax_xy.plot(dp["payload_target_ned.x"], dp["payload_target_ned.y"],
                   color='purple', lw=1.0, ls=':', label="Target")
        ax_alt.plot(dp["t"], -dp["payload_position_ned.z"],
                    color=PAYLOAD_COLOR, lw=1.8, ls='--', label="Payload")
        ax_alt.plot(dp["t"], -dp["payload_target_ned.z"],
                    color='purple', lw=1.0, ls=':', label="Target alt")

    ax_xy.set_xlabel("X NED (m)"); ax_xy.set_ylabel("Y NED (m)")
    ax_xy.set_title("Top-Down Trajectory (XY)", fontsize=11, fontweight='bold')
    ax_xy.set_aspect('equal'); ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(fontsize=8, ncol=2)

    ax_alt.set_xlabel("Time (s)"); ax_alt.set_ylabel("Altitude (m, up+)")
    ax_alt.set_title("Altitude Profile", fontsize=11, fontweight='bold')
    ax_alt.grid(True, alpha=0.3); ax_alt.legend(fontsize=8, ncol=2)

    fig.suptitle("Swarm Flight Overview", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "01_overview.png")


# ── 2. Payload tracking error ──────────────────────────────────────────────────
def plot_payload_error():
    print("[2] Payload tracking error")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
        return

    ex = dp["payload_position_ned.x"] - dp["payload_target_ned.x"]
    ey = dp["payload_position_ned.y"] - dp["payload_target_ned.y"]
    ez = dp["payload_position_ned.z"] - dp["payload_target_ned.z"]
    err = np.sqrt(ex**2 + ey**2 + ez**2)

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    for ax, data, lbl, col in zip(
        axes,
        [ex, ey, ez, err],
        ["e_x (m)", "e_y (m)", "e_z (m)", "|e| (m)"],
        [XYZ_COLORS[0], XYZ_COLORS[1], XYZ_COLORS[2], "#8E44AD"],
    ):
        ax.plot(dp["t"], data, color=col, lw=1.3)
        if lbl != "|e| (m)":
            ax.axhline(0, color='k', lw=0.4, ls='--')
        else:
            ax.fill_between(dp["t"], 0, err, color=col, alpha=0.15)
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Payload Position Tracking Error", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "02_payload_error.png")


# ── 3. Distances: inter-UAV + virtual rest-length error ───────────────────────
def plot_distances():
    print("[3] Distances (inter-UAV & virtual structure)")
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=False)
    ax_uav, ax_virt = axes

    # --- real inter-UAV distances ---
    dfs = {}
    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is not None:
            dfs[uid] = df

    pair_colors = ['#8E44AD', '#16A085', '#D35400']
    for (i, j), col in zip([(1, 2), (1, 3), (2, 3)], pair_colors):
        if i not in dfs or j not in dfs:
            continue
        a, b = dfs[i], dfs[j]
        t_c = np.linspace(max(a["t"].min(), b["t"].min()),
                          min(a["t"].max(), b["t"].max()), 2000)
        dist = np.sqrt(
            (np.interp(t_c, a["t"], a["x"]) - np.interp(t_c, b["t"], b["x"]))**2 +
            (np.interp(t_c, a["t"], a["y"]) - np.interp(t_c, b["t"], b["y"]))**2 +
            (np.interp(t_c, a["t"], a["z"]) - np.interp(t_c, b["t"], b["z"]))**2
        )
        ax_uav.plot(t_c, dist, color=col, lw=1.5, label=f"UAV-{i} ↔ UAV-{j}")

    ax_uav.set_ylabel("Distance (m)")
    ax_uav.set_xlabel("Time (s)")
    ax_uav.set_title("Inter-UAV Real Distances", fontsize=11, fontweight='bold')
    ax_uav.legend(fontsize=9); ax_uav.grid(True, alpha=0.3)

    # --- virtual node rest-length error ---
    dp = load("px4_1_swarm_planner_debug.csv")
    needed = [f"rest_lengths[{i}]" for i in range(25)]
    for i in range(5):
        for c in ["x", "y", "z"]:
            needed.append(f"virtual_positions_ned[{i}].{c}")

    if dp is not None and has_columns(dp, *needed):
        rl = np.array([[dp[f"rest_lengths[{i*5+j}]"].iloc[0] for j in range(5)] for i in range(5)])
        node_labels = ["1", "2", "3", "U", "L"]
        pair_list = [(0,1),(0,2),(1,2),(0,3),(0,4),(1,3),(1,4),(2,3),(2,4),(3,4)]
        cmap = plt.cm.tab10.colors
        abs_errors = []
        for k, (i, j) in enumerate(pair_list):
            ref = rl[i, j]
            if ref <= 0:
                continue
            p_i = virtual_node_position(dp, i)
            p_j = virtual_node_position(dp, j)
            actual = np.sqrt((p_i[0]-p_j[0])**2 + (p_i[1]-p_j[1])**2 + (p_i[2]-p_j[2])**2)
            err = actual - ref
            lbl = f"{node_labels[i]}↔{node_labels[j]} (r={ref:.2f}m)"
            ax_virt.plot(dp["t"], err, color=cmap[k % 10], lw=1.0, label=lbl, alpha=0.85)
            abs_errors.append(np.abs(err))
        ax_virt.axhline(0, color='k', lw=0.5, ls='--')
        ax_virt.legend(fontsize=7, ncol=2)
    else:
        ax_virt.text(0.5, 0.5, "virtual structure data not available",
                     ha='center', va='center', transform=ax_virt.transAxes, fontsize=9)

    ax_virt.set_ylabel("Δ dist − rest_length (m)")
    ax_virt.set_xlabel("Time (s)")
    ax_virt.set_title("Virtual Node Rest-Length Error", fontsize=11, fontweight='bold')
    ax_virt.grid(True, alpha=0.3)

    fig.suptitle("Formation Distances", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "03_distances.png")


# ── 4. Passive forces: spring / damping / friction ────────────────────────────
def plot_passive_forces():
    print("[4] Passive forces (spring / damping / friction)")
    _uav_force_grid(
        rows_keys=["spring_force", "damping_force", "friction_force"],
        rows_labels=["spring (N)", "damping (N)", "friction (N)"],
        title="Passive Network Forces",
        filename="04_passive_forces.png",
    )


# ── 5. Active forces: tracking / azimuth / rope_compensation ─────────────────
def plot_active_forces():
    print("[5] Active forces (tracking_input / azimuth_stab / rope_compensation)")
    _uav_force_grid(
        rows_keys=["tracking_input", "azimuth_stabilization_acceleration", "rope_compensation_acceleration"],
        rows_labels=["tracking input\n(m/s²×m)", "azimuth stab\n(m/s²)", "rope comp\n(m/s²)"],
        title="Active / Compensation Forces",
        filename="05_active_forces.png",
    )


# ── 6. Tracking PID breakdown ─────────────────────────────────────────────────
def plot_tracking_pid():
    print("[6] Tracking PID breakdown (P / I / D / total)")
    _uav_force_grid(
        rows_keys=["tracking_p_term", "tracking_i_term", "tracking_d_term", "tracking_input"],
        rows_labels=["P term", "I term", "D term", "total"],
        title="Payload Tracking PID Breakdown",
        filename="06_tracking_pid.png",
    )


# ── 7. CFO observer internals ─────────────────────────────────────────────────
def plot_cfo():
    print("[7] CFO observer (tension / disturbance_hat / compensation)")
    n_rows = 3
    fig, axes = plt.subplots(n_rows, 3, figsize=(16, 9), sharex=True)
    axes = np.atleast_2d(axes)

    for col, uid in enumerate(UAV_IDS):
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        axes[0, col].set_title(UAV_LABELS[uid], fontweight='bold', fontsize=11)
        axes[-1, col].set_xlabel("Time (s)")

        # Row 0: estimated rope tension (scalar)
        ax = axes[0, col]
        if dp is not None and "estimated_rope_tension_n" in dp.columns:
            ax.plot(dp["t"], dp["estimated_rope_tension_n"], color="#E74C3C", lw=1.2)
        else:
            ax.text(0.5, 0.5, "n/a", ha='center', va='center',
                    transform=ax.transAxes, fontsize=8, color='gray')
        ax.grid(True, alpha=0.3)
        if col == 0:
            ax.set_ylabel("est. tension (N)", fontsize=9)

        # Row 1: rope_axis_disturbance_hat (scalar, proportional to tension/mass)
        ax = axes[1, col]
        if dp is not None and "rope_axis_disturbance_hat" in dp.columns:
            ax.plot(dp["t"], dp["rope_axis_disturbance_hat"], color="#F39C12", lw=1.2)
        else:
            ax.text(0.5, 0.5, "n/a", ha='center', va='center',
                    transform=ax.transAxes, fontsize=8, color='gray')
        ax.grid(True, alpha=0.3)
        if col == 0:
            ax.set_ylabel("disturbance hat\n(m/s²)", fontsize=9)

        # Row 2: rope_compensation_acceleration xyz
        ax = axes[2, col]
        if dp is not None:
            _plot_xyz_force(ax, dp, "rope_compensation_acceleration")
        else:
            ax.text(0.5, 0.5, "no data", ha='center', va='center',
                    transform=ax.transAxes, fontsize=8, color='gray')
            ax.grid(True, alpha=0.3)
        if col == 0:
            ax.set_ylabel("rope comp\naccel (m/s²)", fontsize=9)

    axes[2, 0].plot([], [], color=XYZ_COLORS[0], lw=1.2, label='x')
    axes[2, 0].plot([], [], color=XYZ_COLORS[1], lw=1.2, label='y')
    axes[2, 0].plot([], [], color=XYZ_COLORS[2], lw=1.2, label='z')
    axes[2, 0].legend(fontsize=8)

    fig.suptitle("CFO Rope Observer", fontsize=13, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "07_cfo_observer.png")


# ── 8. Desired acceleration + beta ────────────────────────────────────────────
def plot_desired_accel_beta():
    print("[8] Desired acceleration + beta")
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    for uid in UAV_IDS:
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None:
            continue
        col = UAV_COLORS[uid]
        lbl = UAV_LABELS[uid]
        if has_columns(dp, "desired_acceleration.x", "desired_acceleration.y", "desired_acceleration.z"):
            axes[0].plot(dp["t"], dp["desired_acceleration.x"], color=col, lw=1.1, label=lbl)
            axes[1].plot(dp["t"], dp["desired_acceleration.y"], color=col, lw=1.1)
            axes[2].plot(dp["t"], dp["desired_acceleration.z"], color=col, lw=1.1)
        if has_columns(dp, "beta[0]", "beta[1]", "beta[2]"):
            # Use the UAV's own beta (self_index slot)
            i = uid - 1
            axes[3].plot(dp["t"], dp[f"beta[{i}]"], color=col, lw=1.2, label=lbl)

    for ax, lbl in zip(axes, ["a_x (m/s²)", "a_y (m/s²)", "a_z (m/s²)", "beta (self)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3); ax.axhline(0, color='k', lw=0.4, ls='--')
    axes[0].legend(fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Planner Output: Desired Acceleration & Beta", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "08_desired_accel_beta.png")


# ── 9. Status flags ───────────────────────────────────────────────────────────
def plot_status():
    print("[9] Status flags")
    flags = ["structure_locked", "valid"]
    available = []
    for flag in flags:
        if any(
            (dp := load(f"px4_{uid}_swarm_planner_debug.csv")) is not None and flag in dp.columns
            for uid in UAV_IDS
        ):
            available.append(flag)
    if not available:
        print("   no status flags, skipping")
        return

    fig, axes = plt.subplots(len(available), 1, figsize=(14, 2.8 * len(available) + 1), sharex=True)
    axes = np.atleast_1d(axes)

    for uid in UAV_IDS:
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None:
            continue
        offset = (uid - 1) * 0.08
        for ax, flag in zip(axes, available):
            if flag not in dp.columns:
                continue
            ax.step(dp["t"], dp[flag].astype(float) + offset, where='post',
                    color=UAV_COLORS[uid], lw=1.5, label=UAV_LABELS[uid])

    for ax, flag in zip(axes, available):
        ax.set_ylabel(flag, fontsize=9); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9, ncol=3)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Swarm Planner Status Flags", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "09_status_flags.png")


# ── paper figures ─────────────────────────────────────────────────────────────
def plot_paper_formation():
    """Paper Fig 2 – Formation: inter-UAV distances + virtual rest-length error."""
    print("[paper-2] Formation distances & structure error")

    PAPER_DPI = 300
    REF_DIST_M = 1.2          # equilateral side length from config

    pair_colors = ['#8E44AD', '#16A085', '#D35400']
    pair_labels = ["UAV-1 ↔ UAV-2", "UAV-1 ↔ UAV-3", "UAV-2 ↔ UAV-3"]

    # ── panel 1: real inter-UAV distances ──────────────────────────────────────
    dfs = {}
    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is not None:
            dfs[uid] = df

    has_dist = len(dfs) >= 2

    # ── panel 2: virtual rest-length error (UAV nodes only: 0↔1, 0↔2, 1↔2) ───
    dp = load("px4_1_swarm_planner_debug.csv")
    uav_pairs = [(0, 1), (0, 2), (1, 2)]
    needed_virt = [f"rest_lengths[{i*5+j}]" for i, j in uav_pairs]
    for i, j in uav_pairs:
        for c in ["x", "y", "z"]:
            needed_virt += [f"virtual_positions_ned[{i}].{c}",
                            f"virtual_positions_ned[{j}].{c}"]
    has_virt = dp is not None and has_columns(dp, *needed_virt)

    if not has_dist and not has_virt:
        print("   no formation data, skipping")
        return

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=False)
    ax_dist, ax_err = axes

    # panel 1
    if has_dist:
        for (i, j), col, lbl in zip([(1, 2), (1, 3), (2, 3)], pair_colors, pair_labels):
            if i not in dfs or j not in dfs:
                continue
            a, b = dfs[i], dfs[j]
            t_c = np.linspace(max(a["t"].min(), b["t"].min()),
                              min(a["t"].max(), b["t"].max()), 4000)
            dist = np.sqrt(
                (np.interp(t_c, a["t"], a["x"]) - np.interp(t_c, b["t"], b["x"]))**2 +
                (np.interp(t_c, a["t"], a["y"]) - np.interp(t_c, b["t"], b["y"]))**2 +
                (np.interp(t_c, a["t"], a["z"]) - np.interp(t_c, b["t"], b["z"]))**2
            )
            ax_dist.plot(t_c, dist, color=col, lw=1.2, label=lbl)
        ax_dist.axhline(REF_DIST_M, color='k', lw=0.8, ls='--', label=f"ref {REF_DIST_M} m")
    ax_dist.set_ylabel("Distance (m)", fontsize=10)
    ax_dist.set_xlabel("Time (s)", fontsize=10)
    ax_dist.legend(fontsize=8, ncol=2)
    ax_dist.grid(True, alpha=0.3)
    ax_dist.set_title("Inter-UAV Distances", fontsize=11, fontweight='bold')

    # panel 2
    if has_virt:
        node_labels = ["1", "2", "3"]
        for k, (i, j) in enumerate(uav_pairs):
            ref = dp[f"rest_lengths[{i*5+j}]"].iloc[0]
            if ref <= 0:
                continue
            p_i = virtual_node_position(dp, i)
            p_j = virtual_node_position(dp, j)
            actual = np.sqrt(
                (p_i[0] - p_j[0])**2 +
                (p_i[1] - p_j[1])**2 +
                (p_i[2] - p_j[2])**2
            )
            err = actual - ref
            ax_err.plot(dp["t"], err, color=pair_colors[k], lw=1.2,
                        label=f"{node_labels[i]}↔{node_labels[j]} (r={ref:.2f} m)")
        ax_err.axhline(0, color='k', lw=0.6, ls='--')
        ax_err.legend(fontsize=8, ncol=2)
    else:
        ax_err.text(0.5, 0.5, "virtual structure data not available",
                    ha='center', va='center', transform=ax_err.transAxes,
                    fontsize=9, color='gray')
    ax_err.set_ylabel("Δd − rest length (m)", fontsize=10)
    ax_err.set_xlabel("Time (s)", fontsize=10)
    ax_err.grid(True, alpha=0.3)
    ax_err.set_title("Virtual Structure Error (UAV Nodes)", fontsize=11, fontweight='bold')

    fig.tight_layout()
    savefig(fig, "paper_02_formation.png", dpi=PAPER_DPI)


def plot_paper_cfo():
    """Paper Fig 3 – CFO rope observer: estimated tension per UAV."""
    print("[paper-3] CFO rope observer – estimated tension")

    PAPER_DPI = 300

    # Check if any data is available
    any_data = False
    for uid in UAV_IDS:
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is not None and "estimated_rope_tension_n" in dp.columns:
            any_data = True
            break
    if not any_data:
        print("   no CFO tension data, skipping")
        return

    fig, ax = plt.subplots(1, 1, figsize=(8, 4))

    for uid in UAV_IDS:
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None or "estimated_rope_tension_n" not in dp.columns:
            continue
        ax.plot(dp["t"], dp["estimated_rope_tension_n"],
                color=UAV_COLORS[uid], lw=1.3, label=UAV_LABELS[uid])

    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.set_ylabel("Estimated Rope Tension (N)", fontsize=10)
    ax.set_title("CFO: Estimated Rope Tension", fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    savefig(fig, "paper_03_cfo_tension.png", dpi=PAPER_DPI)


# ── main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate swarm flight analysis plots from CSV data."
    )
    parser.add_argument(
        "analysis_dir",
        type=str,
        help="Path to the analysis directory (must contain a 'csv' subfolder).",
    )
    parser.add_argument(
        "--profile",
        choices=["full", "paper", "both"],
        default="full",
        help="full: all 9 diagnostic plots; paper: publication figures (formation + CFO); both: runs full then paper.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Optional explicit plot output directory.",
    )
    parser.add_argument(
        "--window",
        choices=["full", "valid", "cmd_ctrl"],
        default="full",
        help="Time window: full bag, first common valid pose, or CMD_CTRL only.",
    )
    args = parser.parse_args()
    analysis_dir = os.path.abspath(args.analysis_dir)
    if not os.path.isdir(analysis_dir):
        print(f"Error: not a directory: {analysis_dir}", file=sys.stderr)
        sys.exit(1)
    CSV_DIR = os.path.join(analysis_dir, "csv")
    if not os.path.isdir(CSV_DIR):
        print(f"Error: CSV folder not found: {CSV_DIR}", file=sys.stderr)
        sys.exit(1)

    if args.output_dir:
        OUT_DIR = os.path.abspath(args.output_dir)
    elif args.profile == "paper":
        OUT_DIR = os.path.join(analysis_dir, "plots_paper")
    else:
        OUT_DIR = os.path.join(analysis_dir, "plots")

    window_start_s = configure_time_window(args.window)
    os.makedirs(OUT_DIR, exist_ok=True)
    clear_png_outputs()
    print(f"CSV dir : {CSV_DIR}")
    print(f"Output  : {OUT_DIR}")
    print(f"Profile : {args.profile}")
    print(f"Window  : {args.window} (start = {window_start_s:.3f} s)\n")

    if args.profile == "paper":
        plot_paper_formation()
        plot_paper_cfo()
    else:
        plot_overview()
        plot_payload_error()
        plot_distances()
        plot_passive_forces()
        plot_active_forces()
        plot_tracking_pid()
        plot_cfo()
        plot_desired_accel_beta()
        plot_status()

    print("\nDone! Plots saved to:", OUT_DIR)
