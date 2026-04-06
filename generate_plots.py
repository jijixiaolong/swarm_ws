#!/usr/bin/env python3
"""
Comprehensive swarm flight data visualization script.
Generates detailed plots from CSV data recorded during swarm operation.
"""

import os
import warnings
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.axes import Axes
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
    original = getattr(Axes, name)

    def wrapper(self, *args, **kwargs):
        args = tuple(_to_mpl_value(arg) for arg in args)
        kwargs = {key: _to_mpl_value(val) for key, val in kwargs.items()}
        return original(self, *args, **kwargs)

    setattr(Axes, name, wrapper)


# Pandas 2 no longer supports the legacy indexing path used by older Matplotlib.
# Normalize Series/Index inputs at the plotting boundary so the existing code
# keeps working without rewriting every plotting call.
for _axes_method in ("plot", "scatter", "step", "fill_between"):
    _patch_axes_method(_axes_method)

# Paths: set in main from command-line (analysis directory that contains csv/)
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
    """Prefer FSM debug because its UAV pose is already expressed in a shared NED frame."""
    df = load(f"px4_{uid}_fsmpx4_fsm_debug.csv")
    if df is not None and has_columns(
        df,
        "uav_position.x",
        "uav_position.y",
        "uav_position.z",
        "uav_velocity.x",
        "uav_velocity.y",
        "uav_velocity.z",
    ):
        return df.rename(
            columns={
                "uav_position.x": "x",
                "uav_position.y": "y",
                "uav_position.z": "z",
                "uav_velocity.x": "vx",
                "uav_velocity.y": "vy",
                "uav_velocity.z": "vz",
            }
        )
    return load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")


def load_payload_kinematics():
    """Load the payload trajectory from a full-rate source instead of planner debug."""
    df = load("px4_4_fmu_out_vehicle_local_position.csv")
    if df is not None and has_columns(df, "x", "y", "z", "vx", "vy", "vz"):
        return df

    df = load("swarm_rviz_payload_pose.csv")
    if df is not None and has_columns(
        df,
        "pose.position.x",
        "pose.position.y",
        "pose.position.z",
    ):
        df = df.rename(
            columns={
                "pose.position.x": "x",
                "pose.position.y": "y",
                "pose.position.z": "z",
            }
        )
        if "vx" not in df.columns:
            df["vx"] = np.nan
            df["vy"] = np.nan
            df["vz"] = np.nan
        return df

    df = load("px4_1_swarm_planner_debug.csv")
    if df is not None and has_columns(
        df,
        "payload_position_ned.x",
        "payload_position_ned.y",
        "payload_position_ned.z",
        "payload_velocity_ned.x",
        "payload_velocity_ned.y",
        "payload_velocity_ned.z",
    ):
        return df.rename(
            columns={
                "payload_position_ned.x": "x",
                "payload_position_ned.y": "y",
                "payload_position_ned.z": "z",
                "payload_velocity_ned.x": "vx",
                "payload_velocity_ned.y": "vy",
                "payload_velocity_ned.z": "vz",
            }
        )
    return None


def load_payload_target():
    """Load the task target from a full-window command topic when available."""
    df = load("swarm_cmd_target_ned.csv")
    if df is not None and has_columns(df, "pose.position.x", "pose.position.y", "pose.position.z"):
        return df.rename(
            columns={
                "pose.position.x": "x",
                "pose.position.y": "y",
                "pose.position.z": "z",
            }
        )

    df = load("swarm_rviz_cmd_target_pose.csv")
    if df is not None and has_columns(df, "pose.position.x", "pose.position.y", "pose.position.z"):
        return df.rename(
            columns={
                "pose.position.x": "x",
                "pose.position.y": "y",
                "pose.position.z": "z",
            }
        )

    df = load("px4_1_swarm_planner_debug.csv")
    if df is not None and has_columns(
        df,
        "payload_target_ned.x",
        "payload_target_ned.y",
        "payload_target_ned.z",
    ):
        return df.rename(
            columns={
                "payload_target_ned.x": "x",
                "payload_target_ned.y": "y",
                "payload_target_ned.z": "z",
            }
        )
    return None


def plot_target_xy(ax, target_df, *, color="purple", label="Target", zorder=6):
    if target_df is None or not has_columns(target_df, "x", "y"):
        return

    unique_xy = target_df[["x", "y"]].drop_duplicates()
    if len(unique_xy) <= 1:
        ax.scatter(
            target_df["x"].iloc[-1],
            target_df["y"].iloc[-1],
            color=color,
            marker="*",
            s=120,
            zorder=zorder,
            label=label,
        )
        return

    ax.plot(target_df["x"], target_df["y"], color=color, lw=1.2, ls=":", label=label)

def quat_to_euler(q0, q1, q2, q3):
    """Convert quaternion [w,x,y,z] to Euler angles [roll,pitch,yaw] in degrees."""
    r = Rotation.from_quat(np.column_stack([q1, q2, q3, q0]))  # scipy: [x,y,z,w]
    angles = r.as_euler('xyz', degrees=True)
    return angles[:, 0], angles[:, 1], angles[:, 2]

def savefig(fig, name, dpi=150):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)
    print(f"  saved: {name}")


def has_columns(df, *columns):
    return all(column in df.columns for column in columns)


def planner_node_layout(dp):
    if dp is None:
        return None, 0
    for prefix in ("virtual_positions_ned", "node_positions_ned"):
        if not has_columns(dp, f"{prefix}[0].x", f"{prefix}[0].y", f"{prefix}[0].z"):
            continue
        count = 0
        while has_columns(dp, f"{prefix}[{count}].x", f"{prefix}[{count}].y", f"{prefix}[{count}].z"):
            count += 1
        return prefix, count
    return None, 0


def planner_node_labels(prefix, node_count):
    if prefix == "node_positions_ned" and node_count == 4:
        return ["UAV-1", "UAV-2", "UAV-3", "Payload"]
    if prefix == "virtual_positions_ned":
        return [f"VP-{idx}" for idx in range(node_count)]
    return [f"Node-{idx}" for idx in range(node_count)]


def planner_rest_length_matrix(dp, node_count):
    needed = [f"rest_lengths[{idx}]" for idx in range(node_count * node_count)]
    if not has_columns(dp, *needed):
        return None
    return np.array(
        [
            [dp[f"rest_lengths[{row * node_count + col}]"].iloc[0] for col in range(node_count)]
            for row in range(node_count)
        ]
    )


def planner_rest_length_pairs(node_count):
    if node_count == 4:
        labels = ["1", "2", "3", "P"]
    elif node_count == 5:
        labels = ["1", "2", "3", "u", "l"]
    else:
        labels = [str(idx) for idx in range(node_count)]
    return [((i, j), f"{labels[i]}↔{labels[j]}") for i in range(node_count) for j in range(i + 1, node_count)]


def virtual_node_position(dp, node_idx):
    prefix, node_count = planner_node_layout(dp)
    if prefix is None or node_idx >= node_count:
        raise KeyError(f"planner node {node_idx} not available")
    return (
        dp[f"{prefix}[{node_idx}].x"].values,
        dp[f"{prefix}[{node_idx}].y"].values,
        dp[f"{prefix}[{node_idx}].z"].values,
    )


def add_legend_if_handles(ax, *args, **kwargs):
    handles, labels = ax.get_legend_handles_labels()
    if handles and any(label and not label.startswith("_") for label in labels):
        ax.legend(*args, **kwargs)


def clear_png_outputs():
    if not OUT_DIR or not os.path.isdir(OUT_DIR):
        return
    for name in os.listdir(OUT_DIR):
        if name.endswith(".png"):
            os.unlink(os.path.join(OUT_DIR, name))

def colored_line(ax, x, y, c, cmap='viridis', lw=1.5, **kw):
    """Draw a line colored by parameter c."""
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segs   = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segs, cmap=cmap, linewidth=lw, **kw)
    lc.set_array(c)
    ax.add_collection(lc)
    return lc

# ── 1. 3-projection trajectory (XY / XZ / YZ) ────────────────────────────────
def plot_3d_trajectory():
    print("[1] 3-projection trajectories (XY / XZ / YZ)")
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    def _scatter_ends(ax, x, y, color):
        ax.scatter(x.iloc[0],  y.iloc[0],  color=color, marker='o', s=70, zorder=6)
        ax.scatter(x.iloc[-1], y.iloc[-1], color=color, marker='*', s=130, zorder=6)

    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None: continue
        c = UAV_COLORS[uid]; lbl = UAV_LABELS[uid]
        axes[0].plot(df["x"],  df["y"],  color=c, lw=1.3, label=lbl)
        axes[1].plot(df["x"], -df["z"],  color=c, lw=1.3)
        axes[2].plot(df["y"], -df["z"],  color=c, lw=1.3)
        _scatter_ends(axes[0], df["x"], df["y"],   c)
        _scatter_ends(axes[1], df["x"], -df["z"],  c)
        _scatter_ends(axes[2], df["y"], -df["z"],  c)

    payload_df = load_payload_kinematics()
    if payload_df is not None:
        px = payload_df["x"]
        py = payload_df["y"]
        pz = -payload_df["z"]
        axes[0].plot(px, py, color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")
        axes[1].plot(px, pz, color=PAYLOAD_COLOR, lw=1.5, ls='--')
        axes[2].plot(py, pz, color=PAYLOAD_COLOR, lw=1.5, ls='--')
    plot_target_xy(axes[0], load_payload_target())

    titles = ["Top view (XY)", "Front view (XZ)", "Side view (YZ)"]
    xlabels = ["X (m)", "X (m)", "Y (m)"]
    ylabels = ["Y (m)", "Z (m, up+)", "Z (m, up+)"]
    for ax, t, xl, yl in zip(axes, titles, xlabels, ylabels):
        ax.set_title(t, fontsize=11); ax.set_xlabel(xl); ax.set_ylabel(yl)
        ax.grid(True, alpha=0.3); ax.set_aspect('equal')
    axes[0].legend(fontsize=9, ncol=2)
    fig.suptitle("Trajectories – 3 Projections", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "01_3d_trajectories.png", dpi=150)

# ── 2. Top-down 2-D trajectory (XY) ───────────────────────────────────────────
def plot_xy_trajectory():
    print("[2] XY top-down trajectory")
    fig, ax = plt.subplots(figsize=(10, 8))

    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None: continue
        lc = colored_line(ax, df["x"].values, df["y"].values, df["t"].values,
                          cmap='plasma', lw=1.8)
        ax.scatter(df["x"].iloc[0],  df["y"].iloc[0],  color=UAV_COLORS[uid],
                   marker='o', s=80, zorder=6, label=f"{UAV_LABELS[uid]} start")
        ax.scatter(df["x"].iloc[-1], df["y"].iloc[-1], color=UAV_COLORS[uid],
                   marker='*', s=140, zorder=6, label=f"{UAV_LABELS[uid]} end")

    payload_df = load_payload_kinematics()
    if payload_df is not None:
        ax.plot(payload_df["x"], payload_df["y"],
                color=PAYLOAD_COLOR, lw=1.8, ls='--', label="Payload", alpha=0.9)
    plot_target_xy(ax, load_payload_target(), color='purple', label="Target")

    ax.set_xlabel("X NED (m)"); ax.set_ylabel("Y NED (m)")
    ax.set_title("Top-down Trajectory (XY) – color = time", fontsize=14, fontweight='bold')
    ax.legend(fontsize=8, ncol=2); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "02_xy_trajectory.png")

# ── 3. Position vs time (X, Y, Z) – payload only ──────────────────────────────
def plot_position_time():
    print("[3] Position vs time (payload)")
    payload_df = load_payload_kinematics()
    if payload_df is None:
        print("   no payload data, skipping")
        return
    target_df = load_payload_target()

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    axes[0].plot(payload_df["t"], payload_df["x"],
                 color=PAYLOAD_COLOR, lw=1.5, ls='-', label="Payload")
    axes[1].plot(payload_df["t"], payload_df["y"],
                 color=PAYLOAD_COLOR, lw=1.5, ls='-')
    axes[2].plot(payload_df["t"], -payload_df["z"],
                 color=PAYLOAD_COLOR, lw=1.5, ls='-')
    if target_df is not None:
        axes[0].plot(target_df["t"], target_df["x"],
                     color='purple', lw=1.0, ls='--', label="Target")
        axes[1].plot(target_df["t"], target_df["y"],
                     color='purple', lw=1.0, ls='--')
        axes[2].plot(target_df["t"], -target_df["z"],
                     color='purple', lw=1.0, ls='--')

    labels = ["X (m)", "Y (m)", "Z (m, up+)"]
    for ax, lbl in zip(axes, labels):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Payload Position vs Time", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "03_position_time.png")

# ── 4. Velocity vs time ────────────────────────────────────────────────────────
def plot_velocity_time():
    print("[4] Velocity vs time")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None: continue
        axes[0].plot(df["t"], df["vx"], color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])
        axes[1].plot(df["t"], df["vy"], color=UAV_COLORS[uid], lw=1.2)
        axes[2].plot(df["t"], df["vz"], color=UAV_COLORS[uid], lw=1.2)

    payload_df = load_payload_kinematics()
    if payload_df is not None and has_columns(payload_df, "vx", "vy", "vz"):
        axes[0].plot(payload_df["t"], payload_df["vx"], color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")
        axes[1].plot(payload_df["t"], payload_df["vy"], color=PAYLOAD_COLOR, lw=1.5, ls='--')
        axes[2].plot(payload_df["t"], payload_df["vz"], color=PAYLOAD_COLOR, lw=1.5, ls='--')

    for ax, lbl in zip(axes, ["Vx (m/s)", "Vy (m/s)", "Vz (m/s)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9, ncol=4)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Velocity vs Time – UAVs & Payload", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "04_velocity_time.png")

# ── 5. Attitude (roll, pitch, yaw) ────────────────────────────────────────────
def plot_attitude():
    print("[5] Attitude (Euler) vs time")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_attitude.csv")
        if df is None: continue
        roll, pitch, yaw = quat_to_euler(df["q[0]"], df["q[1]"], df["q[2]"], df["q[3]"])
        axes[0].plot(df["t"], roll,  color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])
        axes[1].plot(df["t"], pitch, color=UAV_COLORS[uid], lw=1.2)
        axes[2].plot(df["t"], yaw,   color=UAV_COLORS[uid], lw=1.2)

    for ax, lbl in zip(axes, ["Roll (°)", "Pitch (°)", "Yaw (°)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Attitude Euler Angles vs Time", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "05_attitude_euler.png")

# ── 6. Attitude setpoint vs actual ────────────────────────────────────────────
def plot_attitude_tracking():
    print("[6] Attitude tracking (setpoint vs actual)")
    fig, axes = plt.subplots(3, len(UAV_IDS), figsize=(16, 10), sharex='col')

    for col, uid in enumerate(UAV_IDS):
        da = load(f"px4_{uid}_fmu_out_vehicle_attitude.csv")
        ds = load(f"px4_{uid}_fmu_in_vehicle_attitude_setpoint.csv")
        if da is None or ds is None: continue

        roll_a, pitch_a, yaw_a = quat_to_euler(da["q[0]"], da["q[1]"], da["q[2]"], da["q[3]"])
        roll_s, pitch_s, yaw_s = quat_to_euler(ds["q_d[0]"], ds["q_d[1]"], ds["q_d[2]"], ds["q_d[3]"])

        axes[0, col].plot(da["t"], roll_a,  color='steelblue', lw=1.2, label='actual')
        axes[0, col].plot(ds["t"], roll_s,  color='tomato',    lw=1.0, ls='--', label='setpoint')
        axes[1, col].plot(da["t"], pitch_a, color='steelblue', lw=1.2)
        axes[1, col].plot(ds["t"], pitch_s, color='tomato',    lw=1.0, ls='--')
        axes[2, col].plot(da["t"], yaw_a,   color='steelblue', lw=1.2)
        axes[2, col].plot(ds["t"], yaw_s,   color='tomato',    lw=1.0, ls='--')

        axes[0, col].set_title(UAV_LABELS[uid], fontweight='bold')
        for ax in axes[:, col]: ax.grid(True, alpha=0.3)
        axes[-1, col].set_xlabel("Time (s)")

    axes[0, 0].legend(fontsize=9)
    for ax, lbl in zip(axes[:, 0], ["Roll (°)", "Pitch (°)", "Yaw (°)"]):
        ax.set_ylabel(lbl)
    fig.suptitle("Attitude Setpoint vs Actual", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "06_attitude_tracking.png")

# ── 7. Commanded thrust ────────────────────────────────────────────────────────
def plot_thrust():
    print("[7] Commanded thrust vs time")
    fig, ax = plt.subplots(figsize=(14, 5))

    for uid in UAV_IDS:
        ds = load(f"px4_{uid}_fmu_in_vehicle_attitude_setpoint.csv")
        if ds is None: continue
        # thrust_body[2] is negative thrust in PX4 convention
        thrust = -ds["thrust_body[2]"]
        ax.plot(ds["t"], thrust, color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])

    ax.set_xlabel("Time (s)"); ax.set_ylabel("Normalized Thrust")
    ax.set_title("Commanded Thrust vs Time", fontsize=14, fontweight='bold')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "07_commanded_thrust.png")

# ── 8. Inter-UAV distances ─────────────────────────────────────────────────────
def plot_inter_uav_distances():
    print("[8] Inter-UAV distances")
    fig, ax = plt.subplots(figsize=(14, 6))

    dfs = {}
    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is not None:
            dfs[uid] = df

    pairs = [(1,2), (1,3), (2,3)]
    colors_pair = ['#8E44AD', '#16A085', '#D35400']
    for (i, j), col in zip(pairs, colors_pair):
        if i not in dfs or j not in dfs: continue
        a, b = dfs[i], dfs[j]
        # Align on time using interpolation
        t_common = np.linspace(max(a["t"].min(), b["t"].min()),
                               min(a["t"].max(), b["t"].max()), 2000)
        xi = np.interp(t_common, a["t"], a["x"])
        yi = np.interp(t_common, a["t"], a["y"])
        zi = np.interp(t_common, a["t"], a["z"])
        xj = np.interp(t_common, b["t"], b["x"])
        yj = np.interp(t_common, b["t"], b["y"])
        zj = np.interp(t_common, b["t"], b["z"])
        dist = np.sqrt((xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2)
        ax.plot(t_common, dist, color=col, lw=1.5, label=f"UAV-{i} ↔ UAV-{j}")

    ax.set_xlabel("Time (s)"); ax.set_ylabel("Distance (m)")
    ax.set_title("Inter-UAV Distances vs Time", fontsize=14, fontweight='bold')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "08_inter_uav_distances.png")

# ── 9. Payload tracking error ──────────────────────────────────────────────────
def plot_payload_tracking_error():
    print("[9] Payload tracking error")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
        return

    ex = dp["payload_position_ned.x"] - dp["payload_target_ned.x"]
    ey = dp["payload_position_ned.y"] - dp["payload_target_ned.y"]
    ez = dp["payload_position_ned.z"] - dp["payload_target_ned.z"]
    err_total = np.sqrt(ex**2 + ey**2 + ez**2)

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    axes[0].plot(dp["t"], ex,        color='#E74C3C', lw=1.2, label='Ex')
    axes[1].plot(dp["t"], ey,        color='#2ECC71', lw=1.2, label='Ey')
    axes[2].plot(dp["t"], ez,        color='#3498DB', lw=1.2, label='Ez')
    axes[3].plot(dp["t"], err_total, color='#8E44AD', lw=1.5, label='|E|')
    axes[3].fill_between(dp["t"], 0, err_total, color='#8E44AD', alpha=0.2)

    for ax, lbl in zip(axes, ["Ex (m)", "Ey (m)", "Ez (m)", "|E| (m)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3); ax.axhline(0, color='k', lw=0.5)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Payload Position Tracking Error", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "09_payload_tracking_error.png")

# ── 10. Desired acceleration (planner output) ─────────────────────────────────
def plot_desired_acceleration():
    print("[10] Desired acceleration")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    has_data = False

    for uid in UAV_IDS:
        df = load(f"px4_{uid}_swarm_planner_debug.csv")
        if df is None: continue
        if not has_columns(df, "desired_acceleration.x", "desired_acceleration.y", "desired_acceleration.z"): continue
        has_data = True
        axes[0].plot(df["t"], df["desired_acceleration.x"], color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])
        axes[1].plot(df["t"], df["desired_acceleration.y"], color=UAV_COLORS[uid], lw=1.2)
        axes[2].plot(df["t"], df["desired_acceleration.z"], color=UAV_COLORS[uid], lw=1.2)

    for ax, lbl in zip(axes, ["ax (m/s²)", "ay (m/s²)", "az (m/s²)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3); ax.axhline(0, color='k', lw=0.5)
    if has_data:
        axes[0].legend(fontsize=9)
    else:
        axes[0].text(0.5, 0.5, "not available", ha='center', va='center',
                     transform=axes[0].transAxes, fontsize=10)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Planner Desired Acceleration vs Time", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "10_desired_acceleration.png")

# ── 11. Beta coefficients ──────────────────────────────────────────────────────
def plot_beta():
    print("[11] Beta coefficients")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    have_beta = False
    for uid in UAV_IDS:
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None or not has_columns(dp, "beta[0]", "beta[1]", "beta[2]"):
            continue
        have_beta = True
        for i, ax in enumerate(axes):
            ax.plot(dp["t"], dp[f"beta[{i}]"], color=UAV_COLORS[uid],
                    lw=1.2, label=UAV_LABELS[uid] if i == 0 else "")

    if have_beta:
        for i, ax in enumerate(axes):
            ax.set_ylabel(f"β[{i}]"); ax.grid(True, alpha=0.3)
        add_legend_if_handles(axes[0], fontsize=9)
        title = "Beta Coefficients (Force Allocation) vs Time"
    else:
        have_tracking_input = False
        for uid in UAV_IDS:
            dp = load(f"px4_{uid}_swarm_planner_debug.csv")
            if dp is None or not has_columns(dp, "tracking_input.x", "tracking_input.y", "tracking_input.z"):
                continue
            have_tracking_input = True
            axes[0].plot(dp["t"], dp["tracking_input.x"], color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])
            axes[1].plot(dp["t"], dp["tracking_input.y"], color=UAV_COLORS[uid], lw=1.2)
            axes[2].plot(dp["t"], dp["tracking_input.z"], color=UAV_COLORS[uid], lw=1.2)
        for ax, lbl in zip(axes, ["tracking_input.x", "tracking_input.y", "tracking_input.z"]):
            ax.set_ylabel(lbl); ax.grid(True, alpha=0.3); ax.axhline(0, color='k', lw=0.5)
        if have_tracking_input:
            add_legend_if_handles(axes[0], fontsize=9)
        else:
            axes[0].text(0.5, 0.5, "not available", ha='center', va='center',
                         transform=axes[0].transAxes, fontsize=10)
        title = "Tracking Input Components (beta not available)"
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "11_beta_coefficients.png")

# ── 12. Virtual positions ──────────────────────────────────────────────────────
def plot_virtual_positions():
    print("[12] Virtual formation positions")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
        return
    prefix, node_count = planner_node_layout(dp)
    if prefix is None:
        print("   no planner node positions, skipping")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    cmaps = plt.cm.Set1.colors
    labels = planner_node_labels(prefix, node_count)
    for vi in range(node_count):
        c = cmaps[vi % len(cmaps)]
        axes[0].plot(dp["t"], dp[f"{prefix}[{vi}].x"], color=c, lw=1.2, label=labels[vi])
        axes[1].plot(dp["t"], dp[f"{prefix}[{vi}].y"], color=c, lw=1.2)
        axes[2].plot(dp["t"], dp[f"{prefix}[{vi}].z"], color=c, lw=1.2)

    for ax, lbl in zip(axes, ["X (m)", "Y (m)", "Z (m)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9, ncol=min(node_count, 5))
    axes[-1].set_xlabel("Time (s)")
    title = "Virtual Formation Positions (NED) vs Time" if prefix == "virtual_positions_ned" \
        else "Planner Node Positions (NED) vs Time"
    fig.suptitle(title, fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "12_virtual_positions.png")

# ── 13. CFO / passive / virtual acceleration ─────────────────────────────────
def plot_control_forces():
    print("[13] Control forces (CFO / passive / virtual)")
    candidates = [
        ("passive_force", "Passive Force"),
        ("tracking_input", "Tracking Input"),
        ("raw_acceleration", "Raw Accel"),
        ("desired_acceleration", "Desired Accel"),
        ("virtual_acceleration", "Virtual Accel"),
        ("cfo_acceleration", "CFO Accel"),
    ]
    available = []
    for key, label in candidates:
        if any(
            (dp := load(f"px4_{uid}_swarm_planner_debug.csv")) is not None and
            has_columns(dp, f"{key}.x", f"{key}.y", f"{key}.z")
            for uid in UAV_IDS
        ):
            available.append((key, label))

    if not available:
        print("   no vector debug fields, skipping")
        return

    fig, axes = plt.subplots(len(available), len(UAV_IDS), figsize=(16, 3.0 * len(available) + 1.5), sharex=True)
    axes = np.asarray(axes, dtype=object)
    if axes.ndim == 1:
        axes = axes[np.newaxis, :]

    for col, uid in enumerate(UAV_IDS):
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None: continue

        for row, (key, label) in enumerate(available):
            ax = axes[row, col]
            if not has_columns(dp, f"{key}.x", f"{key}.y", f"{key}.z"):
                ax.text(0.5, 0.5, "not available", ha='center', va='center',
                        transform=ax.transAxes, fontsize=9)
                ax.grid(True, alpha=0.3)
                if col == 0:
                    ax.set_ylabel(label.replace(' ', '\n'), fontsize=8)
                if row == 0:
                    ax.set_title(UAV_LABELS[uid], fontweight='bold')
                if row == len(available) - 1:
                    ax.set_xlabel("Time (s)")
                continue
            ax.plot(dp["t"], dp[f"{key}.x"], color='#E74C3C', lw=1.0, label='x')
            ax.plot(dp["t"], dp[f"{key}.y"], color='#2ECC71', lw=1.0, label='y')
            ax.plot(dp["t"], dp[f"{key}.z"], color='#3498DB', lw=1.0, label='z')
            ax.grid(True, alpha=0.3); ax.axhline(0, color='k', lw=0.5)
            if col == 0:
                ax.set_ylabel(label.replace(' ', '\n'), fontsize=8)
            if row == 0:
                ax.set_title(UAV_LABELS[uid], fontweight='bold')
            if row == len(available) - 1:
                ax.set_xlabel("Time (s)")

    add_legend_if_handles(axes[0, 0], fontsize=8)
    fig.suptitle("Planner Force / Acceleration Components",
                 fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "13_control_forces.png")

# ── 14. Structure lock + valid status ─────────────────────────────────────────
def plot_status_flags():
    print("[14] Status flags (structure_locked, valid, used_cfo)")
    flags = []
    for flag in ["structure_locked", "valid", "used_cfo"]:
        if any(
            (dp := load(f"px4_{uid}_swarm_planner_debug.csv")) is not None and flag in dp.columns
            for uid in UAV_IDS
        ):
            flags.append(flag)
    if not flags:
        print("   no status flags, skipping")
        return

    fig, axes = plt.subplots(len(flags), 1, figsize=(14, 2.5 * len(flags) + 1), sharex=True)
    axes = np.atleast_1d(axes)

    for uid in UAV_IDS:
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None: continue
        offset = (uid - 1) * 0.1
        for ax, flag in zip(axes, flags):
            if flag not in dp.columns:
                continue
            val = dp[flag].astype(float) + offset
            ax.step(dp["t"], val, where='post', color=UAV_COLORS[uid],
                    lw=1.5, label=UAV_LABELS[uid])

    for ax, flag in zip(axes, flags):
        ax.set_ylabel(flag, fontsize=9); ax.grid(True, alpha=0.3)
        ax.set_yticks([0, 0.1, 0.2, 1.0, 1.1, 1.2])
    axes[0].legend(fontsize=9, ncol=3)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Swarm Planner Status Flags", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "14_status_flags.png")

# ── 15. Formation shape snapshots ─────────────────────────────────────────────
def plot_formation_snapshots():
    print("[15] Formation shape snapshots")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
        return
    prefix, node_count = planner_node_layout(dp)
    show_virtual_nodes = prefix == "virtual_positions_ned"

    t_total = dp["t"].max()
    snap_times = np.linspace(0.1 * t_total, 0.9 * t_total, 6)
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))

    for ax, ts in zip(axes.flat, snap_times):
        idx = (dp["t"] - ts).abs().idxmin()
        row = dp.loc[idx]
        # UAV positions
        for vi, uid in enumerate(UAV_IDS):
            x = row[f"uav_positions_ned[{vi}].x"]
            y = row[f"uav_positions_ned[{vi}].y"]
            ax.scatter(x, y, color=UAV_COLORS[uid], s=120, zorder=5,
                       label=UAV_LABELS[uid])
            ax.annotate(UAV_LABELS[uid], (x, y), textcoords="offset points",
                        xytext=(5, 5), fontsize=8)
        # Payload
        px = row["payload_position_ned.x"]
        py = row["payload_position_ned.y"]
        ax.scatter(px, py, color=PAYLOAD_COLOR, s=150, marker='D', zorder=5, label="Payload")
        # Target
        tx = row["payload_target_ned.x"]
        ty = row["payload_target_ned.y"]
        ax.scatter(tx, ty, color='purple', s=100, marker='*', zorder=5, label="Target")
        if show_virtual_nodes:
            for vi in range(node_count):
                vx = row[f"{prefix}[{vi}].x"]
                vy = row[f"{prefix}[{vi}].y"]
                ax.scatter(vx, vy, color='gray', s=40, marker='^', alpha=0.6, zorder=4)

        ax.set_title(f"t = {ts:.1f} s", fontsize=10)
        ax.grid(True, alpha=0.3); ax.set_aspect('equal')
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")

    # One legend in last subplot
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower right', fontsize=9)
    fig.suptitle("Formation Shape Snapshots (XY plane)", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "15_formation_snapshots.png")

# ── 16. Speed (|V|) over time ─────────────────────────────────────────────────
def plot_speed():
    print("[16] Speed |V| over time")
    fig, ax = plt.subplots(figsize=(14, 5))

    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None: continue
        spd = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
        ax.plot(df["t"], spd, color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])

    payload_df = load_payload_kinematics()
    if payload_df is not None and has_columns(payload_df, "vx", "vy", "vz"):
        pspd = np.sqrt(payload_df["vx"]**2 + payload_df["vy"]**2 + payload_df["vz"]**2)
        ax.plot(payload_df["t"], pspd, color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")

    ax.set_xlabel("Time (s)"); ax.set_ylabel("|V| (m/s)")
    ax.set_title("Total Speed vs Time", fontsize=14, fontweight='bold')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "16_speed.png")

# ── 17. Altitude profile ──────────────────────────────────────────────────────
def plot_altitude():
    print("[17] Altitude profile")
    fig, ax = plt.subplots(figsize=(14, 5))

    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None: continue
        ax.plot(df["t"], -df["z"], color=UAV_COLORS[uid], lw=1.5, label=UAV_LABELS[uid])

    payload_df = load_payload_kinematics()
    if payload_df is not None:
        ax.plot(payload_df["t"], -payload_df["z"],
                color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")
    target_df = load_payload_target()
    if target_df is not None:
        ax.plot(target_df["t"], -target_df["z"],
                color='purple', lw=1.0, ls=':', label="Target altitude")

    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude (m, up+)")
    ax.set_title("Altitude Profile vs Time", fontsize=14, fontweight='bold')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "17_altitude_profile.png")

# ── 18. UAV-to-virtual projection gap ─────────────────────────────────────────
def plot_uav_virtual_error():
    print("[18] UAV-to-virtual projection gap")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
        return
    if not has_columns(
        dp,
        "virtual_positions_ned[0].x",
        "virtual_positions_ned[0].y",
        "virtual_positions_ned[0].z",
    ):
        print("   virtual projection data not available, skipping")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    for vi, uid in enumerate(UAV_IDS):
        ex = dp[f"uav_positions_ned[{vi}].x"] - dp[f"virtual_positions_ned[{vi}].x"]
        ey = dp[f"uav_positions_ned[{vi}].y"] - dp[f"virtual_positions_ned[{vi}].y"]
        ez = dp[f"uav_positions_ned[{vi}].z"] - dp[f"virtual_positions_ned[{vi}].z"]
        err = np.sqrt(ex**2 + ey**2 + ez**2)
        axes[0].plot(dp["t"], ex,  color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])
        axes[1].plot(dp["t"], ey,  color=UAV_COLORS[uid], lw=1.2)
        axes[2].plot(dp["t"], err, color=UAV_COLORS[uid], lw=1.2)

    for ax, lbl in zip(axes, ["ΔX (m)", "ΔY (m)", "|ΔPos| (m)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3); ax.axhline(0, color='k', lw=0.5)
    axes[0].legend(fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("UAV-to-Virtual Projection Gap (Geometry, Not Tracking Error)",
                 fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "18_uav_virtual_error.png")

# ── 18b. Virtual-node rest_length error ───────────────────────────────────────
def plot_formation_rest_length_error():
    """Plot virtual-node structural distance error: |q_i-q_j| - rest_length."""
    print("[18b] Virtual-node rest_length error")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
        return
    prefix, node_count = planner_node_layout(dp)
    if prefix is None:
        print("   planner node positions not available, skipping")
        return
    rl = planner_rest_length_matrix(dp, node_count)
    if rl is None:
        print("   missing rest-length matrix, skipping")
        return

    pair_defs = planner_rest_length_pairs(node_count)
    colors = ['#8E44AD', '#16A085', '#D35400', '#E74C3C', '#2ECC71', '#3498DB',
              '#F39C12', '#1ABC9C', '#9B59B6', '#95A5A6']

    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    ax_err, ax_abs = axes

    for ((i, j), lbl), col in zip(pair_defs, colors):
        ref = rl[i, j]
        if ref <= 0:
            continue
        p_i = virtual_node_position(dp, i)
        p_j = virtual_node_position(dp, j)
        actual = np.sqrt((p_i[0] - p_j[0])**2 + (p_i[1] - p_j[1])**2 + (p_i[2] - p_j[2])**2)
        err = actual - ref
        ax_err.plot(dp["t"], err, color=col, lw=1.2, label=f"{lbl} (ref={ref:.2f})")
        ax_abs.plot(dp["t"], np.abs(err), color=col, lw=1.0, alpha=0.8)

    ax_err.axhline(0, color='k', lw=0.5, linestyle='--')
    ax_err.set_ylabel("Δ (virtual − rest_length) (m)"); ax_err.legend(fontsize=8, ncol=2)
    ax_err.grid(True, alpha=0.3)
    ax_abs.set_ylabel("|Δ| (m)"); ax_abs.set_xlabel("Time (s)")
    ax_abs.grid(True, alpha=0.3)
    title = "Virtual-Node Rest-Length Error" if prefix == "virtual_positions_ned" \
        else "Planner Node Rest-Length Error"
    fig.suptitle(title,
                 fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "18b_formation_rest_length_error.png")

# ── 19. FSM debug: position + attitude tracking (per UAV) ─────────────────────
def plot_fsm_tracking():
    print("[19] FSM position + attitude control outputs")
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fsmpx4_fsm_debug.csv")
        if df is None: continue

        fig = plt.figure(figsize=(16, 12))
        gs  = gridspec.GridSpec(3, 2, figure=fig)

        # commanded position vs actual
        ax0 = fig.add_subplot(gs[0, 0])
        ax0.plot(df["t"], df["cmd_position.x"], 'r--', lw=1.0, label='cmd X')
        ax0.plot(df["t"], df["uav_position.x"], 'r-',  lw=1.2, label='act X')
        ax0.plot(df["t"], df["cmd_position.y"], 'g--', lw=1.0, label='cmd Y')
        ax0.plot(df["t"], df["uav_position.y"], 'g-',  lw=1.2, label='act Y')
        ax0.set_ylabel("Position (m)"); ax0.legend(fontsize=7, ncol=4); ax0.grid(True, alpha=0.3)
        ax0.set_title("Position XY cmd vs actual")

        ax1 = fig.add_subplot(gs[0, 1])
        ax1.plot(df["t"], df["cmd_position.z"], 'b--', lw=1.0, label='cmd Z')
        ax1.plot(df["t"], df["uav_position.z"], 'b-',  lw=1.2, label='act Z')
        ax1.set_ylabel("Z (m)"); ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)
        ax1.set_title("Altitude cmd vs actual")

        # velocity
        ax2 = fig.add_subplot(gs[1, 0])
        for comp, col in zip(['x','y','z'], ['r','g','b']):
            ax2.plot(df["t"], df[f"cmd_velocity.{comp}"], color=col, ls='--', lw=1.0, label=f'cmd {comp}')
            ax2.plot(df["t"], df[f"uav_velocity.{comp}"], color=col, ls='-',  lw=1.2, label=f'act {comp}')
        ax2.set_ylabel("Velocity (m/s)"); ax2.legend(fontsize=7, ncol=6); ax2.grid(True, alpha=0.3)
        ax2.set_title("Velocity cmd vs actual")

        # angular velocity
        ax3 = fig.add_subplot(gs[1, 1])
        for comp, col in zip(['x','y','z'], ['r','g','b']):
            ax3.plot(df["t"], df[f"uav_angular_velocity.{comp}"],
                     color=col, lw=1.2, label=comp)
        ax3.set_ylabel("ω (rad/s)"); ax3.legend(fontsize=9); ax3.grid(True, alpha=0.3)
        ax3.set_title("Angular Velocity")

        # hover thrust
        ax4 = fig.add_subplot(gs[2, 0])
        ax4.plot(df["t"], df["uav_hover_thrust"], color='#8E44AD', lw=1.5)
        ax4.set_ylabel("Hover Thrust"); ax4.grid(True, alpha=0.3)
        ax4.set_title("Estimated Hover Thrust"); ax4.set_xlabel("Time (s)")

        # control thrust
        ax5 = fig.add_subplot(gs[2, 1])
        ax5.plot(df["t"], df["control_thrust"], color='#16A085', lw=1.5)
        ax5.set_ylabel("Control Thrust"); ax5.grid(True, alpha=0.3)
        ax5.set_title("Control Thrust"); ax5.set_xlabel("Time (s)")

        fig.suptitle(f"FSM Debug – {UAV_LABELS[uid]}", fontsize=14, fontweight='bold')
        fig.tight_layout()
        savefig(fig, f"19_fsm_debug_uav{uid}.png")

# ── 20. Summary dashboard ─────────────────────────────────────────────────────
def plot_dashboard():
    print("[20] Summary dashboard")
    dp = load("px4_1_swarm_planner_debug.csv")
    payload_df = load_payload_kinematics()
    fig = plt.figure(figsize=(18, 10))
    gs  = gridspec.GridSpec(2, 4, figure=fig, hspace=0.45, wspace=0.35)

    # Top-down trajectory (XY)
    ax3d = fig.add_subplot(gs[0, 0])
    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None: continue
        ax3d.plot(df["x"], df["y"], color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
    if payload_df is not None:
        ax3d.plot(payload_df["x"], payload_df["y"],
                  color=PAYLOAD_COLOR, lw=1.2, ls='--', label="Payload")
    ax3d.set_title("Top-down Trajectory (XY)", fontsize=10)
    ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)")
    ax3d.legend(fontsize=7); ax3d.grid(True, alpha=0.3); ax3d.set_aspect('equal')

    # altitude
    ax_alt = fig.add_subplot(gs[0, 1])
    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None: continue
        ax_alt.plot(df["t"], -df["z"], color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
    if payload_df is not None:
        ax_alt.plot(payload_df["t"], -payload_df["z"],
                    color=PAYLOAD_COLOR, lw=1.2, ls='--', label="Payload")
    ax_alt.set_title("Altitude (m)"); ax_alt.legend(fontsize=7); ax_alt.grid(True, alpha=0.3)

    # payload error
    ax_err = fig.add_subplot(gs[0, 2])
    if dp is not None:
        err = np.sqrt((dp["payload_position_ned.x"]-dp["payload_target_ned.x"])**2 +
                      (dp["payload_position_ned.y"]-dp["payload_target_ned.y"])**2 +
                      (dp["payload_position_ned.z"]-dp["payload_target_ned.z"])**2)
        ax_err.plot(dp["t"], err, color='#8E44AD', lw=1.2)
        ax_err.fill_between(dp["t"], 0, err, color='#8E44AD', alpha=0.2)
    ax_err.set_title("Payload Tracking Error (m)"); ax_err.grid(True, alpha=0.3)

    # speed
    ax_spd = fig.add_subplot(gs[0, 3])
    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None: continue
        spd = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
        ax_spd.plot(df["t"], spd, color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
    if payload_df is not None and has_columns(payload_df, "vx", "vy", "vz"):
        pspd = np.sqrt(payload_df["vx"]**2 + payload_df["vy"]**2 + payload_df["vz"]**2)
        ax_spd.plot(payload_df["t"], pspd, color=PAYLOAD_COLOR, lw=1.2, ls='--', label="Payload")
    ax_spd.set_title("Speed (m/s)"); ax_spd.legend(fontsize=7); ax_spd.grid(True, alpha=0.3)

    # beta
    ax_beta = fig.add_subplot(gs[1, 0])
    if dp is not None:
        if has_columns(dp, "beta[0]", "beta[1]", "beta[2]"):
            for i, col in enumerate(['#E74C3C', '#2ECC71', '#3498DB']):
                ax_beta.plot(dp["t"], dp[f"beta[{i}]"], color=col, lw=1.0, label=f"β[{i}]")
            ax_beta.set_title("Beta Coefficients")
        elif has_columns(dp, "tracking_input.x", "tracking_input.y", "tracking_input.z"):
            ax_beta.plot(dp["t"], dp["tracking_input.x"], color='#E74C3C', lw=1.0, label='tx')
            ax_beta.plot(dp["t"], dp["tracking_input.y"], color='#2ECC71', lw=1.0, label='ty')
            ax_beta.plot(dp["t"], dp["tracking_input.z"], color='#3498DB', lw=1.0, label='tz')
            ax_beta.axhline(0, color='k', lw=0.5)
            ax_beta.set_title("Tracking Input")
        else:
            ax_beta.text(0.5, 0.5, "not available", ha='center', va='center',
                         transform=ax_beta.transAxes, fontsize=9)
            ax_beta.set_title("Legacy Beta / Tracking Input")
    add_legend_if_handles(ax_beta, fontsize=7)
    ax_beta.grid(True, alpha=0.3)

    # inter-UAV dist
    ax_dist = fig.add_subplot(gs[1, 1])
    dfs = {}
    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is not None: dfs[uid] = df
    pairs = [(1,2,'#8E44AD'),(1,3,'#16A085'),(2,3,'#D35400')]
    for (i, j, col) in pairs:
        if i not in dfs or j not in dfs: continue
        a, b = dfs[i], dfs[j]
        t_c = np.linspace(max(a["t"].min(),b["t"].min()), min(a["t"].max(),b["t"].max()), 1000)
        d = np.sqrt((np.interp(t_c,a["t"],a["x"])-np.interp(t_c,b["t"],b["x"]))**2 +
                    (np.interp(t_c,a["t"],a["y"])-np.interp(t_c,b["t"],b["y"]))**2 +
                    (np.interp(t_c,a["t"],a["z"])-np.interp(t_c,b["t"],b["z"]))**2)
        ax_dist.plot(t_c, d, color=col, lw=1.0, label=f"{i}↔{j}")
    ax_dist.set_title("Inter-UAV Distance (m)"); ax_dist.legend(fontsize=7); ax_dist.grid(True, alpha=0.3)

    # attitude roll/pitch
    ax_att = fig.add_subplot(gs[1, 2])
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_attitude.csv")
        if df is None: continue
        roll, pitch, _ = quat_to_euler(df["q[0]"], df["q[1]"], df["q[2]"], df["q[3]"])
        ax_att.plot(df["t"], roll,  color=UAV_COLORS[uid], lw=0.8, ls='-', alpha=0.8)
        ax_att.plot(df["t"], pitch, color=UAV_COLORS[uid], lw=0.8, ls='--', alpha=0.8)
    ax_att.set_title("Roll (–) Pitch (--) (°)"); ax_att.grid(True, alpha=0.3)

    # desired accel magnitude
    ax_acc = fig.add_subplot(gs[1, 3])
    have_acc = False
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_planner_desired_acceleration.csv")
        if df is not None and has_columns(df, "vector.x", "vector.y", "vector.z"):
            mag = np.sqrt(df["vector.x"]**2 + df["vector.y"]**2 + df["vector.z"]**2)
            ax_acc.plot(df["t"], mag, color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
            have_acc = True
            continue
        df = load(f"px4_{uid}_swarm_planner_debug.csv")
        if df is None or not has_columns(df, "desired_acceleration.x", "desired_acceleration.y", "desired_acceleration.z"):
            continue
        mag = np.sqrt(df["desired_acceleration.x"]**2 + df["desired_acceleration.y"]**2 + df["desired_acceleration.z"]**2)
        ax_acc.plot(df["t"], mag, color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
        have_acc = True
    ax_acc.set_title("|Desired Accel| (m/s²)")
    if not have_acc:
        ax_acc.text(0.5, 0.5, "not available", ha='center', va='center',
                    transform=ax_acc.transAxes, fontsize=9)
    add_legend_if_handles(ax_acc, fontsize=7)
    ax_acc.grid(True, alpha=0.3)

    fig.suptitle("Swarm Flight – Summary Dashboard", fontsize=16, fontweight='bold')
    savefig(fig, "00_summary_dashboard.png", dpi=120)


def compute_inter_uav_distance_series():
    dfs = {}
    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is not None:
            dfs[uid] = df

    series = []
    for i, j, color in [(1, 2, '#8E44AD'), (1, 3, '#16A085'), (2, 3, '#D35400')]:
        if i not in dfs or j not in dfs:
            continue
        a, b = dfs[i], dfs[j]
        t_common = np.linspace(max(a["t"].min(), b["t"].min()),
                               min(a["t"].max(), b["t"].max()), 1200)
        dist = np.sqrt((np.interp(t_common, a["t"], a["x"]) - np.interp(t_common, b["t"], b["x"]))**2 +
                       (np.interp(t_common, a["t"], a["y"]) - np.interp(t_common, b["t"], b["y"]))**2 +
                       (np.interp(t_common, a["t"], a["z"]) - np.interp(t_common, b["t"], b["z"]))**2)
        series.append((f"UAV-{i}↔UAV-{j}", color, t_common, dist))
    return series


def compute_rest_length_error_summary():
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        return None
    prefix, node_count = planner_node_layout(dp)
    if prefix is None:
        return None
    rl = planner_rest_length_matrix(dp, node_count)
    if rl is None:
        return None

    abs_errors = []
    for (i, j), _ in planner_rest_length_pairs(node_count):
        ref = rl[i, j]
        if ref <= 0:
            continue
        p_i = virtual_node_position(dp, i)
        p_j = virtual_node_position(dp, j)
        actual = np.sqrt((p_i[0] - p_j[0])**2 + (p_i[1] - p_j[1])**2 + (p_i[2] - p_j[2])**2)
        abs_errors.append(np.abs(actual - ref))

    if not abs_errors:
        return None

    abs_errors = np.vstack(abs_errors)
    return dp["t"], abs_errors.mean(axis=0), abs_errors.max(axis=0)


def plot_paper_overview():
    print("[P1] Paper overview")
    dp = load("px4_1_swarm_planner_debug.csv")
    payload_df = load_payload_kinematics()
    target_df = load_payload_target()
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    ax_xy, ax_alt = axes

    for uid in UAV_IDS:
        df = load_uav_kinematics(uid)
        if df is None:
            continue
        ax_xy.plot(df["x"], df["y"], color=UAV_COLORS[uid], lw=1.6, label=UAV_LABELS[uid])
        ax_xy.scatter(df["x"].iloc[0], df["y"].iloc[0], color=UAV_COLORS[uid], marker='o', s=40, zorder=6)
        ax_xy.scatter(df["x"].iloc[-1], df["y"].iloc[-1], color=UAV_COLORS[uid], marker='*', s=100, zorder=6)
        ax_alt.plot(df["t"], -df["z"], color=UAV_COLORS[uid], lw=1.5, label=UAV_LABELS[uid])

    if payload_df is not None:
        ax_xy.plot(payload_df["x"], payload_df["y"],
                   color=PAYLOAD_COLOR, lw=2.0, ls='--', label="Payload")
        ax_alt.plot(payload_df["t"], -payload_df["z"],
                    color=PAYLOAD_COLOR, lw=2.0, ls='--', label="Payload")
    if target_df is not None:
        plot_target_xy(ax_xy, target_df, color='black', label="Target")
        ax_alt.plot(target_df["t"], -target_df["z"],
                    color='black', lw=1.2, ls=':', label="Target")

    ax_xy.set_title("Top-Down Transport Trajectory", fontsize=13, fontweight='bold')
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect('equal')
    ax_xy.legend(fontsize=9, ncol=2)

    ax_alt.set_title("Altitude Profile", fontsize=13, fontweight='bold')
    ax_alt.set_xlabel("Time (s)")
    ax_alt.set_ylabel("Altitude (m)")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend(fontsize=9, ncol=2)

    fig.tight_layout()
    savefig(fig, "paper_01_overview.png", dpi=220)


def plot_paper_payload_tracking():
    print("[P2] Paper payload tracking")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
        return

    ex = dp["payload_position_ned.x"] - dp["payload_target_ned.x"]
    ey = dp["payload_position_ned.y"] - dp["payload_target_ned.y"]
    ez = dp["payload_position_ned.z"] - dp["payload_target_ned.z"]
    err = np.sqrt(ex**2 + ey**2 + ez**2)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(dp["t"], ex, color='#E74C3C', lw=1.4, label='e_x')
    axes[0].plot(dp["t"], ey, color='#2ECC71', lw=1.4, label='e_y')
    axes[0].plot(dp["t"], ez, color='#3498DB', lw=1.4, label='e_z')
    axes[0].axhline(0, color='k', lw=0.7, ls='--')
    axes[0].set_ylabel("Component error (m)")
    axes[0].set_title("Payload Tracking Error Components", fontsize=13, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=9, ncol=3)

    axes[1].plot(dp["t"], err, color='#8E44AD', lw=1.8)
    axes[1].fill_between(dp["t"], 0, err, color='#8E44AD', alpha=0.18)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel(r"$\|e_p\|$ (m)")
    axes[1].set_title("Payload Position Error Norm", fontsize=13, fontweight='bold')
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    savefig(fig, "paper_02_payload_tracking.png", dpi=220)


def plot_paper_formation_quality():
    print("[P3] Paper formation quality")
    dist_series = compute_inter_uav_distance_series()
    rest_summary = compute_rest_length_error_summary()
    if not dist_series and rest_summary is None:
        print("   no data, skipping")
        return

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)

    for label, color, t_common, dist in dist_series:
        axes[0].plot(t_common, dist, color=color, lw=1.5, label=label)
    axes[0].set_title("Inter-UAV Distances", fontsize=13, fontweight='bold')
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Distance (m)")
    axes[0].grid(True, alpha=0.3)
    if dist_series:
        axes[0].legend(fontsize=9, ncol=3)

    if rest_summary is not None:
        t, mean_abs, max_abs = rest_summary
        axes[1].plot(t, mean_abs, color='#2980B9', lw=1.8, label='mean |Δrest|')
        axes[1].plot(t, max_abs, color='#C0392B', lw=1.6, label='max |Δrest|')
        axes[1].fill_between(t, 0, mean_abs, color='#2980B9', alpha=0.12)
        axes[1].fill_between(t, mean_abs, max_abs, color='#C0392B', alpha=0.08)
        axes[1].legend(fontsize=9)
    else:
        axes[1].text(0.5, 0.5, "rest-length summary not available",
                     ha='center', va='center', transform=axes[1].transAxes)
    axes[1].set_title("Virtual Structure Error Summary", fontsize=13, fontweight='bold')
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Absolute error (m)")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    savefig(fig, "paper_03_formation_quality.png", dpi=220)


def plot_paper_control_effort():
    print("[P4] Paper control effort")
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    have_acc = False
    have_thr = False

    for uid in UAV_IDS:
        df_acc = load(f"px4_{uid}_swarm_planner_debug.csv")
        if df_acc is not None and has_columns(df_acc, "desired_acceleration.x", "desired_acceleration.y", "desired_acceleration.z"):
            mag = np.sqrt(df_acc["desired_acceleration.x"]**2 + df_acc["desired_acceleration.y"]**2 + df_acc["desired_acceleration.z"]**2)
            axes[0].plot(df_acc["t"], mag, color=UAV_COLORS[uid], lw=1.5, label=UAV_LABELS[uid])
            have_acc = True

        df_thr = load(f"px4_{uid}_fmu_in_vehicle_attitude_setpoint.csv")
        if df_thr is not None and has_columns(df_thr, "thrust_body[2]"):
            axes[1].plot(df_thr["t"], -df_thr["thrust_body[2]"],
                         color=UAV_COLORS[uid], lw=1.5, label=UAV_LABELS[uid])
            have_thr = True

    if have_acc:
        axes[0].legend(fontsize=9, ncol=3)
    else:
        axes[0].text(0.5, 0.5, "desired acceleration not available",
                     ha='center', va='center', transform=axes[0].transAxes)
    axes[0].set_ylabel(r"$\|a_d\|$ (m/s$^2$)")
    axes[0].set_title("Planner Command Magnitude", fontsize=13, fontweight='bold')
    axes[0].grid(True, alpha=0.3)

    if have_thr:
        axes[1].legend(fontsize=9, ncol=3)
    else:
        axes[1].text(0.5, 0.5, "thrust command not available",
                     ha='center', va='center', transform=axes[1].transAxes)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Normalized thrust")
    axes[1].set_title("Low-Level Control Effort", fontsize=13, fontweight='bold')
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    savefig(fig, "paper_04_control_effort.png", dpi=220)

# ── main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate swarm flight analysis plots from CSV data."
    )
    parser.add_argument(
        "analysis_dir",
        type=str,
        help="Path to the analysis directory (must contain a 'csv' subfolder). "
             "Plots will be written to analysis_dir/plots/",
    )
    parser.add_argument(
        "--profile",
        choices=["full", "paper"],
        default="full",
        help="Plot set to generate: full diagnostic set or a compact paper-ready set.",
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
        help="Optional analysis time window: full bag, first common valid pose, or CMD_CTRL only.",
    )
    args = parser.parse_args()
    analysis_dir = os.path.abspath(args.analysis_dir)
    if not os.path.isdir(analysis_dir):
        print(f"Error: not a directory: {analysis_dir}", file=sys.stderr)
        sys.exit(1)
    # set global paths used by helper functions
    CSV_DIR = os.path.join(analysis_dir, "csv")
    if args.output_dir:
        OUT_DIR = os.path.abspath(args.output_dir)
    else:
        OUT_DIR = os.path.join(analysis_dir, "plots" if args.profile == "full" else "paper_plots")
    if not os.path.isdir(CSV_DIR):
        print(f"Error: CSV folder not found: {CSV_DIR}", file=sys.stderr)
        sys.exit(1)
    window_start_s = configure_time_window(args.window)
    os.makedirs(OUT_DIR, exist_ok=True)
    clear_png_outputs()
    print(f"CSV dir:  {CSV_DIR}")
    print(f"Output:   {OUT_DIR}\n")
    print(f"Time origin: global bag start")
    print(f"Window:     {args.window} (start = {window_start_s:.3f} s)\n")
    if args.profile == "paper":
        plot_paper_overview()
        plot_paper_payload_tracking()
        plot_paper_formation_quality()
        plot_paper_control_effort()
        print("\nDone! Paper plots saved to:", OUT_DIR)
    else:
        plot_dashboard()
        plot_3d_trajectory()
        plot_xy_trajectory()
        plot_position_time()
        plot_velocity_time()
        plot_attitude()
        plot_attitude_tracking()
        plot_thrust()
        plot_inter_uav_distances()
        plot_payload_tracking_error()
        plot_desired_acceleration()
        plot_beta()
        plot_virtual_positions()
        plot_control_forces()
        plot_status_flags()
        plot_formation_snapshots()
        plot_speed()
        plot_altitude()
        plot_uav_virtual_error()
        plot_formation_rest_length_error()
        plot_fsm_tracking()
        print("\nDone! All plots saved to:", OUT_DIR)
