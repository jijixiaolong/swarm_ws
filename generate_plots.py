#!/usr/bin/env python3
"""
Comprehensive swarm flight data visualization script.
Generates detailed plots from CSV data recorded during swarm operation.
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from scipy.spatial.transform import Rotation

import warnings
warnings.filterwarnings('ignore')

# ── paths ──────────────────────────────────────────────────────────────────────
CSV_DIR  = os.path.dirname(os.path.abspath(__file__)) + "/csv"
OUT_DIR  = os.path.dirname(os.path.abspath(__file__)) + "/plots"
os.makedirs(OUT_DIR, exist_ok=True)

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
        t0 = df["_bag_time_s"].iloc[0]
        df["t"] = df["_bag_time_s"] - t0
    return df

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
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        c = UAV_COLORS[uid]; lbl = UAV_LABELS[uid]
        axes[0].plot(df["x"],  df["y"],  color=c, lw=1.3, label=lbl)
        axes[1].plot(df["x"], -df["z"],  color=c, lw=1.3)
        axes[2].plot(df["y"], -df["z"],  color=c, lw=1.3)
        _scatter_ends(axes[0], df["x"], df["y"],   c)
        _scatter_ends(axes[1], df["x"], -df["z"],  c)
        _scatter_ends(axes[2], df["y"], -df["z"],  c)

    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is not None:
        px = dp["payload_position_ned.x"]
        py = dp["payload_position_ned.y"]
        pz = -dp["payload_position_ned.z"]
        axes[0].plot(px, py, color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")
        axes[1].plot(px, pz, color=PAYLOAD_COLOR, lw=1.5, ls='--')
        axes[2].plot(py, pz, color=PAYLOAD_COLOR, lw=1.5, ls='--')
        axes[0].plot(dp["payload_target_ned.x"], dp["payload_target_ned.y"],
                     color='purple', lw=1.0, ls=':', label="Target")

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
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        lc = colored_line(ax, df["x"].values, df["y"].values, df["t"].values,
                          cmap='plasma', lw=1.8)
        ax.scatter(df["x"].iloc[0],  df["y"].iloc[0],  color=UAV_COLORS[uid],
                   marker='o', s=80, zorder=6, label=f"{UAV_LABELS[uid]} start")
        ax.scatter(df["x"].iloc[-1], df["y"].iloc[-1], color=UAV_COLORS[uid],
                   marker='*', s=140, zorder=6, label=f"{UAV_LABELS[uid]} end")

    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is not None:
        ax.plot(dp["payload_position_ned.x"], dp["payload_position_ned.y"],
                color=PAYLOAD_COLOR, lw=1.8, ls='--', label="Payload", alpha=0.9)
        ax.plot(dp["payload_target_ned.x"], dp["payload_target_ned.y"],
                color='purple', lw=1.2, ls=':', label="Target", alpha=0.8)

    ax.set_xlabel("X NED (m)"); ax.set_ylabel("Y NED (m)")
    ax.set_title("Top-down Trajectory (XY) – color = time", fontsize=14, fontweight='bold')
    ax.legend(fontsize=8, ncol=2); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "02_xy_trajectory.png")

# ── 3. Position vs time (X, Y, Z) ─────────────────────────────────────────────
def plot_position_time():
    print("[3] Position vs time")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    dp = load("px4_1_swarm_planner_debug.csv")

    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        axes[0].plot(df["t"], df["x"],  color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])
        axes[1].plot(df["t"], df["y"],  color=UAV_COLORS[uid], lw=1.2)
        axes[2].plot(df["t"], -df["z"], color=UAV_COLORS[uid], lw=1.2)

    if dp is not None:
        axes[0].plot(dp["t"], dp["payload_position_ned.x"],
                     color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")
        axes[0].plot(dp["t"], dp["payload_target_ned.x"],
                     color='purple', lw=1.0, ls=':', label="Target")
        axes[1].plot(dp["t"], dp["payload_position_ned.y"],
                     color=PAYLOAD_COLOR, lw=1.5, ls='--')
        axes[1].plot(dp["t"], dp["payload_target_ned.y"],
                     color='purple', lw=1.0, ls=':')
        axes[2].plot(dp["t"], -dp["payload_position_ned.z"],
                     color=PAYLOAD_COLOR, lw=1.5, ls='--')
        axes[2].plot(dp["t"], -dp["payload_target_ned.z"],
                     color='purple', lw=1.0, ls=':')

    labels = ["X (m)", "Y (m)", "Z (m, up+)"]
    for ax, lbl in zip(axes, labels):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9, ncol=5)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Position vs Time – UAVs & Payload", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "03_position_time.png")

# ── 4. Velocity vs time ────────────────────────────────────────────────────────
def plot_velocity_time():
    print("[4] Velocity vs time")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        axes[0].plot(df["t"], df["vx"], color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])
        axes[1].plot(df["t"], df["vy"], color=UAV_COLORS[uid], lw=1.2)
        axes[2].plot(df["t"], df["vz"], color=UAV_COLORS[uid], lw=1.2)

    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is not None:
        axes[0].plot(dp["t"], dp["payload_velocity_ned.x"], color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")
        axes[1].plot(dp["t"], dp["payload_velocity_ned.y"], color=PAYLOAD_COLOR, lw=1.5, ls='--')
        axes[2].plot(dp["t"], dp["payload_velocity_ned.z"], color=PAYLOAD_COLOR, lw=1.5, ls='--')

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
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
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

    for uid in UAV_IDS:
        df = load(f"px4_{uid}_planner_desired_acceleration.csv")
        if df is None: continue
        axes[0].plot(df["t"], df["vector.x"], color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])
        axes[1].plot(df["t"], df["vector.y"], color=UAV_COLORS[uid], lw=1.2)
        axes[2].plot(df["t"], df["vector.z"], color=UAV_COLORS[uid], lw=1.2)

    for ax, lbl in zip(axes, ["ax (m/s²)", "ay (m/s²)", "az (m/s²)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3); ax.axhline(0, color='k', lw=0.5)
    axes[0].legend(fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Planner Desired Acceleration vs Time", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "10_desired_acceleration.png")

# ── 11. Beta coefficients ──────────────────────────────────────────────────────
def plot_beta():
    print("[11] Beta coefficients")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    for uid in UAV_IDS:
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None: continue
        for i, ax in enumerate(axes):
            ax.plot(dp["t"], dp[f"beta[{i}]"], color=UAV_COLORS[uid],
                    lw=1.2, label=UAV_LABELS[uid] if i == 0 else "")

    for i, ax in enumerate(axes):
        ax.set_ylabel(f"β[{i}]"); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Beta Coefficients (Force Allocation) vs Time", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "11_beta_coefficients.png")

# ── 12. Virtual positions ──────────────────────────────────────────────────────
def plot_virtual_positions():
    print("[12] Virtual formation positions")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    cmaps = plt.cm.Set1.colors
    for vi in range(5):
        c = cmaps[vi % len(cmaps)]
        axes[0].plot(dp["t"], dp[f"virtual_positions_ned[{vi}].x"],
                     color=c, lw=1.2, label=f"VP-{vi}")
        axes[1].plot(dp["t"], dp[f"virtual_positions_ned[{vi}].y"], color=c, lw=1.2)
        axes[2].plot(dp["t"], dp[f"virtual_positions_ned[{vi}].z"], color=c, lw=1.2)

    for ax, lbl in zip(axes, ["X (m)", "Y (m)", "Z (m)"]):
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9, ncol=5)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Virtual Formation Positions (NED) vs Time", fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "12_virtual_positions.png")

# ── 13. CFO / passive / virtual acceleration ─────────────────────────────────
def plot_control_forces():
    print("[13] Control forces (CFO / passive / virtual)")
    fig, axes = plt.subplots(3, 3, figsize=(16, 10), sharex=True)

    for col, uid in enumerate(UAV_IDS):
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None: continue

        for row, (key, color) in enumerate([
            ("passive_force",       "#E74C3C"),
            ("virtual_acceleration","#2ECC71"),
            ("cfo_acceleration",    "#3498DB"),
        ]):
            for comp, lbl in enumerate(["x", "y", "z"]):
                # stagger x/y/z on same subplot with different styles
                pass

        for row, (key, color) in enumerate([
            ("passive_force",       "#E74C3C"),
            ("virtual_acceleration","#2ECC71"),
            ("cfo_acceleration",    "#3498DB"),
        ]):
            ax = axes[row, col]
            ax.plot(dp["t"], dp[f"{key}.x"], color='#E74C3C', lw=1.0, label='x')
            ax.plot(dp["t"], dp[f"{key}.y"], color='#2ECC71', lw=1.0, label='y')
            ax.plot(dp["t"], dp[f"{key}.z"], color='#3498DB', lw=1.0, label='z')
            ax.grid(True, alpha=0.3); ax.axhline(0, color='k', lw=0.5)
            if col == 0:
                ax.set_ylabel(key.replace('_', '\n'), fontsize=8)
            if row == 0:
                ax.set_title(UAV_LABELS[uid], fontweight='bold')
            if row == 2:
                ax.set_xlabel("Time (s)")

    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Control Forces: Passive / Virtual / CFO Acceleration",
                 fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "13_control_forces.png")

# ── 14. Structure lock + valid status ─────────────────────────────────────────
def plot_status_flags():
    print("[14] Status flags (structure_locked, valid, used_cfo)")
    fig, axes = plt.subplots(3, 1, figsize=(14, 7), sharex=True)
    flags = ["structure_locked", "valid", "used_cfo"]

    for uid in UAV_IDS:
        dp = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp is None: continue
        offset = (uid - 1) * 0.1
        for ax, flag in zip(axes, flags):
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
        # Virtual positions
        for vi in range(5):
            vx = row[f"virtual_positions_ned[{vi}].x"]
            vy = row[f"virtual_positions_ned[{vi}].y"]
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
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        spd = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
        ax.plot(df["t"], spd, color=UAV_COLORS[uid], lw=1.2, label=UAV_LABELS[uid])

    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is not None:
        pspd = np.sqrt(dp["payload_velocity_ned.x"]**2 +
                       dp["payload_velocity_ned.y"]**2 +
                       dp["payload_velocity_ned.z"]**2)
        ax.plot(dp["t"], pspd, color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")

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
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        ax.plot(df["t"], -df["z"], color=UAV_COLORS[uid], lw=1.5, label=UAV_LABELS[uid])

    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is not None:
        ax.plot(dp["t"], -dp["payload_position_ned.z"],
                color=PAYLOAD_COLOR, lw=1.5, ls='--', label="Payload")
        ax.plot(dp["t"], -dp["payload_target_ned.z"],
                color='purple', lw=1.0, ls=':', label="Target altitude")

    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude (m, up+)")
    ax.set_title("Altitude Profile vs Time", fontsize=14, fontweight='bold')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    savefig(fig, "17_altitude_profile.png")

# ── 18. UAV vs virtual position error ─────────────────────────────────────────
def plot_uav_virtual_error():
    print("[18] UAV vs virtual position error")
    dp = load("px4_1_swarm_planner_debug.csv")
    if dp is None:
        print("   no data, skipping")
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
    fig.suptitle("UAV vs Virtual Position Error (Formation Tracking)",
                 fontsize=14, fontweight='bold')
    fig.tight_layout()
    savefig(fig, "18_uav_virtual_error.png")

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
    fig = plt.figure(figsize=(18, 12))
    gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

    # Top-down trajectory (XY)
    ax3d = fig.add_subplot(gs[0, 0])
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        ax3d.plot(df["x"], df["y"], color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
    if dp is not None:
        ax3d.plot(dp["payload_position_ned.x"], dp["payload_position_ned.y"],
                  color=PAYLOAD_COLOR, lw=1.2, ls='--', label="Payload")
    ax3d.set_title("Top-down Trajectory (XY)", fontsize=10)
    ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)")
    ax3d.legend(fontsize=7); ax3d.grid(True, alpha=0.3); ax3d.set_aspect('equal')

    # altitude
    ax_alt = fig.add_subplot(gs[0, 1])
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        ax_alt.plot(df["t"], -df["z"], color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
    if dp is not None:
        ax_alt.plot(dp["t"], -dp["payload_position_ned.z"],
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
    ax_spd = fig.add_subplot(gs[1, 0])
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
        if df is None: continue
        spd = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
        ax_spd.plot(df["t"], spd, color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
    ax_spd.set_title("Speed (m/s)"); ax_spd.legend(fontsize=7); ax_spd.grid(True, alpha=0.3)

    # beta
    ax_beta = fig.add_subplot(gs[1, 1])
    if dp is not None:
        for i, col in enumerate(['#E74C3C', '#2ECC71', '#3498DB']):
            ax_beta.plot(dp["t"], dp[f"beta[{i}]"], color=col, lw=1.0, label=f"β[{i}]")
    ax_beta.set_title("Beta Coefficients"); ax_beta.legend(fontsize=7); ax_beta.grid(True, alpha=0.3)

    # inter-UAV dist
    ax_dist = fig.add_subplot(gs[1, 2])
    dfs = {}
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_local_position.csv")
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
    ax_att = fig.add_subplot(gs[2, 0])
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_fmu_out_vehicle_attitude.csv")
        if df is None: continue
        roll, pitch, _ = quat_to_euler(df["q[0]"], df["q[1]"], df["q[2]"], df["q[3]"])
        ax_att.plot(df["t"], roll,  color=UAV_COLORS[uid], lw=0.8, ls='-', alpha=0.8)
        ax_att.plot(df["t"], pitch, color=UAV_COLORS[uid], lw=0.8, ls='--', alpha=0.8)
    ax_att.set_title("Roll (–) Pitch (--) (°)"); ax_att.grid(True, alpha=0.3)

    # desired accel magnitude
    ax_acc = fig.add_subplot(gs[2, 1])
    for uid in UAV_IDS:
        df = load(f"px4_{uid}_planner_desired_acceleration.csv")
        if df is None: continue
        mag = np.sqrt(df["vector.x"]**2 + df["vector.y"]**2 + df["vector.z"]**2)
        ax_acc.plot(df["t"], mag, color=UAV_COLORS[uid], lw=1.0, label=UAV_LABELS[uid])
    ax_acc.set_title("|Desired Accel| (m/s²)"); ax_acc.legend(fontsize=7); ax_acc.grid(True, alpha=0.3)

    # structure lock
    ax_lock = fig.add_subplot(gs[2, 2])
    for uid in UAV_IDS:
        dp2 = load(f"px4_{uid}_swarm_planner_debug.csv")
        if dp2 is None: continue
        ax_lock.step(dp2["t"], dp2["structure_locked"].astype(float) + (uid-1)*0.1,
                     where='post', color=UAV_COLORS[uid], lw=1.5, label=UAV_LABELS[uid])
    ax_lock.set_title("Structure Locked"); ax_lock.legend(fontsize=7); ax_lock.grid(True, alpha=0.3)

    fig.suptitle("Swarm Flight – Summary Dashboard", fontsize=16, fontweight='bold')
    savefig(fig, "00_summary_dashboard.png", dpi=120)

# ── main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print(f"Output dir: {OUT_DIR}\n")
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
    plot_fsm_tracking()
    print("\nDone! All plots saved to:", OUT_DIR)
