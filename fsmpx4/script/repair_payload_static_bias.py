#!/usr/bin/env python3
"""
Create a static-bias-corrected copy of an analysis directory.

The correction is estimated from the late-stage mean payload tracking error
recorded in planner debug CSVs. The raw bag and original analysis directory
remain untouched.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import numpy as np
import pandas as pd


PLANNER_DEBUG_GLOB = "px4_*_swarm_planner_debug.csv"
PAYLOAD_LOCAL_POSITION_NAME = "px4_4_fmu_out_vehicle_local_position.csv"
PAYLOAD_RVIZ_POSE_NAME = "swarm_rviz_payload_pose.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Repair payload static bias in an exported swarm analysis directory."
    )
    parser.add_argument(
        "analysis_dir",
        help="Path to an analysis directory that contains csv/ and summary.txt.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Destination analysis directory (default: <analysis_dir>_static_bias_fixed).",
    )
    parser.add_argument(
        "--tail-fraction",
        type=float,
        default=0.10,
        help="Late-stage fraction used to estimate the mean static bias (default: %(default)s).",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite the output directory if it already exists.",
    )
    return parser.parse_args()


def require_columns(df: pd.DataFrame, path: Path, *columns: str) -> None:
    missing = [column for column in columns if column not in df.columns]
    if missing:
        raise ValueError(f"{path} is missing columns: {', '.join(missing)}")


def estimate_payload_bias(planner_csv: Path, tail_fraction: float) -> np.ndarray:
    df = pd.read_csv(planner_csv)
    require_columns(
        df,
        planner_csv,
        "_bag_time_s",
        "payload_position_ned.x",
        "payload_position_ned.y",
        "payload_position_ned.z",
        "payload_target_ned.x",
        "payload_target_ned.y",
        "payload_target_ned.z",
    )

    if not 0.0 < tail_fraction <= 1.0:
        raise ValueError(f"--tail-fraction must be in (0, 1], got {tail_fraction}")

    threshold = float(df["_bag_time_s"].quantile(1.0 - tail_fraction))
    late = df.loc[df["_bag_time_s"] >= threshold]
    if late.empty:
        raise ValueError(f"{planner_csv} does not have a non-empty late-stage window")

    bias = np.array(
        [
            float((late["payload_position_ned.x"] - late["payload_target_ned.x"]).mean()),
            float((late["payload_position_ned.y"] - late["payload_target_ned.y"]).mean()),
            float((late["payload_position_ned.z"] - late["payload_target_ned.z"]).mean()),
        ]
    )
    return bias


def recompute_virtual_payload_state(df: pd.DataFrame, original_payload_z: pd.Series) -> None:
    payload_axes = "xyz"
    payload_cols = [f"payload_position_ned.{axis}" for axis in payload_axes]
    payload_vel_cols = [f"payload_velocity_ned.{axis}" for axis in payload_axes]
    hub_cols = [f"virtual_positions_ned[3].{axis}" for axis in payload_axes]
    payload_virtual_cols = [f"virtual_positions_ned[4].{axis}" for axis in payload_axes]

    if not all(column in df.columns for column in payload_cols + hub_cols + payload_virtual_cols):
        return

    corrected_payload = df[payload_cols].to_numpy(dtype=float)
    original_hub_z = df["virtual_positions_ned[3].z"].to_numpy(dtype=float)
    h_u = original_payload_z.to_numpy(dtype=float) - original_hub_z

    for axis_index, axis in enumerate(payload_axes):
        df[f"virtual_positions_ned[4].{axis}"] = corrected_payload[:, axis_index]

    df["virtual_positions_ned[3].x"] = corrected_payload[:, 0]
    df["virtual_positions_ned[3].y"] = corrected_payload[:, 1]
    df["virtual_positions_ned[3].z"] = corrected_payload[:, 2] - h_u

    if all(column in df.columns for column in payload_vel_cols):
        corrected_payload_vel = df[payload_vel_cols].to_numpy(dtype=float)
        for axis_index, axis in enumerate(payload_axes):
            df[f"virtual_velocities_ned[4].{axis}"] = corrected_payload_vel[:, axis_index]
            df[f"virtual_velocities_ned[3].{axis}"] = corrected_payload_vel[:, axis_index]
    else:
        corrected_payload_vel = None

    for uav_index in range(3):
        uav_pos_cols = [f"uav_positions_ned[{uav_index}].{axis}" for axis in payload_axes]
        virt_pos_cols = [f"virtual_positions_ned[{uav_index}].{axis}" for axis in payload_axes]
        beta_col = f"beta[{uav_index}]"
        if not all(column in df.columns for column in uav_pos_cols + virt_pos_cols):
            continue

        uav_pos = df[uav_pos_cols].to_numpy(dtype=float)
        dz = np.abs(corrected_payload[:, 2] - uav_pos[:, 2])
        safe_dz = np.maximum(dz, 1e-4)
        alpha = h_u / safe_dz
        corrected_virtual = corrected_payload + alpha[:, None] * (uav_pos - corrected_payload)

        for axis_index, axis in enumerate(payload_axes):
            df[f"virtual_positions_ned[{uav_index}].{axis}"] = corrected_virtual[:, axis_index]

        if beta_col in df.columns:
            safe_h_u = np.maximum(np.abs(h_u), 1e-6)
            df[beta_col] = dz / safe_h_u

        uav_vel_cols = [f"uav_velocities_ned[{uav_index}].{axis}" for axis in payload_axes]
        virt_vel_cols = [f"virtual_velocities_ned[{uav_index}].{axis}" for axis in payload_axes]
        if corrected_payload_vel is None or not all(column in df.columns for column in uav_vel_cols + virt_vel_cols):
            continue

        uav_vel = df[uav_vel_cols].to_numpy(dtype=float)
        corrected_virtual_vel = corrected_payload_vel + alpha[:, None] * (uav_vel - corrected_payload_vel)
        for axis_index, axis in enumerate(payload_axes):
            df[f"virtual_velocities_ned[{uav_index}].{axis}"] = corrected_virtual_vel[:, axis_index]


def repair_planner_debug_csv(src: Path, dst: Path, bias_ned: np.ndarray) -> None:
    df = pd.read_csv(src)
    payload_cols = [f"payload_position_ned.{axis}" for axis in "xyz"]
    require_columns(df, src, *payload_cols)

    original_payload_z = df["payload_position_ned.z"].copy()
    for axis_index, axis in enumerate("xyz"):
        df[f"payload_position_ned.{axis}"] = df[f"payload_position_ned.{axis}"] - bias_ned[axis_index]

    recompute_virtual_payload_state(df, original_payload_z)
    df.to_csv(dst, index=False)


def repair_payload_local_position_csv(src: Path, dst: Path, bias_ned: np.ndarray) -> None:
    df = pd.read_csv(src)
    if all(column in df.columns for column in ("x", "y", "z")):
        df["x"] = df["x"] - bias_ned[0]
        df["y"] = df["y"] - bias_ned[1]
        df["z"] = df["z"] - bias_ned[2]
    df.to_csv(dst, index=False)


def repair_payload_rviz_pose_csv(src: Path, dst: Path, bias_ned: np.ndarray) -> None:
    df = pd.read_csv(src)
    if all(column in df.columns for column in ("pose.position.x", "pose.position.y", "pose.position.z")):
        # ENU = [east, north, up] = [ned.y, ned.x, -ned.z]
        df["pose.position.x"] = df["pose.position.x"] - bias_ned[1]
        df["pose.position.y"] = df["pose.position.y"] - bias_ned[0]
        df["pose.position.z"] = df["pose.position.z"] + bias_ned[2]
    df.to_csv(dst, index=False)


def rewrite_summary(summary_src: Path, summary_dst: Path, csv_src: Path, csv_dst: Path) -> None:
    text = summary_src.read_text(encoding="utf-8")
    text = text.replace(str(csv_src), str(csv_dst))
    summary_dst.write_text(text, encoding="utf-8")


def write_report(report_path: Path, source_dir: Path, output_dir: Path, tail_fraction: float, bias_ned: np.ndarray) -> None:
    report = "\n".join(
        [
            "Payload static-bias repair",
            f"source_analysis_dir: {source_dir}",
            f"output_analysis_dir: {output_dir}",
            f"tail_fraction: {tail_fraction:.3f}",
            f"payload_bias_ned.x: {bias_ned[0]:.6f}",
            f"payload_bias_ned.y: {bias_ned[1]:.6f}",
            f"payload_bias_ned.z: {bias_ned[2]:.6f}",
            "modified_csvs:",
            "  - px4_*_swarm_planner_debug.csv",
            f"  - {PAYLOAD_LOCAL_POSITION_NAME}",
            f"  - {PAYLOAD_RVIZ_POSE_NAME}",
        ]
    )
    report_path.write_text(report + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    source_dir = Path(args.analysis_dir).expanduser().resolve()
    if not source_dir.is_dir():
        raise FileNotFoundError(f"analysis directory not found: {source_dir}")

    csv_src = source_dir / "csv"
    if not csv_src.is_dir():
        raise FileNotFoundError(f"csv directory not found: {csv_src}")

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else source_dir.parent / f"{source_dir.name}_static_bias_fixed"
    )

    if output_dir.exists():
        if not args.force:
            raise FileExistsError(f"output directory already exists: {output_dir}")
        shutil.rmtree(output_dir)

    csv_dst = output_dir / "csv"
    csv_dst.mkdir(parents=True, exist_ok=True)

    source_planner_csv = csv_src / "px4_1_swarm_planner_debug.csv"
    if not source_planner_csv.is_file():
        raise FileNotFoundError(f"planner debug CSV not found: {source_planner_csv}")

    bias_ned = estimate_payload_bias(source_planner_csv, args.tail_fraction)

    for src_csv in sorted(csv_src.glob("*.csv")):
        dst_csv = csv_dst / src_csv.name
        if src_csv.match(PLANNER_DEBUG_GLOB):
            repair_planner_debug_csv(src_csv, dst_csv, bias_ned)
        elif src_csv.name == PAYLOAD_LOCAL_POSITION_NAME:
            repair_payload_local_position_csv(src_csv, dst_csv, bias_ned)
        elif src_csv.name == PAYLOAD_RVIZ_POSE_NAME:
            repair_payload_rviz_pose_csv(src_csv, dst_csv, bias_ned)
        else:
            shutil.copy2(src_csv, dst_csv)

    summary_src = source_dir / "summary.txt"
    if summary_src.is_file():
        rewrite_summary(summary_src, output_dir / "summary.txt", csv_src, csv_dst)

    write_report(
        output_dir / "static_bias_report.txt",
        source_dir,
        output_dir,
        args.tail_fraction,
        bias_ned,
    )

    print(f"source_analysis_dir={source_dir}")
    print(f"output_analysis_dir={output_dir}")
    print(f"payload_bias_ned=({bias_ned[0]:.6f}, {bias_ned[1]:.6f}, {bias_ned[2]:.6f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
