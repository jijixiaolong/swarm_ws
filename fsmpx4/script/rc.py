#!/usr/bin/env python3
"""
Interactive GUI publisher for px4_msgs/msg/ManualControlSetpoint.

This tool mimics a simple RC transmitter so FSMPX4 can switch MANUAL/OFFBOARD/HOVER/CMD
states without real hardware. Sliders control the four sticks, buttons select modes.
"""

import argparse
import sys
import threading
from typing import Dict, List, Sequence

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from px4_msgs.msg import ManualControlSetpoint


DEFAULT_TOPICS = (
    "/rc/manual_control_setpoint",
    "/px4_1/rc/manual_control_setpoint",
    "/px4_2/rc/manual_control_setpoint",
    "/px4_3/rc/manual_control_setpoint",
)


def _normalize_topic(topic: str) -> str:
    normalized = topic.strip()
    if not normalized:
        return ""
    if not normalized.startswith("/"):
        normalized = f"/{normalized}"
    return normalized


def _parse_topics(items: Sequence[str]) -> List[str]:
    topics: List[str] = []
    seen = set()
    for item in items:
        for raw_topic in item.split(","):
            topic = _normalize_topic(raw_topic)
            if not topic or topic in seen:
                continue
            seen.add(topic)
            topics.append(topic)
    return topics


def _create_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="GUI RC publisher for FSMPX4")
    parser.add_argument(
        "--topic",
        action="append",
        default=None,
        help=(
            "ManualControlSetpoint topic to publish. "
            "Repeat this option to publish to multiple topics."
        ),
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        default=None,
        help=(
            "Additional topic list (space/comma separated), for example: "
            "--topics /px4_1/rc/manual_control_setpoint "
            "/px4_2/rc/manual_control_setpoint /px4_3/rc/manual_control_setpoint"
        ),
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Publish rate in Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--node-name",
        default="manual_control_gui",
        help="ROS 2 node name (default: %(default)s)",
    )
    return parser


class ManualControlPublisher(Node):
    """ROS 2 node that owns and publishes the ManualControlSetpoint message."""

    GEAR_HOVER_VALUE = 0.6  # mapped through (aux2 + 1) / 2 to satisfy hover threshold

    def __init__(self, node_name: str, topics: Sequence[str], rate_hz: float) -> None:
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._topics = list(topics)
        self._publishers = [
            self.create_publisher(ManualControlSetpoint, topic, 10) for topic in self._topics
        ]

        self._msg = ManualControlSetpoint()
        self._msg.valid = True
        self._msg.data_source = ManualControlSetpoint.SOURCE_RC
        self._msg.roll = 0.0
        self._msg.pitch = 0.0
        self._msg.yaw = 0.0
        self._msg.throttle = -1.0  # 默认油门最低，避免上电抬升
        self._msg.aux3 = 0.0
        self._msg.aux4 = 0.0
        self._msg.aux5 = 0.0
        self._msg.aux6 = 0.0
        self._msg.buttons = 0
        self._msg.sticks_moving = False

        self._set_mode_internal("manual")

        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.02
        self.create_timer(period, self._on_timer)

    # ------------------------------------------------------------------ publishing
    def _on_timer(self) -> None:
        if self._stop_event.is_set():
            return
        now = self.get_clock().now().nanoseconds
        with self._lock:
            self._msg.timestamp = now
            self._msg.timestamp_sample = now
            self._msg.sticks_moving = any(
                abs(v) > 1e-3
                for v in (self._msg.roll, self._msg.pitch, self._msg.yaw, self._msg.throttle)
            )
            for publisher in self._publishers:
                publisher.publish(self._msg)

    # -------------------------------------------------------------------- external
    def set_stick(self, axis: str, value: float) -> None:
        value = max(-1.0, min(1.0, value))
        with self._lock:
            setattr(self._msg, axis, value)

    def center_sticks(self) -> None:
        for axis in ("roll", "pitch", "yaw", "throttle"):
            self.set_stick(axis, 0.0)

    def set_mode(self, mode: str) -> None:
        self._set_mode_internal(mode.lower())

    def request_shutdown(self) -> None:
        self._stop_event.set()

    def state_snapshot(self) -> Dict[str, float]:
        with self._lock:
            return {
                "roll": float(self._msg.roll),
                "pitch": float(self._msg.pitch),
                "yaw": float(self._msg.yaw),
                "throttle": float(self._msg.throttle),
                "aux1": float(self._msg.aux1),
                "aux2": float(self._msg.aux2),
            }

    @property
    def topics(self) -> List[str]:
        return list(self._topics)

    # -------------------------------------------------------------------- internals
    def _set_mode_internal(self, mode: str) -> None:
        aux1_aux2_map = {
            "manual": (-1.0, -1.0),
            "offboard": (1.0, -1.0),
            "hover": (1.0, self._gear_to_aux(self.GEAR_HOVER_VALUE)),
            "cmd": (1.0, 1.0),
            "command": (1.0, 1.0),
        }
        values = aux1_aux2_map.get(mode)
        if values is None:
            self.get_logger().warn(
                f"Unknown mode '{mode}'. Using MANUAL (-1 aux values)."
            )
            values = aux1_aux2_map["manual"]
        with self._lock:
            self._msg.aux1, self._msg.aux2 = values

    @staticmethod
    def _gear_to_aux(gear_value: float) -> float:
        gear_value = max(0.0, min(1.0, gear_value))
        return 2.0 * gear_value - 1.0


class ManualControlGUI(tk.Tk):
    """Tkinter front-end to manipulate ManualControlPublisher."""

    def __init__(self, node: ManualControlPublisher, topics: Sequence[str]) -> None:
        super().__init__()
        self.title("FSMPX4 Manual Control")
        self.resizable(False, False)
        self.node = node
        self.topics = list(topics)

        self.status_var = tk.StringVar()
        self.mode_var = tk.StringVar(value="Mode: MANUAL")

        self._build_layout()
        self._refresh_status()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ----------------------------------------------------------------- construction
    def _build_layout(self) -> None:
        pad = {"padx": 8, "pady": 4}

        if len(self.topics) == 1:
            header_text = f"Publishing to {self.topics[0]}"
        else:
            header_text = "Publishing to:\n" + "\n".join(self.topics)
        header = ttk.Label(self, text=header_text, font=("Arial", 11, "bold"), justify="left")
        header.grid(row=0, column=0, columnspan=4, **pad)

        axes = [
            ("Roll", "roll"),
            ("Pitch", "pitch"),
            ("Yaw", "yaw"),
            ("Throttle", "throttle"),
        ]
        self.scales = {}
        for idx, (label_text, axis) in enumerate(axes, start=1):
            ttk.Label(self, text=label_text).grid(row=idx, column=0, sticky="e", **pad)
            scale = tk.Scale(
                self,
                from_=1.0,
                to=-1.0,
                resolution=0.01,
                orient=tk.VERTICAL,
                length=200,
                command=lambda value, ax=axis: self._on_scale(ax, value),
            )
            default_val = -1.0 if axis == "throttle" else 0.0
            scale.set(default_val)
            scale.grid(row=idx, column=1, **pad)
            self.scales[axis] = scale

        mode_frame = ttk.LabelFrame(self, text="Modes")
        mode_frame.grid(row=1, column=2, rowspan=len(axes), sticky="nsew", **pad)
        buttons = [
            ("Manual", "manual"),
            ("Offboard", "offboard"),
            ("Hover", "hover"),
            ("Command", "cmd"),
        ]
        for i, (text, mode) in enumerate(buttons):
            ttk.Button(
                mode_frame,
                text=text,
                command=lambda m=mode, label=text: self._on_mode(m, label),
                width=12,
            ).grid(row=i, column=0, sticky="ew", padx=6, pady=4)

        control_row = len(axes) + 1
        ttk.Button(self, text="Center Sticks", command=self._on_center).grid(
            row=control_row, column=0, columnspan=2, sticky="ew", **pad
        )
        ttk.Button(self, text="Quit", command=self._on_close).grid(
            row=control_row, column=2, sticky="ew", **pad
        )

        ttk.Label(self, textvariable=self.mode_var).grid(
            row=control_row + 1, column=0, columnspan=3, sticky="w", **pad
        )
        ttk.Label(self, textvariable=self.status_var).grid(
            row=control_row + 2, column=0, columnspan=3, sticky="w", **pad
        )

    # --------------------------------------------------------------------- callbacks
    def _on_scale(self, axis: str, value: str) -> None:
        try:
            numeric = float(value)
        except ValueError:
            return
        self.node.set_stick(axis, numeric)

    def _on_mode(self, mode: str, label: str) -> None:
        self.node.set_mode(mode)
        self.mode_var.set(f"Mode: {label.upper()}")

    def _on_center(self) -> None:
        self.node.center_sticks()
        for scale in self.scales.values():
            scale.set(0.0)

    def _on_close(self) -> None:
        self.node.request_shutdown()
        self.destroy()

    def _refresh_status(self) -> None:
        state = self.node.state_snapshot()
        self.status_var.set(
            "roll={roll:+.2f} pitch={pitch:+.2f} yaw={yaw:+.2f} thr={throttle:+.2f} "
            "aux1={aux1:+.2f} aux2={aux2:+.2f}".format(**state)
        )
        self.after(200, self._refresh_status)


def main() -> None:
    parser = _create_arg_parser()
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    topic_inputs: List[str] = []
    if args.topic:
        topic_inputs.extend(args.topic)
    if args.topics:
        topic_inputs.extend(args.topics)

    # Backward compatibility: no explicit topic args => publish to single + 3 UAV defaults.
    if not topic_inputs:
        topics = list(DEFAULT_TOPICS)
    else:
        topics = _parse_topics(topic_inputs)
        if not topics:
            parser.error("No valid topics were provided.")

    rclpy.init()
    node = ManualControlPublisher(args.node_name, topics, args.rate)
    node.get_logger().info(f"Publishing ManualControlSetpoint to: {', '.join(topics)}")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    gui = ManualControlGUI(node, topics)
    try:
        gui.mainloop()
    finally:
        node.request_shutdown()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
