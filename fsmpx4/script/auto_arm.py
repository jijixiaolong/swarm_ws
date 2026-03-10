#!/usr/bin/env python3
"""
Auto-arm helper for PX4 swarm.

Repeatedly sends VEHICLE_CMD_COMPONENT_ARM_DISARM to each UAV until armed
or timeout. Intended for SITL regression tests.
"""

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException

from px4_msgs.msg import VehicleCommand, VehicleStatus


@dataclass
class ArmState:
    system_id: int = 0
    armed: bool = False
    last_arming_state: int = 0
    received_status: bool = False
    sent_commands: int = 0


class AutoArmNode(Node):
    def __init__(
        self,
        uavs: List[str],
        cmd_period_s: float,
        timeout_s: float,
        source_system: int,
        source_component: int,
        force_arm: bool,
    ) -> None:
        super().__init__("auto_arm_node")
        self._uavs = uavs
        self._cmd_period_s = max(0.1, cmd_period_s)
        self._timeout_s = max(1.0, timeout_s)
        self._source_system = max(0, min(255, source_system))
        self._source_component = max(0, min(255, source_component))
        self._force_arm = force_arm

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self._state: Dict[str, ArmState] = {}
        self._last_cmd_ts: Dict[str, float] = {}
        self._start_ts = time.monotonic()

        for uav in self._uavs:
            status_topic = f"/{uav}/fmu/out/vehicle_status_v1"
            cmd_topic = f"/{uav}/fmu/in/vehicle_command"
            self._state[uav] = ArmState()
            self._last_cmd_ts[uav] = 0.0
            self._pubs[uav] = self.create_publisher(VehicleCommand, cmd_topic, cmd_qos)
            self.create_subscription(
                VehicleStatus,
                status_topic,
                lambda msg, name=uav: self._on_status(name, msg),
                sensor_qos,
            )

        self.get_logger().info(
            f"Auto-arm targets: {', '.join('/' + u for u in self._uavs)} | "
            f"timeout={self._timeout_s:.1f}s cmd_period={self._cmd_period_s:.2f}s force_arm={self._force_arm}"
        )

    def _on_status(self, uav: str, msg: VehicleStatus) -> None:
        st = self._state[uav]
        st.received_status = True
        st.last_arming_state = int(msg.arming_state)
        if msg.system_id > 0:
            st.system_id = int(msg.system_id)
        st.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _all_armed(self) -> bool:
        return all(self._state[u].armed for u in self._uavs)

    def _timed_out(self) -> bool:
        return (time.monotonic() - self._start_ts) >= self._timeout_s

    def _publish_arm_cmd(self, uav: str) -> None:
        st = self._state[uav]
        if st.system_id <= 0:
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.param2 = 21196.0 if self._force_arm else 0.0
        msg.target_system = st.system_id
        msg.target_component = 1
        msg.source_system = self._source_system
        msg.source_component = self._source_component
        msg.from_external = True

        self._pubs[uav].publish(msg)
        st.sent_commands += 1

    def step(self) -> None:
        now = time.monotonic()
        for uav in self._uavs:
            st = self._state[uav]
            if st.armed:
                continue
            if now - self._last_cmd_ts[uav] < self._cmd_period_s:
                continue
            self._last_cmd_ts[uav] = now
            self._publish_arm_cmd(uav)

    def summary(self) -> str:
        lines = ["Auto-arm summary:"]
        for uav in self._uavs:
            st = self._state[uav]
            lines.append(
                f"  {uav}: armed={st.armed} sysid={st.system_id} "
                f"status_rx={st.received_status} arming_state={st.last_arming_state} sent_cmd={st.sent_commands}"
            )
        return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Auto-arm PX4 UAV(s)")
    parser.add_argument(
        "--uavs",
        nargs="+",
        default=["px4_1", "px4_2", "px4_3"],
        help="UAV namespaces without leading slash",
    )
    parser.add_argument("--timeout", type=float, default=15.0, help="Timeout in seconds")
    parser.add_argument("--period", type=float, default=0.5, help="Arm command period in seconds")
    parser.add_argument("--source-system", type=int, default=1, help="VehicleCommand source_system")
    parser.add_argument("--source-component", type=int, default=1, help="VehicleCommand source_component")
    parser.add_argument(
        "--force-arm",
        action="store_true",
        help="Send force-arm magic number (param2=21196)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    uavs = [u.strip().lstrip("/") for u in args.uavs if u.strip()]
    if not uavs:
        print("No UAV namespaces provided.", file=sys.stderr)
        return 2

    rclpy.init()
    node = AutoArmNode(
        uavs=uavs,
        cmd_period_s=args.period,
        timeout_s=args.timeout,
        source_system=args.source_system,
        source_component=args.source_component,
        force_arm=args.force_arm,
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.step()
            if node._all_armed():
                print(node.summary())
                return 0
            if node._timed_out():
                print(node.summary(), file=sys.stderr)
                return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
