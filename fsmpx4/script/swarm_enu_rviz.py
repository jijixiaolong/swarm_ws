#!/usr/bin/env python3
"""Publish swarm global positions into a shared ENU frame for RViz."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


EARTH_RADIUS_M = 6378137.0
DEG_TO_RAD = pi / 180.0
DEFAULT_UAV_NAMESPACES = ["/px4_1", "/px4_2", "/px4_3"]
DEFAULT_PAYLOAD_TOPIC = "/px4_4/fmu/out/vehicle_global_position"


@dataclass
class EntityState:
    name: str
    label: str
    topic: str
    marker_type: int
    marker_scale_m: float
    color: Tuple[float, float, float]
    pose_pub: rclpy.publisher.Publisher
    position_enu: Optional[Tuple[float, float, float]] = None
    last_update_ns: int = 0


class SwarmEnuRvizNode(Node):
    def __init__(self) -> None:
        super().__init__("swarm_enu_rviz")

        self._origin_lat_deg = float(
            self.declare_parameter("position.gps_origin_latitude_deg", 47.397742).value
        )
        self._origin_lon_deg = float(
            self.declare_parameter("position.gps_origin_longitude_deg", 8.545594).value
        )
        self._origin_alt_m = float(
            self.declare_parameter("position.gps_origin_altitude_m", 488.0).value
        )
        self._origin_lat_rad = self._origin_lat_deg * DEG_TO_RAD

        raw_uav_namespaces = self.declare_parameter(
            "swarm.uav_namespaces", DEFAULT_UAV_NAMESPACES
        ).value
        self._uav_namespaces = [self._normalize_namespace(ns) for ns in raw_uav_namespaces if ns]
        self._payload_topic = self._normalize_topic(
            str(
                self.declare_parameter(
                    "swarm.payload_global_position_topic", DEFAULT_PAYLOAD_TOPIC
                ).value
            )
        )

        self._fixed_frame = str(self.declare_parameter("rviz.fixed_frame", "map").value)
        self._fixed_frame_parent = str(
            self.declare_parameter("rviz.fixed_frame_parent", "world").value
        )
        self._publish_fixed_frame_tf = bool(
            self.declare_parameter("rviz.publish_fixed_frame_tf", True).value
        )
        self._marker_topic = self._normalize_topic(
            str(self.declare_parameter("rviz.marker_topic", "/swarm/rviz/markers").value)
        )
        self._compat_marker_topic = self._normalize_topic(
            str(
                self.declare_parameter(
                    "rviz.compat_marker_topic", "/visualization_marker_array"
                ).value
            )
        )
        self._pose_topic_prefix = self._normalize_topic(
            str(self.declare_parameter("rviz.pose_topic_prefix", "/swarm/rviz").value)
        ).rstrip("/")
        self._publish_rate_hz = max(
            1.0, float(self.declare_parameter("rviz.publish_rate_hz", 20.0).value)
        )
        self._freshness_timeout_s = max(
            0.1, float(self.declare_parameter("rviz.freshness_timeout_s", 0.6).value)
        )
        self._uav_marker_scale_m = max(
            0.05, float(self.declare_parameter("rviz.uav_marker_scale_m", 0.35).value)
        )
        self._payload_marker_scale_m = max(
            0.05, float(self.declare_parameter("rviz.payload_marker_scale_m", 0.28).value)
        )
        self._text_height_m = max(
            0.05, float(self.declare_parameter("rviz.text_height_m", 0.22).value)
        )
        self._rope_width_m = max(
            0.01, float(self.declare_parameter("rviz.rope_width_m", 0.03).value)
        )
        self._marker_lifetime = self._duration_from_seconds(
            max(0.2, 2.0 / self._publish_rate_hz)
        )

        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._marker_pubs = [
            self.create_publisher(MarkerArray, self._marker_topic, marker_qos),
        ]
        if self._compat_marker_topic and self._compat_marker_topic != self._marker_topic:
            self._marker_pubs.append(
                self.create_publisher(MarkerArray, self._compat_marker_topic, marker_qos)
            )
        self._tf_static_broadcaster: Optional[StaticTransformBroadcaster] = None
        self._entities: Dict[str, EntityState] = {}
        self._entity_order: List[str] = []
        self._subscriptions = []

        uav_colors = (
            (0.95, 0.25, 0.25),
            (0.15, 0.75, 0.25),
            (0.20, 0.45, 0.95),
        )
        for index, namespace in enumerate(self._uav_namespaces):
            name = namespace.lstrip("/").replace("/", "_") or f"uav_{index + 1}"
            label = f"UAV {index + 1}"
            topic = f"{namespace}/fmu/out/vehicle_global_position"
            pose_topic = f"{self._pose_topic_prefix}/{name}/pose"
            entity = EntityState(
                name=name,
                label=label,
                topic=topic,
                marker_type=Marker.SPHERE,
                marker_scale_m=self._uav_marker_scale_m,
                color=uav_colors[index % len(uav_colors)],
                pose_pub=self.create_publisher(PoseStamped, pose_topic, marker_qos),
            )
            self._entities[name] = entity
            self._entity_order.append(name)
            self._subscriptions.append(
                self.create_subscription(
                    VehicleGlobalPosition,
                    topic,
                    lambda msg, key=name: self._on_global_position(key, msg),
                    sensor_qos,
                )
            )

        if self._payload_topic:
            payload_pose_topic = f"{self._pose_topic_prefix}/payload/pose"
            payload = EntityState(
                name="payload",
                label="Payload",
                topic=self._payload_topic,
                marker_type=Marker.CUBE,
                marker_scale_m=self._payload_marker_scale_m,
                color=(1.0, 0.65, 0.10),
                pose_pub=self.create_publisher(PoseStamped, payload_pose_topic, marker_qos),
            )
            self._entities[payload.name] = payload
            self._entity_order.append(payload.name)
            self._subscriptions.append(
                self.create_subscription(
                    VehicleGlobalPosition,
                    self._payload_topic,
                    lambda msg, key=payload.name: self._on_global_position(key, msg),
                    sensor_qos,
                )
            )

        self._publish_fixed_frame_transform()
        self.create_timer(1.0 / self._publish_rate_hz, self._publish_visualization)

        watched_topics = [self._entities[key].topic for key in self._entity_order]
        self.get_logger().info(
            "ENU RViz bridge started | frame=%s marker_topics=%s pose_prefix=%s topics=%s"
            % (
                self._fixed_frame,
                ",".join(pub.topic_name for pub in self._marker_pubs),
                self._pose_topic_prefix,
                ", ".join(watched_topics),
            )
        )

    def _on_global_position(self, entity_name: str, msg: VehicleGlobalPosition) -> None:
        if not msg.lat_lon_valid or not msg.alt_valid:
            return

        east, north, up = self._lla_to_enu(msg.lat, msg.lon, msg.alt)
        entity = self._entities[entity_name]
        entity.position_enu = (east, north, up)
        entity.last_update_ns = self.get_clock().now().nanoseconds

    def _publish_visualization(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        marker_array = MarkerArray()

        for marker in self._build_entity_markers(stamp, now):
            marker_array.markers.append(marker)

        rope_marker = self._build_rope_marker(stamp, now)
        if rope_marker is not None:
            marker_array.markers.append(rope_marker)

        for publisher in self._marker_pubs:
            publisher.publish(marker_array)

    def _build_entity_markers(
        self, stamp, now: rclpy.time.Time
    ) -> List[Marker]:
        markers: List[Marker] = []
        for index, key in enumerate(self._entity_order):
            entity = self._entities[key]
            if not self._is_fresh(entity, now):
                continue

            self._publish_pose(entity, stamp)
            markers.append(self._make_body_marker(index, entity, stamp))
            markers.append(self._make_text_marker(index, entity, stamp))
        return markers

    def _build_rope_marker(self, stamp, now: rclpy.time.Time) -> Optional[Marker]:
        payload = self._entities.get("payload")
        if payload is None or not self._is_fresh(payload, now) or payload.position_enu is None:
            return None

        rope = Marker()
        rope.header.frame_id = self._fixed_frame
        rope.header.stamp = stamp
        rope.ns = "swarm"
        rope.id = 1000
        rope.type = Marker.LINE_LIST
        rope.action = Marker.ADD
        rope.pose.orientation.w = 1.0
        rope.scale.x = self._rope_width_m
        rope.color.r = 0.85
        rope.color.g = 0.85
        rope.color.b = 0.85
        rope.color.a = 0.9
        rope.lifetime = self._marker_lifetime

        payload_point = self._to_point(payload.position_enu)
        for key in self._entity_order:
            if key == "payload":
                continue
            entity = self._entities[key]
            if not self._is_fresh(entity, now) or entity.position_enu is None:
                continue
            rope.points.append(payload_point)
            rope.points.append(self._to_point(entity.position_enu))

        if not rope.points:
            return None
        return rope

    def _publish_pose(self, entity: EntityState, stamp) -> None:
        if entity.position_enu is None:
            return

        pose = PoseStamped()
        pose.header.frame_id = self._fixed_frame
        pose.header.stamp = stamp
        pose.pose.position = self._to_point(entity.position_enu)
        pose.pose.orientation.w = 1.0
        entity.pose_pub.publish(pose)

    def _make_body_marker(self, index: int, entity: EntityState, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._fixed_frame
        marker.header.stamp = stamp
        marker.ns = "swarm_body"
        marker.id = index
        marker.type = entity.marker_type
        marker.action = Marker.ADD
        marker.pose.position = self._to_point(entity.position_enu)
        marker.pose.orientation.w = 1.0
        marker.scale.x = entity.marker_scale_m
        marker.scale.y = entity.marker_scale_m
        marker.scale.z = entity.marker_scale_m
        marker.color.r = entity.color[0]
        marker.color.g = entity.color[1]
        marker.color.b = entity.color[2]
        marker.color.a = 0.95
        marker.lifetime = self._marker_lifetime
        return marker

    def _make_text_marker(self, index: int, entity: EntityState, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._fixed_frame
        marker.header.stamp = stamp
        marker.ns = "swarm_text"
        marker.id = 100 + index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = self._to_point(entity.position_enu)
        marker.pose.position.z += entity.marker_scale_m * 0.75 + 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._text_height_m
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = entity.label
        marker.lifetime = self._marker_lifetime
        return marker

    def _is_fresh(self, entity: EntityState, now: rclpy.time.Time) -> bool:
        if entity.position_enu is None or entity.last_update_ns <= 0:
            return False
        age_s = (now.nanoseconds - entity.last_update_ns) / 1e9
        return age_s <= self._freshness_timeout_s

    def _lla_to_enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        north = (lat_deg - self._origin_lat_deg) * DEG_TO_RAD * EARTH_RADIUS_M
        east = (lon_deg - self._origin_lon_deg) * DEG_TO_RAD * EARTH_RADIUS_M * cos(
            self._origin_lat_rad
        )
        down = self._origin_alt_m - alt_m
        return east, north, -down

    def _publish_fixed_frame_transform(self) -> None:
        if not self._publish_fixed_frame_tf:
            return
        if not self._fixed_frame or not self._fixed_frame_parent:
            return
        if self._fixed_frame == self._fixed_frame_parent:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._fixed_frame_parent
        transform.child_frame_id = self._fixed_frame
        transform.transform.rotation.w = 1.0

        self._tf_static_broadcaster = StaticTransformBroadcaster(self)
        self._tf_static_broadcaster.sendTransform(transform)

    @staticmethod
    def _to_point(position_enu: Tuple[float, float, float]) -> Point:
        point = Point()
        point.x = float(position_enu[0])
        point.y = float(position_enu[1])
        point.z = float(position_enu[2])
        return point

    @staticmethod
    def _normalize_topic(topic: str) -> str:
        topic = topic.strip()
        if not topic:
            return ""
        return topic if topic.startswith("/") else f"/{topic}"

    @staticmethod
    def _normalize_namespace(namespace: str) -> str:
        namespace = namespace.strip()
        if not namespace:
            return ""
        namespace = namespace if namespace.startswith("/") else f"/{namespace}"
        return namespace.rstrip("/")

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        total_nanoseconds = max(0, int(seconds * 1e9))
        msg = Duration()
        msg.sec = total_nanoseconds // 1_000_000_000
        msg.nanosec = total_nanoseconds % 1_000_000_000
        return msg


def main() -> int:
    rclpy.init()
    node = SwarmEnuRvizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
