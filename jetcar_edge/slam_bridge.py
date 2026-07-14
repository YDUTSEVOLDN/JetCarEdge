from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


@dataclass
class MapSnapshot:
    png_bytes: bytes
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    updated_at: float


@dataclass
class PoseSnapshot:
    x: float
    y: float
    yaw: float
    frame_id: str
    updated_at: float


@dataclass
class ScanSnapshot:
    frame_id: str
    angle_min: float
    angle_increment: float
    points: list[list[float]]
    updated_at: float


def _yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = float(math.cos(yaw * 0.5))
    q.x = 0.0
    q.y = 0.0
    q.z = float(math.sin(yaw * 0.5))
    return q


def _grid_to_png_bytes(msg: OccupancyGrid) -> bytes:
    from PIL import Image

    info = msg.info
    w = int(info.width)
    h = int(info.height)
    data = np.array(msg.data, dtype=np.int16).reshape((h, w))
    img = np.empty((h, w), dtype=np.uint8)

    unknown = data < 0
    img[unknown] = 205
    known = ~unknown
    if np.any(known):
        val = np.clip(data[known], 0, 100).astype(np.float32)
        img[known] = np.clip(255.0 - (val * 255.0 / 100.0), 0.0, 255.0).astype(np.uint8)

    pil = Image.fromarray(img, mode="L")
    out = bytearray()
    with Image.new("L", (w, h)) as canvas:
        canvas.paste(pil)
        buf = __import__("io").BytesIO()
        canvas.save(buf, format="PNG")
        out.extend(buf.getvalue())
    return bytes(out)


class SlamBridge(Node):
    def __init__(
        self,
        *,
        map_topic: str = "/map",
        odom_topic: str = "/odom",
        scan_topic: str = "/scan",
        goal_topic: str = "/goal_pose",
        base_frame_candidates: list[str] | None = None,
        scan_max_points: int = 720,
        scan_offset_x: float | None = None,
        scan_offset_y: float | None = None,
        scan_offset_yaw: float | None = None,
    ) -> None:
        super().__init__("jetcar_slam_bridge")

        self._lock = threading.Lock()
        self._map: Optional[MapSnapshot] = None
        self._odom_pose: Optional[PoseSnapshot] = None
        self._scan: Optional[ScanSnapshot] = None

        self._goal_topic = goal_topic
        self._goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._base_frames = base_frame_candidates or ["base_link", "base_footprint"]
        self._scan_max_points = max(1, int(scan_max_points))
        self._scan_offset_x = (
            float(scan_offset_x)
            if scan_offset_x is not None
            else float(os.getenv("SCAN_OFFSET_X", "0.0"))
        )
        self._scan_offset_y = (
            float(scan_offset_y)
            if scan_offset_y is not None
            else float(os.getenv("SCAN_OFFSET_Y", "0.0"))
        )
        self._scan_offset_yaw = (
            float(scan_offset_yaw)
            if scan_offset_yaw is not None
            else float(os.getenv("SCAN_OFFSET_YAW", "0.0"))
        )

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 1)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        png = _grid_to_png_bytes(msg)
        origin = info.origin
        origin_yaw = _yaw_from_quat(origin.orientation)
        snapshot = MapSnapshot(
            png_bytes=png,
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=float(origin_yaw),
            updated_at=time.time(),
        )
        with self._lock:
            self._map = snapshot

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = _yaw_from_quat(pose.orientation)
        snapshot = PoseSnapshot(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=float(yaw),
            frame_id=str(msg.header.frame_id or "odom"),
            updated_at=time.time(),
        )
        with self._lock:
            self._odom_pose = snapshot

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        count = len(ranges)
        if count <= 0:
            return
        stride = max(1, count // self._scan_max_points)

        pts: list[list[float]] = []
        angle = float(msg.angle_min)
        cos_offset = math.cos(self._scan_offset_yaw)
        sin_offset = math.sin(self._scan_offset_yaw)
        for i in range(0, count, stride):
            r = ranges[i]
            angle_i = angle + i * float(msg.angle_increment)
            if not math.isfinite(r) or r <= 0.02:
                continue
            x = float(r * math.cos(angle_i))
            y = float(r * math.sin(angle_i))
            x_fixed = x * cos_offset - y * sin_offset + self._scan_offset_x
            y_fixed = x * sin_offset + y * cos_offset + self._scan_offset_y
            pts.append([x_fixed, y_fixed])

        snapshot = ScanSnapshot(
            frame_id=str(msg.header.frame_id or "base_link"),
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            points=pts,
            updated_at=time.time(),
        )
        with self._lock:
            self._scan = snapshot

    def latest_map(self) -> Optional[MapSnapshot]:
        with self._lock:
            return self._map

    def latest_scan(self) -> Optional[ScanSnapshot]:
        with self._lock:
            return self._scan

    def car_pose(self) -> Optional[PoseSnapshot]:
        for base_frame in self._base_frames:
            try:
                tf = self._tf_buffer.lookup_transform("map", base_frame, Time(), timeout_sec=0.05)
            except Exception:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = _yaw_from_quat(Quaternion(x=q.x, y=q.y, z=q.z, w=q.w))
            return PoseSnapshot(
                x=float(t.x),
                y=float(t.y),
                yaw=float(yaw),
                frame_id="map",
                updated_at=time.time(),
            )

        with self._lock:
            return self._odom_pose

    def publish_goal(self, *, x: float, y: float, yaw: float, frame_id: str) -> None:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation = _quat_from_yaw(float(yaw))
        self._goal_pub.publish(msg)

    def publish_initial_pose(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str = "map",
    ) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _quat_from_yaw(float(yaw))
        msg.pose.covariance = [
            0.2,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.2,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
        ]
        self._initialpose_pub.publish(msg)
