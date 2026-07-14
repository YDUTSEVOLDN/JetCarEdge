from __future__ import annotations

import base64
import os
import threading
import time
from typing import Any, Optional

import rclpy
import uvicorn
from fastapi import FastAPI, Query
from pydantic import BaseModel, Field

from jetcar_edge.slam_bridge import PoseSnapshot, ScanSnapshot, SlamBridge


class GoalRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = Field(default="map")


class InitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = Field(default="map")


class SlamRuntime:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._bridge: Optional[SlamBridge] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return

            rclpy.init(args=None)
            goal_topic = os.getenv("ROS_GOAL_TOPIC", "/goal_pose").strip() or "/goal_pose"
            map_topic = os.getenv("ROS_MAP_TOPIC", "/map").strip() or "/map"
            odom_topic = os.getenv("ROS_ODOM_TOPIC", "/odom").strip() or "/odom"
            scan_topic = os.getenv("ROS_SCAN_TOPIC", "/scan").strip() or "/scan"
            base_frames = [s.strip() for s in os.getenv("ROS_BASE_FRAMES", "base_link,base_footprint").split(",") if s.strip()]

            self._bridge = SlamBridge(
                map_topic=map_topic,
                odom_topic=odom_topic,
                scan_topic=scan_topic,
                goal_topic=goal_topic,
                base_frame_candidates=base_frames,
                scan_max_points=int(os.getenv("SLAM_SCAN_MAX_POINTS", "720")),
            )

            def runner() -> None:
                try:
                    rclpy.spin(self._bridge)
                finally:
                    try:
                        self._bridge.destroy_node()
                    except Exception:
                        pass
                    rclpy.shutdown()

            self._thread = threading.Thread(target=runner, name="jetcar-slam-bridge", daemon=True)
            self._thread.start()

    def bridge(self) -> SlamBridge:
        with self._lock:
            if self._bridge is None:
                raise RuntimeError("slam runtime not started")
            return self._bridge


runtime = SlamRuntime()
app = FastAPI(title="JetCar SLAM Web API")


@app.on_event("startup")
async def _startup() -> None:
    runtime.start()


def _pose_dict(pose: Optional[PoseSnapshot]) -> dict[str, Any]:
    if pose is None:
        return {"available": False}
    return {
        "available": True,
        "x": pose.x,
        "y": pose.y,
        "yaw": pose.yaw,
        "frame_id": pose.frame_id,
        "updated_at": pose.updated_at,
    }


def _scan_dict(scan: Optional[ScanSnapshot]) -> dict[str, Any]:
    if scan is None:
        return {"available": False}
    return {
        "available": True,
        "frame_id": scan.frame_id,
        "angle_min": scan.angle_min,
        "angle_increment": scan.angle_increment,
        "points": scan.points,
        "updated_at": scan.updated_at,
    }


@app.get("/api/slam/map")
async def get_slam_map(include_map: bool = Query(default=True)) -> dict[str, Any]:
    bridge = runtime.bridge()
    pose = bridge.car_pose()
    scan = bridge.latest_scan()
    snapshot = bridge.latest_map()

    if snapshot is None:
        return {
            "available": False,
            "map_png_base64": "",
            "car_x": pose.x if pose else 0.0,
            "car_y": pose.y if pose else 0.0,
            "car_yaw": pose.yaw if pose else 0.0,
            "pose_frame_id": pose.frame_id if pose else "",
            "pose_updated_at": pose.updated_at if pose else 0.0,
            "map_origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "resolution": 0.0,
            "width": 0,
            "height": 0,
            "map_updated_at": 0.0,
            "bounds": {
                "min_x": 0.0,
                "min_y": 0.0,
                "max_x": 0.0,
                "max_y": 0.0,
            },
            "pose": _pose_dict(pose),
            "scan": _scan_dict(scan),
        }

    map_b64 = ""
    if include_map:
        map_b64 = base64.b64encode(snapshot.png_bytes).decode("ascii")

    return {
        "available": True,
        "map_png_base64": map_b64,
        "car_x": pose.x if pose else 0.0,
        "car_y": pose.y if pose else 0.0,
        "car_yaw": pose.yaw if pose else 0.0,
        "pose_frame_id": pose.frame_id if pose else "",
        "pose_updated_at": pose.updated_at if pose else 0.0,
        "map_origin": {"x": snapshot.origin_x, "y": snapshot.origin_y, "yaw": snapshot.origin_yaw},
        "resolution": snapshot.resolution,
        "width": snapshot.width,
        "height": snapshot.height,
        "map_updated_at": snapshot.updated_at,
        "bounds": {
            "min_x": snapshot.origin_x,
            "min_y": snapshot.origin_y,
            "max_x": snapshot.origin_x + snapshot.width * snapshot.resolution,
            "max_y": snapshot.origin_y + snapshot.height * snapshot.resolution,
        },
        "pose": _pose_dict(pose),
        "scan": _scan_dict(scan),
    }


@app.post("/api/slam/goal")
async def post_slam_goal(payload: GoalRequest) -> dict[str, Any]:
    bridge = runtime.bridge()
    bridge.publish_goal(x=payload.x, y=payload.y, yaw=payload.yaw, frame_id=payload.frame_id)
    return {
        "ok": True,
        "topic": os.getenv("ROS_GOAL_TOPIC", "/goal_pose").strip() or "/goal_pose",
        "goal": payload.model_dump(mode="json"),
        "server_time": time.time(),
    }


@app.post("/api/set_initial_pose")
async def post_initial_pose(payload: InitialPoseRequest) -> dict[str, Any]:
    bridge = runtime.bridge()
    bridge.publish_initial_pose(
        x=payload.x,
        y=payload.y,
        yaw=payload.yaw,
        frame_id=payload.frame_id,
    )
    return {
        "ok": True,
        "topic": "/initialpose",
        "initial_pose": payload.model_dump(mode="json"),
        "server_time": time.time(),
    }


def main() -> None:
    host = os.getenv("SLAM_API_HOST", "0.0.0.0").strip() or "0.0.0.0"
    port = int(os.getenv("SLAM_API_PORT", "8000"))
    uvicorn.run(app, host=host, port=port, log_level="info")
