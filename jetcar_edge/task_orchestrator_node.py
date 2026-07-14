from __future__ import annotations

import json
import math
import socketserver
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

try:
    from nav2_msgs.action import NavigateToPose
except Exception:  # pragma: no cover - Jetson ROS env decides this.
    NavigateToPose = None


INSPECTION_ALGORITHMS = ["yolov5-manhole-detect", "yolov8-road-damage"]
SIMILARITY_ALGORITHMS = ["yolov5-similarity"]


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float = 0.0
    hold_seconds: float = 1.0
    label: str = ""


@dataclass
class TaskState:
    task_id: str = ""
    mode: str = "idle"
    status: str = "idle"
    message: str = ""
    active: bool = False
    started_at: float = 0.0
    updated_at: float = 0.0
    current_index: int = -1
    total: int = 0
    pose: dict[str, float] = field(default_factory=dict)
    last_nav_status: str = ""
    summary: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": "edge_task_state",
            "task_id": self.task_id,
            "mode": self.mode,
            "status": self.status,
            "message": self.message,
            "active": self.active,
            "started_at": self.started_at,
            "updated_at": self.updated_at,
            "current_index": self.current_index,
            "total": self.total,
            "pose": self.pose,
            "last_nav_status": self.last_nav_status,
            "summary": self.summary,
        }


class TaskOrchestratorNode(Node):
    """Small demo-first task layer over Nav2 and the existing Edge AI uploader.

    It intentionally depends on standard Nav2 action/topic names. If the vendor
    launch exposes different names, change only the launch/config parameters.
    """

    def __init__(self) -> None:
        super().__init__("jetcar_edge_task_orchestrator")

        self.declare_parameter("task_control_host", "0.0.0.0")
        self.declare_parameter("task_control_port", 6002)
        self.declare_parameter("algorithm_control_topic", "/jetcar/algorithm_ids")
        self.declare_parameter("ai_result_topic", "/jetcar/ai_result")
        self.declare_parameter("task_status_topic", "/jetcar/task_status")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("default_goal_timeout_seconds", 90.0)
        self.declare_parameter("stop_on_nav_failure", True)
        self.declare_parameter("publish_status_seconds", 1.0)

        self._algorithm_pub = self.create_publisher(
            String,
            str(self.get_parameter("algorithm_control_topic").value),
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("task_status_topic").value),
            10,
        )
        self._cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            self._on_pose,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("ai_result_topic").value),
            self._on_ai_result,
            10,
        )

        self._navigator = (
            ActionClient(self, NavigateToPose, str(self.get_parameter("navigate_action").value))
            if NavigateToPose is not None
            else None
        )
        self._state = TaskState(updated_at=time.time())
        self._state_lock = threading.Lock()
        self._task_cancel = threading.Event()
        self._task_thread: Optional[threading.Thread] = None
        self._current_goal_handle = None
        self._pose: dict[str, float] = {}
        self._visual_servo_takeover = threading.Event()
        self._waypoint_sets = self._load_waypoints()
        self._task_summary: dict[str, Any] = {}
        self._server = None
        self._server_thread = None

        self._start_control_server()
        self.create_timer(float(self.get_parameter("publish_status_seconds").value), self._publish_state)
        self.get_logger().info("JetCar task orchestrator started")

    def destroy_node(self) -> bool:
        self._cancel_active_task("node_destroy")
        self._stop_control_server()
        return super().destroy_node()

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        yaw = _yaw_from_quaternion(pose.orientation)
        self._pose = {
            "x": round(float(pose.position.x), 4),
            "y": round(float(pose.position.y), 4),
            "yaw": round(yaw, 4),
            "stamp": time.time(),
        }
        with self._state_lock:
            self._state.pose = dict(self._pose)
            self._state.updated_at = time.time()

    def _on_ai_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return
        if payload.get("type") != "edge_similarity_search":
            return
        event = str(payload.get("event") or "")
        if event not in {"target_tracking", "target_found"}:
            return
        with self._state_lock:
            is_search_task = self._state.active and self._state.mode == "similarity_search_task"
        if not is_search_task:
            return
        self._visual_servo_takeover.set()
        self._cancel_nav_goal()
        self._update_state(
            status="visual_servo",
            message="similarity target locked; Nav2 goal cancelled for final visual servo",
        )

    def _start_control_server(self) -> None:
        host = str(self.get_parameter("task_control_host").value).strip()
        port = int(self.get_parameter("task_control_port").value)
        if port <= 0:
            self.get_logger().info("task control TCP server disabled")
            return
        node = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self) -> None:
                for raw in self.rfile:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    try:
                        payload = json.loads(line)
                        if not isinstance(payload, dict):
                            raise ValueError("task payload must be a JSON object")
                        result = node._handle_command(payload)
                    except Exception as exc:
                        result = {"ok": False, "error": str(exc)}
                    self.wfile.write((json.dumps(result, ensure_ascii=False) + "\n").encode("utf-8"))
                    self.wfile.flush()

        class ThreadingServer(socketserver.ThreadingTCPServer):
            allow_reuse_address = True

        self._server = ThreadingServer((host, port), Handler)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="jetcar-task-control",
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(f"task control TCP server listening on {host}:{port}")

    def _stop_control_server(self) -> None:
        if self._server is None:
            return
        self._server.shutdown()
        self._server.server_close()
        if self._server_thread is not None:
            self._server_thread.join(timeout=2.0)
        self._server = None
        self._server_thread = None

    def _handle_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = str(payload.get("mode") or payload.get("type") or "").strip().lower()
        if mode in {"status", "task_status"}:
            return {"ok": True, "state": self._snapshot_state()}
        if mode in {"waypoints", "list_waypoints"}:
            return {"ok": True, "waypoints": self._waypoint_sets}
        if mode in {"set_waypoints", "update_waypoints"}:
            name = str(payload.get("name") or payload.get("set") or "").strip()
            if not name:
                raise ValueError("set_waypoints requires name")
            waypoints = payload.get("waypoints")
            if not isinstance(waypoints, list):
                raise ValueError("set_waypoints requires waypoints list")
            self._waypoint_sets[name] = [item for item in waypoints if isinstance(item, dict)]
            return {"ok": True, "name": name, "count": len(self._waypoint_sets[name])}
        if mode in {"stop", "stop_task", "cancel"}:
            self._cancel_active_task("app_stop")
            return {"ok": True, "state": self._snapshot_state()}
        if mode == "navigate_to_point":
            waypoint = Waypoint(
                x=float(payload["x"]),
                y=float(payload["y"]),
                yaw=float(payload.get("yaw", 0.0)),
                hold_seconds=float(payload.get("hold_seconds", 0.0)),
                label=str(payload.get("label") or "app_goal"),
            )
            return self._start_task("navigate_to_point", [waypoint], [])
        if mode in {"inspection_task", "road_inspection_task"}:
            waypoints = self._waypoints_from_payload(payload, fallback_key="inspection")
            return self._start_task("inspection_task", waypoints, INSPECTION_ALGORITHMS)
        if mode in {"similarity_search_task", "search_task"}:
            waypoints = self._waypoints_from_payload(payload, fallback_key="search")
            return self._start_task("similarity_search_task", waypoints, SIMILARITY_ALGORITHMS)
        raise ValueError(f"unsupported task mode: {mode}")

    def _start_task(self, mode: str, waypoints: list[Waypoint], algorithms: list[str]) -> dict[str, Any]:
        if not waypoints:
            raise ValueError(f"{mode} requires at least one waypoint")
        self._cancel_active_task("new_task")
        self._task_cancel.clear()
        self._visual_servo_takeover.clear()
        self._task_summary = {
            "visited": [],
            "failed": [],
            "started_at": time.time(),
        }
        task_id = f"{mode}-{int(time.time() * 1000)}"
        with self._state_lock:
            self._state = TaskState(
                task_id=task_id,
                mode=mode,
                status="starting",
                message="task queued",
                active=True,
                started_at=time.time(),
                updated_at=time.time(),
                current_index=0,
                total=len(waypoints),
                pose=dict(self._pose),
            )
        self._task_thread = threading.Thread(
            target=self._run_waypoint_task,
            args=(task_id, mode, waypoints, algorithms),
            name=f"jetcar-task-{mode}",
            daemon=True,
        )
        self._task_thread.start()
        self._publish_state()
        return {"ok": True, "task_id": task_id, "state": self._snapshot_state()}

    def _cancel_active_task(self, reason: str) -> None:
        self._task_cancel.set()
        self._cancel_nav_goal()
        self._publish_algorithms([])
        self._publish_stop()
        with self._state_lock:
            if self._state.active:
                self._state.status = "cancelled"
                self._state.message = reason
                self._state.active = False
                self._state.updated_at = time.time()
        self._publish_state()

    def _run_waypoint_task(
        self,
        task_id: str,
        mode: str,
        waypoints: list[Waypoint],
        algorithms: list[str],
    ) -> None:
        self._publish_algorithms(algorithms)
        try:
            for index, waypoint in enumerate(waypoints):
                if self._task_cancel.is_set():
                    return
                self._update_state(
                    status="navigating",
                    message=waypoint.label or f"waypoint {index + 1}/{len(waypoints)}",
                    current_index=index,
                )
                nav = self._navigate_to(waypoint)
                if not nav["ok"]:
                    if mode == "similarity_search_task" and nav.get("status") == "visual_servo_takeover":
                        return
                    self._task_summary["failed"].append(
                        {
                            "index": index,
                            "label": waypoint.label,
                            "x": waypoint.x,
                            "y": waypoint.y,
                            "yaw": waypoint.yaw,
                            "error": nav.get("error", ""),
                            "status": nav.get("status", ""),
                            "pose": dict(self._pose),
                            "time": time.time(),
                        }
                    )
                    self._update_state(status="failed", message=nav["error"], last_nav_status=nav.get("status", ""))
                    if bool(self.get_parameter("stop_on_nav_failure").value):
                        self._publish_algorithms([])
                        self._publish_stop()
                        return
                else:
                    self._task_summary["visited"].append(
                        {
                            "index": index,
                            "label": waypoint.label,
                            "x": waypoint.x,
                            "y": waypoint.y,
                            "yaw": waypoint.yaw,
                            "pose": dict(self._pose),
                            "time": time.time(),
                        }
                    )
                if waypoint.hold_seconds > 0 and not self._task_cancel.is_set():
                    self._update_state(status="holding", message=f"hold {waypoint.hold_seconds:.1f}s")
                    self._sleep_interruptible(waypoint.hold_seconds)

            if mode == "similarity_search_task":
                self._task_summary["finished_navigation_at"] = time.time()
                self._update_state(status="visual_search", message="waypoints finished; Edge visual servo continues if target is matched")
            else:
                self._task_summary["finished_at"] = time.time()
                self._publish_algorithms([])
                self._publish_stop()
                self._update_state(status="completed", message="task completed", active=False)
        finally:
            if self._task_cancel.is_set():
                self._publish_algorithms([])
                self._publish_stop()
                self._update_state(status="cancelled", message="task cancelled", active=False)

    def _navigate_to(self, waypoint: Waypoint) -> dict[str, Any]:
        if self._navigator is None:
            return {"ok": False, "error": "nav2_msgs is unavailable in this ROS environment", "status": "navigator_unavailable"}
        if not self._navigator.wait_for_server(timeout_sec=3.0):
            return {"ok": False, "error": "navigate_to_pose action server unavailable", "status": "navigator_unavailable"}

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(waypoint.x)
        goal_msg.pose.pose.position.y = float(waypoint.y)
        goal_msg.pose.pose.orientation = _quaternion_from_yaw(float(waypoint.yaw))

        send_future = self._navigator.send_goal_async(goal_msg)
        if not self._wait_future(send_future, timeout_seconds=5.0):
            return {"ok": False, "error": "navigation goal send timeout", "status": "send_timeout"}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"ok": False, "error": "navigation goal rejected", "status": "goal_rejected"}
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        timeout = float(self.get_parameter("default_goal_timeout_seconds").value)
        started = time.monotonic()
        while not result_future.done():
            if self._task_cancel.is_set():
                self._cancel_nav_goal()
                return {"ok": False, "error": "navigation cancelled", "status": "cancelled"}
            if self._visual_servo_takeover.is_set():
                return {"ok": False, "error": "visual servo takeover", "status": "visual_servo_takeover"}
            if time.monotonic() - started > timeout:
                self._cancel_nav_goal()
                return {"ok": False, "error": "navigation timeout", "status": "timeout"}
            time.sleep(0.1)
        self._current_goal_handle = None
        result = result_future.result()
        if result is None:
            return {"ok": False, "error": "navigation returned no result", "status": "no_result"}
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return {"ok": True, "status": "succeeded"}
        return {"ok": False, "error": f"navigation failed status={result.status}", "status": str(result.status)}

    def _cancel_nav_goal(self) -> None:
        goal_handle = self._current_goal_handle
        self._current_goal_handle = None
        if goal_handle is None:
            return
        try:
            future = goal_handle.cancel_goal_async()
            self._wait_future(future, timeout_seconds=1.0)
        except Exception as exc:
            self.get_logger().warning(f"failed to cancel nav goal: {exc}")

    def _wait_future(self, future, *, timeout_seconds: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        while not future.done() and time.monotonic() < deadline and not self._task_cancel.is_set():
            time.sleep(0.05)
        return future.done()

    def _publish_algorithms(self, algorithms: list[str]) -> None:
        self._algorithm_pub.publish(String(data=json.dumps({"algorithm_ids": algorithms}, separators=(",", ":"))))

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_state(self) -> None:
        self._status_pub.publish(String(data=json.dumps(self._snapshot_state(), ensure_ascii=False, separators=(",", ":"))))

    def _snapshot_state(self) -> dict[str, Any]:
        with self._state_lock:
            state = self._state.to_dict()
        return state

    def _update_state(self, **updates: Any) -> None:
        with self._state_lock:
            for key, value in updates.items():
                if hasattr(self._state, key):
                    setattr(self._state, key, value)
            self._state.pose = dict(self._pose)
            self._state.summary = dict(self._task_summary)
            self._state.updated_at = time.time()
        self._publish_state()

    def _sleep_interruptible(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and not self._task_cancel.is_set():
            time.sleep(0.1)

    def _waypoints_from_payload(self, payload: dict[str, Any], *, fallback_key: str) -> list[Waypoint]:
        raw = payload.get("waypoints")
        if raw is None:
            raw = self._waypoint_sets.get(fallback_key, [])
        return [_waypoint_from_dict(item) for item in raw if isinstance(item, dict)]

    def _load_waypoints(self) -> dict[str, list[dict[str, Any]]]:
        path = str(self.get_parameter("waypoints_file").value).strip()
        if not path:
            return {}
        try:
            import yaml

            data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
            if isinstance(data, dict):
                return {
                    str(key): value
                    for key, value in data.items()
                    if isinstance(value, list)
                }
        except Exception as exc:
            self.get_logger().warning(f"failed to load waypoints file {path}: {exc}")
        return {}


def _waypoint_from_dict(item: dict[str, Any]) -> Waypoint:
    return Waypoint(
        x=float(item["x"]),
        y=float(item["y"]),
        yaw=float(item.get("yaw", 0.0)),
        hold_seconds=float(item.get("hold_seconds", 1.0)),
        label=str(item.get("label") or ""),
    )


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
