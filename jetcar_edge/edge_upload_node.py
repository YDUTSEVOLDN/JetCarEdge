from __future__ import annotations

import json
import base64
import os
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib import request
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Empty, String

from jetcar_edge.cloud_result_client import CloudResultClient
from jetcar_edge.cloud_discovery import CloudDiscoveryConfig, discover_cloud_host
from jetcar_edge.docker_orchestrator import DockerOrchestrator, DockerProgram
from jetcar_edge.image_codec import ImageCodec
from jetcar_edge.models import VideoFrameUpload
from jetcar_edge.motion_controller import VisualServoConfig, VisualServoController
from jetcar_edge.safety import SafetyMonitor
from jetcar_edge.sensor_buffer import SensorBuffer
from jetcar_edge.similarity_search_controller import SimilaritySearchConfig, SimilaritySearchController
from jetcar_edge.video_stream_manager import VideoStreamManager
from jetcar_edge.ws_client import CloudWsClient


class EdgeUploadNode(Node):
    def __init__(self) -> None:
        super().__init__("jetcar_edge_upload")

        self.declare_parameter("car_id", "car_001")
        self.declare_parameter("stream_id", "camera_front")
        self.declare_parameter("cloud_host", "192.168.175.90")
        self.declare_parameter("cloud_port", 8000)
        self.declare_parameter("cloud_discovery_enabled", False)
        self.declare_parameter("cloud_discovery_port", 8765)
        self.declare_parameter("cloud_discovery_listen_seconds", 2.0)
        self.declare_parameter("cloud_discovery_service", "JetCarCloud")
        self.declare_parameter(
            "algorithm_ids",
            [],
        )
        self.declare_parameter(
            "cloud_url",
            "",
        )
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("ai_enable_topic", "/jetcar/ai_enable")
        self.declare_parameter("algorithm_control_topic", "/jetcar/algorithm_ids")
        self.declare_parameter("app_control_host", "0.0.0.0")
        self.declare_parameter("app_control_port", 6001)
        self.declare_parameter("frame_server_host", "0.0.0.0")
        self.declare_parameter("frame_server_port", 8100)
        self.declare_parameter("snapshot_topic", "/jetcar/snapshot")
        self.declare_parameter("ai_result_topic", "/jetcar/ai_result")
        self.declare_parameter("emergency_stop_topic", "/jetcar/emergency_stop")
        self.declare_parameter("task_status_topic", "/jetcar/task_status")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("upload_fps", 5.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("queue_size", 2)
        self.declare_parameter("danger_distance_m", 1.5)
        self.declare_parameter("reconnect_seconds", 2.0)
        self.declare_parameter("docker_orchestrator_enabled", False)
        self.declare_parameter("docker_executable", "docker")
        self.declare_parameter("docker_command_prefix", "")
        self.declare_parameter("autodrive_container", "")
        self.declare_parameter(
            "similarity_search_programs",
            [
                "ros2 run yahboomcar_bringup Mcnamu_driver_X3",
                "ros2 launch sllidar_ros2 sllidar_launch.py",
                "ros2 launch astra_camera astra.launch.xml",
            ],
        )
        self.declare_parameter("similarity_match_threshold", 0.45)
        self.declare_parameter("similarity_auto_stop_on_match", True)
        self.declare_parameter("similarity_motion_enabled", True)
        self.declare_parameter("similarity_align_tolerance", 0.12)
        self.declare_parameter("similarity_target_stop_distance_m", 0.7)
        self.declare_parameter("similarity_safety_stop_distance_m", 0.35)
        self.declare_parameter("similarity_lost_timeout_seconds", 3.0)
        self.declare_parameter("similarity_search_angular_z", 0.22)
        self.declare_parameter("similarity_align_angular_gain", 0.8)
        self.declare_parameter("similarity_approach_linear_x", 0.12)
        self.declare_parameter("similarity_approach_angular_gain", 0.45)

        self._car_id = str(self.get_parameter("car_id").value)
        self._stream_id = str(self.get_parameter("stream_id").value)
        self._cloud_host = self._resolve_cloud_host()
        self._algorithm_ids = self._read_algorithm_ids()
        self._upload_interval = 1.0 / max(float(self.get_parameter("upload_fps").value), 0.1)
        self._last_upload_at = 0.0
        self._upload_enabled = False
        self._snapshot_requested = False
        self._control_server = None
        self._control_thread = None
        self._frame_server = None
        self._frame_thread = None
        self._latest_jpeg = None
        self._latest_jpeg_lock = threading.Lock()
        self._camera_frame_count = 0
        self._last_camera_frame_at = 0.0
        self._last_camera_status_log_at = 0.0
        self._video_stream = VideoStreamManager(
            port=int(os.getenv("WEB_VIDEO_SERVER_PORT", "8080")),
            cmd=os.getenv("WEB_VIDEO_SERVER_CMD", "web_video_server --port {port}"),
        )

        self._codec = ImageCodec(
            target_width=int(self.get_parameter("image_width").value),
            jpeg_quality=int(self.get_parameter("jpeg_quality").value),
        )
        self._sensors = SensorBuffer()
        self._safety = SafetyMonitor(
            danger_distance_m=float(self.get_parameter("danger_distance_m").value),
        )

        self._result_pub = self.create_publisher(
            String,
            str(self.get_parameter("ai_result_topic").value),
            10,
        )
        self._emergency_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            10,
        )
        self._cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )

        self.create_subscription(
            Bool,
            str(self.get_parameter("ai_enable_topic").value),
            self._on_ai_enable,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("algorithm_control_topic").value),
            self._on_algorithm_ids,
            10,
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("snapshot_topic").value),
            self._on_snapshot,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._sensors.update_lidar,
            10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._sensors.update_imu,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("task_status_topic").value),
            self._on_task_status,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self._on_image,
            10,
        )

        self._cloud = CloudWsClient(
            self._cloud_url(),
            queue_size=int(self.get_parameter("queue_size").value),
            reconnect_seconds=float(self.get_parameter("reconnect_seconds").value),
            expect_response=False,
            on_result=self._on_cloud_result,
            on_log=lambda msg: self.get_logger().info(msg),
        )
        self._cloud_results = CloudResultClient(
            self._cloud_result_url(),
            reconnect_seconds=float(self.get_parameter("reconnect_seconds").value),
            on_result=self._on_cloud_result,
            on_log=lambda msg: self.get_logger().info(msg),
        )
        self._orchestrator = DockerOrchestrator(
            container=str(self.get_parameter("autodrive_container").value).strip(),
            programs=self._read_similarity_search_programs(),
            docker_executable=str(self.get_parameter("docker_executable").value).strip() or "docker",
            command_prefix=str(self.get_parameter("docker_command_prefix").value).strip(),
            enabled=bool(self.get_parameter("docker_orchestrator_enabled").value),
            on_log=lambda msg: self.get_logger().info(msg),
        )
        self._motion = VisualServoController(
            config=VisualServoConfig(
                enabled=bool(self.get_parameter("similarity_motion_enabled").value),
                cmd_vel_topic=str(self.get_parameter("cmd_vel_topic").value),
                align_tolerance=float(self.get_parameter("similarity_align_tolerance").value),
                target_stop_distance_m=float(self.get_parameter("similarity_target_stop_distance_m").value),
                safety_stop_distance_m=float(self.get_parameter("similarity_safety_stop_distance_m").value),
                lost_timeout_seconds=float(self.get_parameter("similarity_lost_timeout_seconds").value),
                search_angular_z=float(self.get_parameter("similarity_search_angular_z").value),
                align_angular_gain=float(self.get_parameter("similarity_align_angular_gain").value),
                approach_linear_x=float(self.get_parameter("similarity_approach_linear_x").value),
                approach_angular_gain=float(self.get_parameter("similarity_approach_angular_gain").value),
            ),
            sensor_buffer=self._sensors,
            cmd_pub=self._cmd_pub,
            on_log=lambda msg: self.get_logger().info(msg),
        )
        self._similarity_controller = SimilaritySearchController(
            car_id=self._car_id,
            stream_id=self._stream_id,
            config=SimilaritySearchConfig(
                match_threshold=float(self.get_parameter("similarity_match_threshold").value),
                auto_stop_on_match=bool(self.get_parameter("similarity_auto_stop_on_match").value),
                stop_on_arrival=bool(self.get_parameter("similarity_auto_stop_on_match").value),
            ),
            orchestrator=self._orchestrator,
            motion=self._motion,
            result_pub=self._result_pub,
            stop_ai=lambda reason: self._set_ai_algorithms([], reason=reason),
            report_event=self._report_edge_event,
            on_log=lambda msg: self.get_logger().info(msg),
        )
        self._start_control_server()
        self._start_frame_server()
        self._cloud_results.start()
        self.create_timer(0.5, self._on_timer)
        self.create_timer(5.0, self._log_camera_status)
        self.get_logger().info(
            f"JetCar edge upload node started stream_id={self._stream_id} upload_enabled={self._upload_enabled}"
        )

    def destroy_node(self) -> bool:
        self._stop_control_server()
        self._stop_frame_server()
        self._video_stream.stop()
        self._similarity_controller.stop(reason="node_destroy")
        self._cloud_results.stop()
        self._cloud.stop()
        return super().destroy_node()

    def _on_ai_enable(self, msg: Bool) -> None:
        self._set_ai_enabled(bool(msg.data), reason="ros_ai_enable")

    def _on_snapshot(self, _msg: Empty) -> None:
        self._snapshot_requested = True
        self.get_logger().info("single-frame snapshot requested")

    def _on_algorithm_ids(self, msg: String) -> None:
        algorithms = self._parse_algorithm_text(msg.data)
        if not algorithms:
            self._set_ai_algorithms([], reason="ros_algorithm_ids_empty")
            self._similarity_controller.stop(reason="ros_algorithm_ids_empty")
            return
        self._set_ai_algorithms(algorithms, reason="ros_algorithm_ids")
        if "yolov5-similarity" in algorithms:
            self._similarity_controller.start()
        else:
            self._similarity_controller.stop(reason="ros_algorithm_ids_without_similarity")

    def _on_task_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return
        self._report_edge_event("task_status", payload)

    def _set_ai_enabled(self, enabled: bool, *, reason: str) -> None:
        if self._upload_enabled == enabled:
            return
        self._upload_enabled = enabled
        if enabled:
            self._cloud.update_url(self._cloud_url())
            self._cloud.start()
        else:
            self._cloud.stop()
        self.get_logger().info(f"AI upload enabled={enabled} reason={reason}")

    def _set_ai_algorithms(self, algorithms: list[str], *, reason: str) -> None:
        if algorithms == self._algorithm_ids:
            self._set_ai_enabled(bool(algorithms), reason=reason)
            return
        self._algorithm_ids = algorithms
        if algorithms:
            self._cloud.update_url(self._cloud_url())
            self._set_ai_enabled(True, reason=reason)
        else:
            self._set_ai_enabled(False, reason=reason)
        self.get_logger().info(f"algorithm_ids updated: {','.join(self._algorithm_ids) or '<none>'}")

    def _on_app_control(self, payload: dict) -> dict:
        cmd = str(payload.get("cmd") or "").strip().lower()
        if cmd == "start_video_stream":
            status = self._video_stream.start()
            return {"ok": bool(status.get("ok", False)), "cmd": cmd, "video": status}
        if cmd == "stop_video_stream":
            status = self._video_stream.stop()
            return {"ok": bool(status.get("ok", False)), "cmd": cmd, "video": status}

        mode = str(payload.get("mode") or "").strip().lower()
        algorithms = self._algorithms_from_control(payload)
        self._set_ai_algorithms(algorithms, reason="app_control")
        if mode in {"similarity", "search"}:
            self._similarity_controller.start()
        elif not algorithms:
            self._similarity_controller.stop(reason="app_control_off")
        return {
            "ok": True,
            "car_id": self._car_id,
            "stream_id": self._stream_id,
            "upload_enabled": self._upload_enabled,
            "algorithm_ids": self._algorithm_ids,
            "similarity_search": {
                "active": self._similarity_controller.active,
                "state": self._similarity_controller.state,
            },
            "cloud_url": self._cloud_url() if self._algorithm_ids else "",
        }

    def _on_image(self, msg: Image) -> None:
        self._camera_frame_count += 1
        self._last_camera_frame_at = time.monotonic()
        now = time.monotonic()
        due = now - self._last_upload_at >= self._upload_interval
        should_upload = self._upload_enabled and due
        should_cache = self._frame_server is not None
        if not should_upload and not self._snapshot_requested and not should_cache:
            return

        snapshot_requested = self._snapshot_requested
        if should_upload:
            self._last_upload_at = now
        self._snapshot_requested = False

        try:
            encoded = self._codec.encode(msg)
            self._store_latest_jpeg(encoded)
            if should_upload or snapshot_requested:
                frame = VideoFrameUpload(
                    car_id=self._car_id,
                    image=encoded,
                )
                self._cloud.submit(frame.to_dict())
                self.get_logger().info(
                    f"frame queued for cloud upload: {encoded.width}x{encoded.height}"
                )
        except Exception as exc:
            self.get_logger().warning(f"failed to process camera frame: {exc}")

    def _on_cloud_result(self, result: dict) -> None:
        text = json.dumps(result, ensure_ascii=False, separators=(",", ":"))
        self._result_pub.publish(String(data=text))
        self._similarity_controller.handle_cloud_result(result)

        dangerous = self._safety.is_dangerous(result)
        self._emergency_pub.publish(Bool(data=dangerous))
        if dangerous:
            self.get_logger().warning("dangerous object detected; emergency_stop=true")

    def _cloud_url(self) -> str:
        explicit = str(self.get_parameter("cloud_url").value).strip()
        if explicit:
            return self._with_algorithm_query(explicit)
        host = self._cloud_host
        port = int(self.get_parameter("cloud_port").value)
        algorithms = ",".join(self._algorithm_ids)
        return (
            f"ws://{host}:{port}/ws/video/{self._car_id}/{self._stream_id}/edge"
            f"?algorithm_ids={algorithms}&include_image=true"
        )

    def _cloud_result_url(self) -> str:
        host = self._cloud_host
        port = int(self.get_parameter("cloud_port").value)
        return f"ws://{host}:{port}/ws/inference/{self._car_id}/app"

    def _cloud_edge_event_url(self) -> str:
        host = self._cloud_host
        port = int(self.get_parameter("cloud_port").value)
        return f"http://{host}:{port}/api/edge/events"

    def _report_edge_event(self, event: str, payload: dict) -> None:
        if event not in {
            "search_started",
            "target_tracking",
            "target_found",
            "search_stopped",
            "search_warning",
            "motion_update",
            "task_status",
        }:
            return
        if event == "target_found" and not payload.get("final_image"):
            final_image = self._latest_image_payload()
            if final_image is not None:
                payload = dict(payload)
                payload["final_image"] = final_image
        threading.Thread(
            target=self._post_edge_event,
            args=(event, payload),
            name=f"jetcar-edge-event-{event}",
            daemon=True,
        ).start()

    def _post_edge_event(self, event: str, payload: dict) -> None:
        try:
            body = json.dumps(
                {
                    "car_id": self._car_id,
                    "stream_id": self._stream_id,
                    "event": event,
                    "payload": payload,
                },
                ensure_ascii=False,
            ).encode("utf-8")
            req = request.Request(
                self._cloud_edge_event_url(),
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with request.urlopen(req, timeout=2.0) as response:
                response.read()
        except Exception as exc:
            self.get_logger().warning(f"failed to report edge event {event}: {exc}")

    def _on_timer(self) -> None:
        self._similarity_controller.tick()

    def _log_camera_status(self) -> None:
        age = None
        if self._last_camera_frame_at > 0.0:
            age = time.monotonic() - self._last_camera_frame_at
        age_text = "never" if age is None else f"{age:.1f}s ago"
        self.get_logger().info(
            f"camera status topic={str(self.get_parameter('camera_topic').value)} "
            f"frames={self._camera_frame_count} last_frame={age_text} "
            f"upload_enabled={self._upload_enabled} algorithms={','.join(self._algorithm_ids) or '<none>'}"
        )

    def _start_control_server(self) -> None:
        host = str(self.get_parameter("app_control_host").value).strip()
        port = int(self.get_parameter("app_control_port").value)
        if port <= 0:
            self.get_logger().info("app control TCP server disabled")
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
                            raise ValueError("control payload must be a JSON object")
                        result = node._on_app_control(payload)
                    except Exception as exc:
                        result = {"ok": False, "error": str(exc)}
                    self.wfile.write((json.dumps(result, ensure_ascii=False) + "\n").encode("utf-8"))
                    self.wfile.flush()

        class ThreadingServer(socketserver.ThreadingTCPServer):
            allow_reuse_address = True

        self._control_server = ThreadingServer((host, port), Handler)
        self._control_thread = threading.Thread(
            target=self._control_server.serve_forever,
            name="jetcar-app-ai-control",
            daemon=True,
        )
        self._control_thread.start()
        self.get_logger().info(f"app AI control TCP server listening on {host}:{port}")

    def _stop_control_server(self) -> None:
        if self._control_server is None:
            return
        self._control_server.shutdown()
        self._control_server.server_close()
        if self._control_thread is not None:
            self._control_thread.join(timeout=2.0)
        self._control_server = None
        self._control_thread = None

    def _start_frame_server(self) -> None:
        host = str(self.get_parameter("frame_server_host").value).strip()
        port = int(self.get_parameter("frame_server_port").value)
        if port <= 0:
            self.get_logger().info("frame HTTP server disabled")
            return
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path.split("?", 1)[0] not in {"/api/frame", "/frame.jpg"}:
                    self.send_error(404, "not found")
                    return
                data = node._latest_jpeg_bytes()
                if data is None:
                    self.send_error(503, "no camera frame received yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, fmt: str, *args) -> None:
                node.get_logger().info("frame server: " + fmt % args)

        try:
            self._frame_server = ThreadingHTTPServer((host, port), Handler)
        except OSError as exc:
            self._frame_server = None
            self._frame_thread = None
            self.get_logger().warning(
                f"camera frame HTTP server disabled: {host}:{port} unavailable ({exc})"
            )
            return
        self._frame_thread = threading.Thread(
            target=self._frame_server.serve_forever,
            name="jetcar-frame-server",
            daemon=True,
        )
        self._frame_thread.start()
        self.get_logger().info(f"camera frame HTTP server listening on {host}:{port}")

    def _stop_frame_server(self) -> None:
        if self._frame_server is None:
            return
        self._frame_server.shutdown()
        self._frame_server.server_close()
        if self._frame_thread is not None:
            self._frame_thread.join(timeout=2.0)
        self._frame_server = None
        self._frame_thread = None

    def _store_latest_jpeg(self, encoded) -> None:
        try:
            data = base64.b64decode(encoded.data)
        except Exception:
            return
        with self._latest_jpeg_lock:
            self._latest_jpeg = data

    def _latest_jpeg_bytes(self):
        with self._latest_jpeg_lock:
            return self._latest_jpeg

    def _latest_image_payload(self):
        data = self._latest_jpeg_bytes()
        if data is None:
            return None
        return {
            "encoding": "jpeg",
            "data": base64.b64encode(data).decode("ascii"),
        }

    def _read_algorithm_ids(self) -> list[str]:
        value = self.get_parameter("algorithm_ids").value
        if isinstance(value, str):
            items = value.split(",")
        else:
            items = list(value)
        algorithms = [str(item).strip() for item in items if str(item).strip()]
        return algorithms

    def _read_similarity_search_programs(self) -> list[DockerProgram]:
        value = self.get_parameter("similarity_search_programs").value
        if isinstance(value, str):
            items = [item.strip() for item in value.split(";") if item.strip()]
        else:
            items = [str(item).strip() for item in list(value) if str(item).strip()]
        return [
            DockerProgram(name=f"similarity_search_{index}", command=command, required=False)
            for index, command in enumerate(items)
        ]

    def _resolve_cloud_host(self) -> str:
        configured = str(self.get_parameter("cloud_host").value).strip()
        discovered = discover_cloud_host(
            CloudDiscoveryConfig(
                enabled=bool(self.get_parameter("cloud_discovery_enabled").value),
                port=int(self.get_parameter("cloud_discovery_port").value),
                listen_seconds=float(self.get_parameter("cloud_discovery_listen_seconds").value),
                service_name=str(self.get_parameter("cloud_discovery_service").value),
            )
        )
        if discovered:
            self.get_logger().info(f"discovered cloud host {discovered}")
            return discovered
        if bool(self.get_parameter("cloud_discovery_enabled").value):
            self.get_logger().warning(f"cloud discovery timed out; using configured host {configured}")
        return configured

    def _with_algorithm_query(self, url: str) -> str:
        algorithms = ",".join(self._algorithm_ids)
        parts = urlsplit(url)
        query = {
            key: value
            for key, value in parse_qsl(parts.query, keep_blank_values=True)
            if key not in {"algorithm_id", "algorithm_ids", "algorithms", "include_image"}
        }
        if algorithms:
            query["algorithm_ids"] = algorithms
        query["include_image"] = "true"
        return urlunsplit(
            (
                parts.scheme,
                parts.netloc,
                parts.path,
                urlencode(query),
                parts.fragment,
            )
        )

    def _parse_algorithm_text(self, value: str) -> list[str]:
        text = value.strip()
        if not text:
            return []
        try:
            loaded = json.loads(text)
            if isinstance(loaded, list):
                return [str(item).strip() for item in loaded if str(item).strip()]
            if isinstance(loaded, dict):
                raw = loaded.get("algorithm_ids") or loaded.get("algorithms") or []
                if isinstance(raw, str):
                    return [item.strip() for item in raw.split(",") if item.strip()]
                return [str(item).strip() for item in raw if str(item).strip()]
        except json.JSONDecodeError:
            pass
        return [item.strip() for item in text.split(",") if item.strip()]

    def _algorithms_from_control(self, payload: dict) -> list[str]:
        raw_algorithms = payload.get("algorithm_ids") or payload.get("algorithms")
        if isinstance(raw_algorithms, str):
            return [item.strip() for item in raw_algorithms.split(",") if item.strip()]
        if isinstance(raw_algorithms, list):
            return [str(item).strip() for item in raw_algorithms if str(item).strip()]

        mode = str(payload.get("mode") or "").strip().lower()
        if mode in {"off", "stop", "idle", "none"}:
            return []
        if mode in {"similarity", "search"}:
            return ["yolov5-similarity"]

        mask = str(payload.get("mask") or "").strip().upper()
        if mask:
            if len(mask) < 2:
                raise ValueError("mask must contain at least two characters, for example TF or FF")
            algorithms = []
            if mask[0] == "T":
                algorithms.append("yolov5-manhole-detect")
            if mask[1] == "T":
                algorithms.append("yolov8-road-damage")
            return algorithms

        enabled = payload.get("enabled")
        if enabled is False:
            return []
        return self._algorithm_ids


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EdgeUploadNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
