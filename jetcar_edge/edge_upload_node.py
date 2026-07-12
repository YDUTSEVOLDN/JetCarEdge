from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Empty, String

from jetcar_edge.image_codec import ImageCodec
from jetcar_edge.models import VideoFrameUpload
from jetcar_edge.safety import SafetyMonitor
from jetcar_edge.sensor_buffer import SensorBuffer
from jetcar_edge.ws_client import CloudWsClient


class EdgeUploadNode(Node):
    def __init__(self) -> None:
        super().__init__("jetcar_edge_upload")

        self.declare_parameter("car_id", "car_001")
        self.declare_parameter(
            "cloud_url",
            "ws://192.168.137.1:8000/ws/video/car_001/camera_front/edge?algorithm_id=yolov5-similarity",
        )
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("ai_enable_topic", "/jetcar/ai_enable")
        self.declare_parameter("snapshot_topic", "/jetcar/snapshot")
        self.declare_parameter("ai_result_topic", "/jetcar/ai_result")
        self.declare_parameter("emergency_stop_topic", "/jetcar/emergency_stop")
        self.declare_parameter("upload_fps", 5.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("queue_size", 2)
        self.declare_parameter("danger_distance_m", 1.5)
        self.declare_parameter("reconnect_seconds", 2.0)

        self._car_id = self.get_parameter("car_id").value
        self._upload_interval = 1.0 / max(float(self.get_parameter("upload_fps").value), 0.1)
        self._last_upload_at = 0.0
        self._upload_enabled = True
        self._snapshot_requested = False

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

        self.create_subscription(
            Bool,
            str(self.get_parameter("ai_enable_topic").value),
            self._on_ai_enable,
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
            Image,
            str(self.get_parameter("camera_topic").value),
            self._on_image,
            10,
        )

        self._cloud = CloudWsClient(
            str(self.get_parameter("cloud_url").value),
            queue_size=int(self.get_parameter("queue_size").value),
            reconnect_seconds=float(self.get_parameter("reconnect_seconds").value),
            expect_response=False,
            on_result=self._on_cloud_result,
            on_log=lambda msg: self.get_logger().info(msg),
        )
        self._cloud.start()
        self.get_logger().info("JetCar edge upload node started")

    def destroy_node(self) -> bool:
        self._cloud.stop()
        return super().destroy_node()

    def _on_ai_enable(self, msg: Bool) -> None:
        self._upload_enabled = bool(msg.data)
        self.get_logger().info(f"AI upload enabled={self._upload_enabled}")

    def _on_snapshot(self, _msg: Empty) -> None:
        self._snapshot_requested = True
        self.get_logger().info("single-frame snapshot requested")

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        due = now - self._last_upload_at >= self._upload_interval
        should_upload = self._upload_enabled and due
        if not should_upload and not self._snapshot_requested:
            return

        self._last_upload_at = now
        self._snapshot_requested = False

        try:
            encoded = self._codec.encode(msg)
            frame = VideoFrameUpload(
                car_id=str(self._car_id),
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

        dangerous = self._safety.is_dangerous(result)
        self._emergency_pub.publish(Bool(data=dangerous))
        if dangerous:
            self.get_logger().warning("dangerous object detected; emergency_stop=true")


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
