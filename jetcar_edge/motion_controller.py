from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from geometry_msgs.msg import Twist

from jetcar_edge.sensor_buffer import SensorBuffer


LogFn = Callable[[str], None]


@dataclass
class VisualServoConfig:
    enabled: bool = True
    cmd_vel_topic: str = "/cmd_vel"
    align_tolerance: float = 0.12
    target_stop_distance_m: float = 0.7
    safety_stop_distance_m: float = 0.35
    lost_timeout_seconds: float = 3.0
    search_angular_z: float = 0.22
    align_angular_gain: float = 0.8
    approach_linear_x: float = 0.12
    approach_angular_gain: float = 0.45
    command_timeout_seconds: float = 0.8


class VisualServoController:
    def __init__(
        self,
        *,
        config: VisualServoConfig,
        sensor_buffer: SensorBuffer,
        cmd_pub,
        on_log: Optional[LogFn] = None,
    ) -> None:
        self._config = config
        self._sensors = sensor_buffer
        self._cmd_pub = cmd_pub
        self._on_log = on_log or (lambda _msg: None)
        self._active = False
        self._state = "idle"
        self._last_target_at = 0.0
        self._last_command_at = 0.0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def state(self) -> str:
        return self._state

    def start(self) -> None:
        if not self._config.enabled:
            self._on_log("visual servo disabled; motion commands will not be published")
            return
        self._active = True
        self._state = "searching"
        self._last_target_at = 0.0
        self._publish(0.0, self._config.search_angular_z)

    def stop(self, reason: str = "stopped") -> None:
        if not self._active and self._state == "idle":
            return
        self._publish_stop()
        self._active = False
        self._state = "idle"
        self._on_log(f"visual servo stopped reason={reason}")

    def handle_similarity_result(self, result: dict[str, Any]) -> dict[str, Any]:
        if not self._active or not self._config.enabled:
            return {"motion_state": self._state, "command": "disabled"}

        matched = _as_bool(result.get("matched"))
        similarity = _as_float(result.get("similarity"), 0.0)
        center_norm = _as_float_pair(result.get("center_norm"))
        front_distance = self._sensors.front_distance_m()

        if front_distance is not None and front_distance <= self._config.safety_stop_distance_m:
            self._state = "safety_stop"
            self._publish_stop()
            return {
                "motion_state": self._state,
                "command": "stop",
                "reason": "front_obstacle_too_close",
                "front_distance_m": front_distance,
                "similarity": similarity,
            }

        if not matched or center_norm is None:
            self._state = "searching"
            self._publish(0.0, self._config.search_angular_z)
            return {
                "motion_state": self._state,
                "command": "search",
                "angular_z": self._config.search_angular_z,
                "front_distance_m": front_distance,
                "similarity": similarity,
            }

        self._last_target_at = time.monotonic()
        x_error = center_norm[0] - 0.5
        if abs(x_error) > self._config.align_tolerance:
            angular_z = -x_error * self._config.align_angular_gain
            self._state = "aligning"
            self._publish(0.0, angular_z)
            return {
                "motion_state": self._state,
                "command": "align",
                "center_norm": center_norm,
                "x_error": round(x_error, 4),
                "angular_z": round(angular_z, 4),
                "front_distance_m": front_distance,
                "similarity": similarity,
            }

        if front_distance is not None and front_distance <= self._config.target_stop_distance_m:
            self._state = "arrived"
            self._publish_stop()
            return {
                "motion_state": self._state,
                "command": "stop",
                "reason": "target_distance_reached",
                "center_norm": center_norm,
                "front_distance_m": front_distance,
                "similarity": similarity,
            }

        if front_distance is None:
            self._state = "waiting_distance"
            self._publish_stop()
            return {
                "motion_state": self._state,
                "command": "stop",
                "reason": "front_distance_unavailable",
                "center_norm": center_norm,
                "similarity": similarity,
            }

        angular_z = -x_error * self._config.approach_angular_gain
        self._state = "approaching"
        self._publish(self._config.approach_linear_x, angular_z)
        return {
            "motion_state": self._state,
            "command": "approach",
            "center_norm": center_norm,
            "x_error": round(x_error, 4),
            "linear_x": self._config.approach_linear_x,
            "angular_z": round(angular_z, 4),
            "front_distance_m": front_distance,
            "similarity": similarity,
        }

    def tick(self) -> dict[str, Any] | None:
        if not self._active or not self._config.enabled:
            return None
        now = time.monotonic()
        if (
            self._last_target_at > 0.0
            and now - self._last_target_at > self._config.lost_timeout_seconds
        ):
            self._state = "searching"
            self._last_target_at = 0.0
            self._publish(0.0, self._config.search_angular_z)
            return {"motion_state": self._state, "command": "search", "reason": "target_lost"}
        if now - self._last_command_at > self._config.command_timeout_seconds:
            if self._state == "searching":
                self._publish(0.0, self._config.search_angular_z)
            elif self._state in {"aligning", "approaching"}:
                self._publish_stop()
                return {"motion_state": self._state, "command": "stop", "reason": "command_timeout"}
        return None

    def _publish(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)
        self._last_command_at = time.monotonic()

    def _publish_stop(self) -> None:
        self._publish(0.0, 0.0)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"true", "1", "yes", "matched", "found"}
    if isinstance(value, (int, float)):
        return value != 0
    return False


def _as_float(value: Any, default: float) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return default
    return default


def _as_float_pair(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    x = _as_float(value[0], -1.0)
    y = _as_float(value[1], -1.0)
    if x < 0.0 or y < 0.0:
        return None
    return [max(0.0, min(1.0, x)), max(0.0, min(1.0, y))]
