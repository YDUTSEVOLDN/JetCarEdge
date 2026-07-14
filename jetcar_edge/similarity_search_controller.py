from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from std_msgs.msg import String

from jetcar_edge.docker_orchestrator import DockerOrchestrator
from jetcar_edge.motion_controller import VisualServoController


LogFn = Callable[[str], None]
StopAiFn = Callable[[str], None]
ReportEventFn = Callable[[str, Dict[str, Any]], None]


@dataclass
class SimilaritySearchConfig:
    algorithm_id: str = "yolov5-similarity"
    match_threshold: float = 0.45
    stale_result_seconds: float = 5.0
    auto_stop_on_match: bool = True
    start_docker_on_search: bool = True
    stop_on_arrival: bool = True


class SimilaritySearchController:
    def __init__(
        self,
        *,
        car_id: str,
        stream_id: str,
        config: SimilaritySearchConfig,
        orchestrator: DockerOrchestrator,
        motion: VisualServoController,
        result_pub,
        stop_ai: StopAiFn,
        report_event: Optional[ReportEventFn] = None,
        on_log: Optional[LogFn] = None,
    ) -> None:
        self._car_id = car_id
        self._stream_id = stream_id
        self._config = config
        self._orchestrator = orchestrator
        self._motion = motion
        self._result_pub = result_pub
        self._stop_ai = stop_ai
        self._report_event = report_event
        self._on_log = on_log or (lambda _msg: None)
        self._active = False
        self._state = "idle"
        self._started_at = 0.0
        self._last_result_at = 0.0
        self._motion_on_match_only = False

    @property
    def active(self) -> bool:
        return self._active

    @property
    def state(self) -> str:
        return self._state

    def start(self, *, motion_on_match_only: bool = False) -> None:
        if self._active:
            self._motion_on_match_only = motion_on_match_only
            return
        self._active = True
        self._state = "searching"
        self._started_at = time.time()
        self._last_result_at = 0.0
        self._motion_on_match_only = motion_on_match_only
        if self._config.start_docker_on_search:
            self._orchestrator.start_stack()
        if not self._motion_on_match_only:
            self._motion.start()
        self._publish_event("search_started", {"state": self._state})
        self._on_log("similarity search controller started")

    def stop(self, reason: str = "stopped") -> None:
        if not self._active:
            return
        self._active = False
        self._state = "idle"
        self._motion.stop(reason=reason)
        self._orchestrator.stop_stack()
        self._publish_event("search_stopped", {"reason": reason})
        self._on_log(f"similarity search controller stopped reason={reason}")

    def handle_cloud_result(self, message: Dict[str, Any]) -> None:
        if not self._active:
            return
        if message.get("type") != "algorithm_result":
            return
        if str(message.get("algorithm_id") or "") != self._config.algorithm_id:
            return
        if str(message.get("car_id") or "") != self._car_id:
            return
        incoming_stream = str(message.get("stream_id") or "")
        if incoming_stream and incoming_stream != self._stream_id:
            return

        self._last_result_at = time.time()
        result = message.get("result")
        if not isinstance(result, dict):
            result = {}
        matched = _as_bool(result.get("matched"))
        similarity = _as_float(result.get("similarity"), 0.0)
        center_norm = result.get("center_norm")
        if matched and self._motion_on_match_only and not self._motion.active:
            self._state = "target_locked"
            self._publish_event(
                "target_tracking",
                {
                    "similarity": similarity,
                    "center_norm": center_norm,
                    "motion": {
                        "motion_state": "pending_visual_servo",
                        "command": "cancel_nav_then_start_visual_servo",
                    },
                },
            )
            time.sleep(0.2)
            self._motion.start()
        motion = self._motion.handle_similarity_result(result)
        self._publish_event(
            "search_result",
            {
                "matched": matched,
                "similarity": similarity,
                "center_norm": center_norm,
                "latency_ms": message.get("latency_ms"),
                "motion": motion,
            },
        )
        if matched and similarity >= self._config.match_threshold:
            motion_state = str(motion.get("motion_state") or "")
            if motion_state in {"arrived", "safety_stop"} and self._config.stop_on_arrival:
                self._state = "found"
                self._publish_event(
                    "target_found",
                    {"similarity": similarity, "center_norm": center_norm, "motion": motion},
                )
                self._stop_ai("target_found")
                self.stop(reason="target_found")
            else:
                self._state = "approaching"
                self._publish_event(
                    "target_tracking",
                    {"similarity": similarity, "center_norm": center_norm, "motion": motion},
                )
                if motion_state == "disabled" and self._config.auto_stop_on_match:
                    self._stop_ai("target_found_motion_disabled")
                    self.stop(reason="target_found_motion_disabled")

    def tick(self) -> None:
        if not self._active:
            return
        if self._last_result_at > 0:
            elapsed = time.time() - self._last_result_at
            if elapsed > self._config.stale_result_seconds:
                self._publish_event("search_warning", {"reason": "cloud_result_stale", "elapsed_seconds": elapsed})
                self._last_result_at = time.time()
        motion = self._motion.tick()
        if motion is not None:
            self._publish_event("motion_update", {"motion": motion})

    def _publish_event(self, event: str, payload: Dict[str, Any]) -> None:
        message = {
            "type": "edge_similarity_search",
            "event": event,
            "car_id": self._car_id,
            "stream_id": self._stream_id,
            "state": self._state,
            "active": self._active,
            "started_at": self._started_at,
            **payload,
        }
        self._result_pub.publish(String(data=json.dumps(message, ensure_ascii=False, separators=(",", ":"))))
        if self._report_event is not None:
            try:
                self._report_event(event, message)
            except Exception as exc:
                self._on_log(f"failed to report edge event {event}: {exc}")


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
