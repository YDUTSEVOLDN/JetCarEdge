from __future__ import annotations

import json
import queue
import threading
import time
from collections.abc import Callable
from typing import Any, Dict, Optional

import websocket


LogFn = Callable[[str], None]
ResultFn = Callable[[Dict[str, Any]], None]


class CloudWsClient:
    def __init__(
        self,
        url: str,
        *,
        queue_size: int = 2,
        reconnect_seconds: float = 2.0,
        expect_response: bool = False,
        on_result: Optional[ResultFn] = None,
        on_log: Optional[LogFn] = None,
    ) -> None:
        self._url = url
        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._reconnect_seconds = reconnect_seconds
        self._expect_response = expect_response
        self._on_result = on_result
        self._on_log = on_log or (lambda _msg: None)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="jetcar-cloud-ws", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def submit(self, payload: Dict[str, Any]) -> None:
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self._queue.put_nowait(payload)

    def _run(self) -> None:
        while not self._stop.is_set():
            ws = None
            try:
                self._on_log(f"connecting cloud {self._url}")
                ws = websocket.create_connection(self._url, timeout=5)
                ws.settimeout(5)
                self._on_log("cloud connected")
                self._send_loop(ws)
            except Exception as exc:
                self._on_log(f"cloud connection error: {exc}")
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
            self._stop.wait(self._reconnect_seconds)

    def _send_loop(self, ws: websocket.WebSocket) -> None:
        while not self._stop.is_set():
            try:
                payload = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue

            ws.send(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
            car_id = payload.get("car_id", "unknown")
            self._on_log(f"frame sent to cloud for car_id={car_id}")
            if self._expect_response:
                raw = ws.recv()
                if not raw:
                    continue

                result = json.loads(raw)
                if isinstance(result, dict) and self._on_result:
                    self._on_result(result)
            time.sleep(0)
