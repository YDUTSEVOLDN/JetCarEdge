from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable, Dict, Optional

import websocket


LogFn = Callable[[str], None]
ResultFn = Callable[[Dict[str, Any]], None]


class CloudResultClient:
    def __init__(
        self,
        url: str,
        *,
        reconnect_seconds: float = 2.0,
        on_result: Optional[ResultFn] = None,
        on_log: Optional[LogFn] = None,
    ) -> None:
        self._url = url
        self._reconnect_seconds = reconnect_seconds
        self._on_result = on_result
        self._on_log = on_log or (lambda _msg: None)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="jetcar-cloud-results", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def update_url(self, url: str) -> None:
        if url == self._url:
            return
        was_running = self.is_running
        if was_running:
            self.stop()
        self._url = url
        if was_running:
            self.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            ws = None
            try:
                self._on_log(f"connecting cloud result channel {self._url}")
                ws = websocket.create_connection(self._url, timeout=5)
                ws.settimeout(5)
                self._on_log("cloud result channel connected")
                self._receive_loop(ws)
            except Exception as exc:
                self._on_log(f"cloud result channel error: {exc}")
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
            self._stop.wait(self._reconnect_seconds)

    def _receive_loop(self, ws: websocket.WebSocket) -> None:
        next_ping = time.monotonic() + 15.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_ping:
                try:
                    ws.send("ping")
                except Exception:
                    raise
                next_ping = now + 15.0

            try:
                raw = ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            if not raw:
                continue
            result = json.loads(raw)
            if isinstance(result, dict) and self._on_result:
                self._on_result(result)
