from __future__ import annotations

import os
import shlex
import subprocess
import threading
import time
from typing import Any, Optional


class VideoStreamManager:
    def __init__(
        self,
        *,
        port: int = 8080,
        cmd: str = "web_video_server --port {port}",
    ) -> None:
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._port = int(port)
        self._cmd = cmd

    @property
    def port(self) -> int:
        return self._port

    def status(self) -> dict[str, Any]:
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            pid = int(self._proc.pid) if running and self._proc is not None else None
            return {"running": running, "pid": pid, "port": self._port}

    def start(self) -> dict[str, Any]:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return {"ok": True, **self.status()}

            rendered = self._cmd.format(port=self._port)
            argv = shlex.split(rendered)

            env = os.environ.copy()
            self._proc = subprocess.Popen(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
            )
            time.sleep(0.2)
            if self._proc.poll() is not None:
                err = ""
                try:
                    if self._proc.stderr is not None:
                        err = (self._proc.stderr.read() or b"")[:2000].decode("utf-8", errors="replace")
                except Exception:
                    err = ""
                return {"ok": False, "error": (err.strip() or "process exited"), **self.status()}

            return {"ok": True, **self.status()}

    def stop(self) -> dict[str, Any]:
        with self._lock:
            proc = self._proc
            self._proc = None
        if proc is None:
            return {"ok": True, "running": False, "pid": None, "port": self._port}

        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3.0)
                except Exception:
                    proc.kill()
        except Exception as exc:
            return {"ok": False, "error": str(exc), "running": False, "pid": None, "port": self._port}

        return {"ok": True, "running": False, "pid": None, "port": self._port}
