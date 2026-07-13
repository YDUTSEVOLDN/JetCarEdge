from __future__ import annotations

import shlex
import subprocess
import threading
from dataclasses import dataclass
from typing import Callable, Optional


LogFn = Callable[[str], None]


@dataclass(frozen=True)
class DockerProgram:
    name: str
    command: str
    required: bool = True


class DockerOrchestrator:
    def __init__(
        self,
        *,
        container: str,
        programs: list[DockerProgram],
        docker_executable: str = "docker",
        command_prefix: str = "",
        enabled: bool = False,
        on_log: Optional[LogFn] = None,
    ) -> None:
        self._container = container
        self._programs = programs
        self._docker = docker_executable
        self._command_prefix = command_prefix.strip()
        self._enabled = enabled and bool(container)
        self._on_log = on_log or (lambda _msg: None)
        self._lock = threading.Lock()
        self._started = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def start_stack(self) -> None:
        if not self._enabled:
            self._on_log("docker orchestrator disabled; skip start stack")
            return
        with self._lock:
            if self._started:
                return
            self._run([self._docker, "start", self._container], required=True)
            for item in self._programs:
                command = item.command
                if self._command_prefix:
                    command = f"{self._command_prefix} && {command}"
                self._run(
                    [
                        self._docker,
                        "exec",
                        "-d",
                        self._container,
                        "bash",
                        "-lc",
                        command,
                    ],
                    required=item.required,
                )
            self._started = True

    def stop_stack(self) -> None:
        if not self._enabled:
            return
        with self._lock:
            if not self._started:
                return
            self._run([self._docker, "stop", self._container], required=False)
            self._started = False

    def _run(self, command: list[str], *, required: bool) -> None:
        display = " ".join(shlex.quote(item) for item in command)
        self._on_log(f"docker command: {display}")
        completed = subprocess.run(command, capture_output=True, text=True, timeout=20)
        if completed.stdout.strip():
            self._on_log(f"docker stdout: {completed.stdout.strip()}")
        if completed.stderr.strip():
            self._on_log(f"docker stderr: {completed.stderr.strip()}")
        if required and completed.returncode != 0:
            raise RuntimeError(f"docker command failed with {completed.returncode}: {display}")
