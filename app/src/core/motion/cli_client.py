"""Thin subprocess wrapper around motion_cli.

stdout is one JSON object. stderr is streamed line-by-line into the Python logger.
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import threading
from typing import Any, Optional

logger = logging.getLogger(__name__)

_HERE = os.path.abspath(os.path.dirname(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, "../../../.."))
_DEFAULT_CLI = os.path.join(_HERE, "build", "motion_cli")
_DEFAULT_CONFIG = os.path.join(_REPO_ROOT, "configs", "aisprayer_config.yaml")


class MotionCliError(RuntimeError):
    def __init__(self, message: str, *, exit_code: Optional[int] = None, payload: Optional[dict] = None):
        super().__init__(message)
        self.exit_code = exit_code
        self.payload = payload or {}


def motion_cli_path() -> str:
    override = os.environ.get("MOTION_CLI")
    return override if override else _DEFAULT_CLI


def aisprayer_config_path() -> str:
    override = os.environ.get("AISPRAYER_CONFIG")
    return override if override else _DEFAULT_CONFIG


def _log_stderr_line(line: str) -> None:
    text = line.rstrip("\n")
    if not text:
        return
    if text.startswith("⚠️") or " failed" in text.lower() or "error" in text.lower():
        logger.warning(text)
    else:
        logger.info(text)


def _drain_stderr(pipe) -> None:
    try:
        while True:
            try:
                line = pipe.readline()
            except ValueError:
                break
            if line == "":
                break
            _log_stderr_line(line)
    finally:
        try:
            pipe.close()
        except Exception:
            pass


def run_motion_cli(args: list[str], *, timeout: float = 180.0) -> dict[str, Any]:
    cli = motion_cli_path()
    if not os.path.isfile(cli) or not os.access(cli, os.X_OK):
        raise MotionCliError(
            f"motion_cli 不存在或不可执行: {cli}。请先运行 app/src/core/motion/scripts/build.sh"
        )

    cmd = [cli, *args]
    logger.info("▶ %s", " ".join(cmd))
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
    except OSError as e:
        raise MotionCliError(f"无法启动 motion_cli: {e}") from e

    err_thread = threading.Thread(target=_drain_stderr, args=(proc.stderr,), daemon=True)
    err_thread.start()
    try:
        stdout = proc.stdout.read() if proc.stdout is not None else ""
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired as e:
        proc.kill()
        if proc.stdout is not None:
            proc.stdout.read()
        err_thread.join(timeout=2.0)
        raise MotionCliError(f"motion_cli 超时 ({timeout:.0f}s)") from e
    if proc.stdout is not None:
        proc.stdout.close()
    err_thread.join(timeout=2.0)

    payload: dict[str, Any] = {}
    raw = (stdout or "").strip()
    if raw:
        try:
            payload = json.loads(raw.splitlines()[-1])
        except json.JSONDecodeError as e:
            raise MotionCliError(
                f"motion_cli stdout 不是合法 JSON: {raw[:240]!r}",
                exit_code=proc.returncode,
            ) from e

    if proc.returncode != 0:
        msg = payload.get("message") or f"motion_cli 退出码 {proc.returncode}"
        raise MotionCliError(msg, exit_code=proc.returncode, payload=payload)
    if not payload:
        raise MotionCliError("motion_cli 没有输出 JSON", exit_code=proc.returncode)
    return payload


def _csv(values: list[float] | tuple[float, ...]) -> str:
    return ",".join(str(v) for v in values)


def verify_path(
    input_yaml: str,
    output_yaml: str,
    *,
    speed_mm_s: Optional[float] = None,
    step_mm: Optional[float] = None,
    timeout: float = 180.0,
) -> dict[str, Any]:
    args = ["--config", aisprayer_config_path(), "verify", "--input", input_yaml, "--output", output_yaml]
    if speed_mm_s is not None:
        args += ["--speed", str(speed_mm_s)]
    if step_mm is not None:
        args += ["--step", str(step_mm)]
    return run_motion_cli(args, timeout=timeout)


def optimize_path(
    input_yaml: str,
    output_yaml: str,
    *,
    state_type: str = "auto_poi",
    anchor_source: Optional[str] = None,
    ref_rpy_deg: Optional[list[float]] = None,
    tolerance_rpy_deg: Optional[list[float]] = None,
    speed_mm_s: Optional[float] = None,
    step_mm: Optional[float] = None,
    timeout: float = 180.0,
) -> dict[str, Any]:
    args = [
        "--config",
        aisprayer_config_path(),
        "optimize",
        "--input",
        input_yaml,
        "--output",
        output_yaml,
        "--state-type",
        state_type,
    ]
    if anchor_source:
        args += ["--anchor-source", anchor_source]
    if ref_rpy_deg and len(ref_rpy_deg) == 3:
        args += ["--ref-rpy", _csv(ref_rpy_deg)]
    if tolerance_rpy_deg and len(tolerance_rpy_deg) == 3:
        args += ["--anchor-tol", _csv(tolerance_rpy_deg)]
    if speed_mm_s is not None:
        args += ["--speed", str(speed_mm_s)]
    if step_mm is not None:
        args += ["--step", str(step_mm)]
    return run_motion_cli(args, timeout=timeout)
