"""Thin subprocess wrapper for vision_cli. FastAPI is not wired yet."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path


class VisionCliError(RuntimeError):
    pass


def run_vision_cli(cli: str | Path, args: list[str], timeout_s: float = 120.0) -> dict:
    proc = subprocess.run(
        [str(cli), *args],
        capture_output=True,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    if proc.returncode != 0:
        raise VisionCliError(f"exit {proc.returncode}: {proc.stderr.strip() or proc.stdout.strip()}")
    lines = [ln for ln in proc.stdout.splitlines() if ln.strip()]
    if not lines:
        raise VisionCliError("vision_cli produced no stdout JSON")
    return json.loads(lines[-1])
