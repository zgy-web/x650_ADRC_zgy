from __future__ import annotations
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]

def env_path(name: str, default: Path) -> Path:
    value = os.environ.get(name)
    return Path(value).expanduser().resolve() if value else default.resolve()

def workspace_root() -> Path:
    return env_path("AMSIM_WORKSPACE", PROJECT_ROOT.parent)

def px4_dir() -> Path:
    return env_path("AMSIM_PX4_DIR", workspace_root() / "PX4-Autopilot")

def ros_ws() -> Path:
    return env_path("AMSIM_ROS_WS", workspace_root() / "ros2_px4_ws")

def results_dir() -> Path:
    return env_path("AMSIM_RESULTS_DIR", PROJECT_ROOT / "runs")

def reference_dir() -> Path:
    return PROJECT_ROOT / "legacy" / "v3_reference"

def model_dir() -> Path:
    return PROJECT_ROOT / "assets" / "models" / "x500_underfold"
