#!/usr/bin/env bash
# Create a project-local numerical environment; never install into the host Python.
set -euo pipefail

COMPARISON_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPARISON_VENV="$COMPARISON_ROOT/.amsim/comparison-venv"
COMPARISON_REQUIREMENTS="$COMPARISON_ROOT/simulations/x650_control_comparison_v1/requirements.txt"
export PYTHONNOUSERSITE=1

# Ubuntu's system pip is intentionally inherited. --without-pip also supports
# workstations with python3-pip but without the optional ensurepip/venv wheel set.
/usr/bin/python3 -m venv --without-pip --system-site-packages "$COMPARISON_VENV"
"$COMPARISON_VENV/bin/python3" -m pip --require-virtualenv install -r "$COMPARISON_REQUIREMENTS"

# ROS setup scripts are not guaranteed to support bash nounset.
set +u
if [[ -f "$COMPARISON_ROOT/.amsim/local.env" ]]; then
  set -a
  source "$COMPARISON_ROOT/.amsim/local.env"
  set +a
fi
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
COMPARISON_ROS_WS="${AMSIM_ROS_WS:-$COMPARISON_ROOT/../ros2_px4_ws}"
if [[ -f "$COMPARISON_ROS_WS/install/setup.bash" ]]; then
  source "$COMPARISON_ROS_WS/install/setup.bash"
fi
set -u
export PYTHONNOUSERSITE=1

"$COMPARISON_VENV/bin/python3" - <<'PY'
from importlib import import_module, metadata
from pathlib import Path
import sys

import numpy as np
import scipy.sparse as sp
import osqp

prefix = Path(sys.prefix).resolve()
if sys.prefix == sys.base_prefix:
    raise RuntimeError("Comparison Python must be a virtual environment")
for name in ("numpy", "scipy", "osqp", "matplotlib", "pymavlink"):
    module = import_module(name)
    origin = Path(module.__file__).resolve()
    if not origin.is_relative_to(prefix):
        raise RuntimeError(f"{name} unexpectedly imported outside comparison venv: {origin}")
    print(f"{name} {metadata.version(name)}: {origin}")
for name in ("rclpy", "px4_msgs.msg", "gz.transport13", "gz.msgs10.entity_wrench_pb2"):
    module = import_module(name)
    print(f"{name}: {module.__file__}")
problem = osqp.OSQP()
problem.setup(P=sp.eye(1, format="csc"), q=np.array([-1.]),
              A=sp.eye(1, format="csc"), l=np.array([0.]), u=np.array([2.]),
              verbose=False, eps_abs=1e-7, eps_rel=1e-7)
result = problem.solve(raise_error=True)
if result.info.status != "solved" or abs(float(result.x[0])-1.) > 1e-5:
    raise RuntimeError(f"OSQP numerical smoke check failed: {result.info.status}")
print(f"Comparison environment ready: {sys.executable}")
PY
