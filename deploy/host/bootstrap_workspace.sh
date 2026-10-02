#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORKSPACE="${AMSIM_WORKSPACE:-$(cd "$ROOT/.." && pwd)}"
PX4_DIR="${AMSIM_PX4_DIR:-$WORKSPACE/PX4-Autopilot}"
ROS_WS="${AMSIM_ROS_WS:-$WORKSPACE/ros2_px4_ws}"
VENV="${AMSIM_VENV:-$WORKSPACE/amsim-venv}"
PX4_COMMIT="d6f12ad1c4f70ad3230afd7d86e971421e02fef4"

CHECK_ONLY=0
SKIP_PX4_BUILD=0

for arg in "$@"; do
  case "$arg" in
    --check-only) CHECK_ONLY=1 ;;
    --skip-px4-build) SKIP_PX4_BUILD=1 ;;
    *) echo "Unknown option: $arg" >&2; exit 2 ;;
  esac
done

"$ROOT/deploy/host/check_system.sh"
[[ "$CHECK_ONLY" -eq 1 ]] && { echo "[PASS] host prerequisites"; exit 0; }

echo "=== AMSIM user-space bootstrap ==="
echo "project   : $ROOT"
echo "workspace : $WORKSPACE"
echo "PX4       : $PX4_DIR"
echo "ROS ws    : $ROS_WS"
echo "venv      : $VENV"

if [[ ! -d "$PX4_DIR/.git" ]]; then
  git clone --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi

actual="$(git -C "$PX4_DIR" rev-parse HEAD)"
if [[ "$actual" != "$PX4_COMMIT" ]]; then
  if [[ -n "$(git -C "$PX4_DIR" status --porcelain)" ]]; then
    echo "[FAIL] PX4 checkout has local changes; refusing checkout" >&2
    exit 2
  fi
  git -C "$PX4_DIR" fetch --all --tags
  git -C "$PX4_DIR" checkout "$PX4_COMMIT"
fi

git -C "$PX4_DIR" submodule update --init --recursive

if [[ "$SKIP_PX4_BUILD" -eq 0 && ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  cmake -S "$PX4_DIR" -B "$PX4_DIR/build/px4_sitl_default" -GNinja -DCONFIG=px4_sitl_default
  cmake --build "$PX4_DIR/build/px4_sitl_default"
fi

rm -rf "$ROS_WS/src/px4_msgs"
mkdir -p "$ROS_WS/src/px4_msgs/msg"
cp "$ROOT/vendor/px4_msgs/CMakeLists.txt" "$ROS_WS/src/px4_msgs/CMakeLists.txt"
cp "$ROOT/vendor/px4_msgs/package.xml" "$ROS_WS/src/px4_msgs/package.xml"
cp "$PX4_DIR"/msg/*.msg "$ROS_WS/src/px4_msgs/msg/"

set +u
source /opt/ros/humble/setup.bash
set -u
(
  cd "$ROS_WS"
  colcon build --symlink-install --packages-select px4_msgs
)

if [[ ! -x "$VENV/bin/python3" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/python3" -m pip install -r "$ROOT/deploy/host/runtime-requirements.txt"

MODEL_DST="$PX4_DIR/Tools/simulation/gz/models/x500_underfold"
mkdir -p "$(dirname "$MODEL_DST")"
rm -rf "$MODEL_DST"
cp -a "$ROOT/assets/models/x500_underfold" "$MODEL_DST"

mkdir -p "$ROOT/.amsim"
cat > "$ROOT/.amsim/local.env" <<EOF
AMSIM_WORKSPACE=$WORKSPACE
AMSIM_PX4_DIR=$PX4_DIR
AMSIM_ROS_WS=$ROS_WS
AMSIM_VENV=$VENV
EOF

echo "[PASS] user-space workspace prepared"
