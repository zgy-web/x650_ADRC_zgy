#!/usr/bin/env bash
set -o pipefail

cd "$(dirname "$0")"

# ROS setup scripts are not nounset-safe.
source ./cloud_env.sh

DISPLAY_NUM=":1"
XAUTH="/root/.Xauthority"

EGL_DIR="/root/autodl-tmp/egl-test"
EGL_LIB_DIR="${EGL_DIR}/lib"
EGL_JSON="${EGL_DIR}/10_nvidia.json"

VGLRUN="/opt/VirtualGL/bin/vglrun"

PX4_DIR="/root/autodl-tmp/PX4-Autopilot"

LOG_DIR="/root/autodl-tmp/desktop_logs"
mkdir -p "$LOG_DIR"

echo "============================================================"
echo " UNDERFOLD V3 - ATTACH GAZEBO GUI"
echo "============================================================"
echo "This GUI is observation only."
echo "It does NOT start PX4 and does NOT send flight commands."
echo

# ------------------------------------------------------------
# 1. X desktop
# ------------------------------------------------------------

if ! DISPLAY="$DISPLAY_NUM" \
     XAUTHORITY="$XAUTH" \
     xset q >/dev/null 2>&1; then

    echo "[FAIL] DISPLAY=:1 is not available."
    echo "Restore the VNC/X desktop first."
    exit 1
fi

echo "[ OK ] DISPLAY=:1"


# ------------------------------------------------------------
# 2. Wake / inhibit screensaver
# ------------------------------------------------------------

WAKE="/root/autodl-tmp/underfold_v1/wake_desktop.sh"
INHIBIT="/root/autodl-tmp/underfold_v1/screen_inhibit_on.sh"

if [[ -x "$WAKE" ]]; then
    "$WAKE" || true
fi

if [[ -x "$INHIBIT" ]]; then
    "$INHIBIT" || true
else
    echo "[WARN] screen_inhibit_on.sh not found;"
    echo "       desktop may blank again after idle timeout."
fi


# ------------------------------------------------------------
# 3. Verify GPU runtime matches current driver
# ------------------------------------------------------------

DRIVER="$(
    nvidia-smi \
      --query-gpu=driver_version \
      --format=csv,noheader \
      | head -n1
)"

GPU="$(
    nvidia-smi \
      --query-gpu=name \
      --format=csv,noheader \
      | head -n1
)"

GPUCOMP="${EGL_LIB_DIR}/libnvidia-gpucomp.so.${DRIVER}"

echo
echo "[INFO] GPU    : $GPU"
echo "[INFO] Driver : $DRIVER"

if [[ ! -f "$GPUCOMP" ]]; then
    echo "[FAIL] Missing matching gpucomp:"
    echo "$GPUCOMP"
    exit 1
fi

if [[ ! -f "$EGL_JSON" ]]; then
    echo "[FAIL] Missing EGL JSON:"
    echo "$EGL_JSON"
    exit 1
fi

echo "[ OK ] NVIDIA GUI runtime"


# ------------------------------------------------------------
# 4. Verify the cloud Gazebo server is really alive
# ------------------------------------------------------------

echo
echo "Checking Gazebo server on partition:"
echo "  $GZ_PARTITION"

TOPICS="$(
    timeout 8 gz topic -l 2>/dev/null || true
)"

if ! grep -q '^/world/default/' <<< "$TOPICS"; then
    echo
    echo "[FAIL] No /world/default Gazebo server found"
    echo "       on GZ_PARTITION=$GZ_PARTITION."
    echo
    echo "Start v3 simulation first:"
    echo "  bash start_cloud.sh"
    exit 1
fi

if ! grep -q 'x500_underfold' <<< "$TOPICS"; then
    echo
    echo "[WARN] Gazebo server exists but no underfold"
    echo "       topic was detected yet."
else
    echo "[ OK ] underfold Gazebo server detected"
fi


# ------------------------------------------------------------
# 5. Avoid duplicate GUI
# ------------------------------------------------------------

if pgrep -af \
  'gz sim.*-g|gz sim.*--gui|gz sim gui|gz-sim-gui' \
  >/tmp/underfold_gui_existing.txt 2>/dev/null; then

    echo
    echo "[FAIL] A Gazebo GUI appears to be already running:"
    cat /tmp/underfold_gui_existing.txt
    echo
    echo "Not starting a duplicate GUI."
    exit 1
fi


# ------------------------------------------------------------
# 6. GUI resources
# ------------------------------------------------------------

RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
    RESOURCE_PATH="${RESOURCE_PATH}:${GZ_SIM_RESOURCE_PATH}"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$LOG_DIR/gazebo-cloud-gui-${STAMP}.log"

echo
echo "============================================================"
echo " Starting GUI-only Gazebo"
echo "============================================================"
echo "Partition : $GZ_PARTITION"
echo "Log       : $LOG"
echo
echo "Close ONLY the GUI with Ctrl+C in this terminal."
echo "The PX4/Gazebo server runs independently."
echo "============================================================"
echo


# HEADLESS belongs to the PX4 simulation startup path.
# The GUI process must not inherit it.

env \
  -u HEADLESS \
  -u LIBGL_ALWAYS_SOFTWARE \
  -u SESSION_MANAGER \
  -u DBUS_SESSION_BUS_ADDRESS \
  DISPLAY="$DISPLAY_NUM" \
  XAUTHORITY="$XAUTH" \
  GZ_PARTITION="$GZ_PARTITION" \
  GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" \
  LD_LIBRARY_PATH="${EGL_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
  __EGL_VENDOR_LIBRARY_FILENAMES="$EGL_JSON" \
  dbus-run-session -- \
  "$VGLRUN" -d egl0 \
  gz sim --force-version 8 -g \
  2>&1 | tee "$LOG"
