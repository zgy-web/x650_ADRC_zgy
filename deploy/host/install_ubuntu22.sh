#!/usr/bin/env bash
set -euo pipefail

# Installs host-level dependencies only. It deliberately does not install or
# modify the NVIDIA driver, firewall, SSH, or cloud-vendor services.
if [[ "$(id -u)" -ne 0 ]]; then
  echo "[FAIL] run this host installer as root (or with sudo)" >&2
  exit 2
fi

. /etc/os-release
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  echo "[FAIL] Ubuntu 22.04 required; got ${PRETTY_NAME:-unknown}" >&2
  exit 2
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y \
  build-essential ca-certificates cmake curl git gnupg lsb-release \
  locales ninja-build python3 python3-dev python3-pip python3-venv \
  software-properties-common

locale-gen en_US en_US.UTF-8 >/dev/null 2>&1 || true
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true
add-apt-repository -y universe

mkdir -p /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg
cat > /etc/apt/sources.list.d/ros2.list <<'EOF'
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
EOF

curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  | gpg --dearmor --yes -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
cat > /etc/apt/sources.list.d/gazebo-stable.list <<'EOF'
deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main
EOF

apt-get update
apt-get install -y \
  gz-harmonic \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros-base ros-humble-rosbag2 ros-humble-ros-gzharmonic \
  dbus-x11 iproute2 mesa-utils novnc websockify x11-xserver-utils \
  x11vnc xvfb
XRCE_VERSION=2.4.2
XRCE_ROOT=/opt/amsim/Micro-XRCE-DDS-Agent
if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  mkdir -p /opt/amsim
  rm -rf "$XRCE_ROOT"
  git clone --branch "v$XRCE_VERSION" --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$XRCE_ROOT"
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cmake -S "$XRCE_ROOT" -B "$XRCE_ROOT/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DUAGENT_USE_SYSTEM_FASTDDS=ON \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON \
    -DUAGENT_USE_SYSTEM_LOGGER=ON \
    -DUAGENT_P2P_PROFILE=OFF
  cmake --build "$XRCE_ROOT/build" -j"$(nproc)"
  cmake --install "$XRCE_ROOT/build"
  ldconfig
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  VGL_VERSION=3.1.4
  VGL_DEB="/tmp/virtualgl_${VGL_VERSION}_amd64.deb"
  if [[ ! -x /opt/VirtualGL/bin/vglrun ]]; then
    curl -fL --retry 3 \
      "https://github.com/VirtualGL/virtualgl/releases/download/${VGL_VERSION}/virtualgl_${VGL_VERSION}_amd64.deb" \
      -o "$VGL_DEB"
    dpkg -i "$VGL_DEB" || apt-get -f install -y
  fi
fi
echo "=== AMSIM host install verification ==="
/usr/bin/python3 --version
set +u
source /opt/ros/humble/setup.bash
set -u
ros2 --help >/dev/null
gz sim --versions | head -n1
command -v colcon
command -v MicroXRCEAgent
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -n1
fi
echo "[PASS] host dependencies installed"
