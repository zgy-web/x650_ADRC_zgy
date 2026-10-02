# Installation Guide

This guide describes the validated installation path for the `simulation`
branch of **safe-aerial-manipulator-testbed**.

## 1. Supported host

Validated target:

- Ubuntu 22.04 LTS, x86_64
- Python 3.10
- ROS 2 Humble
- Gazebo Harmonic / Gazebo Sim 8
- NVIDIA GPU + driver for hardware-accelerated remote visualization

A GPU is not required for headless batch use, but the product's visualization
capability is part of the validated workflow.

Before starting, verify the OS:

```bash
cat /etc/os-release
uname -m
```

For NVIDIA rendering:

```bash
nvidia-smi
```

Do not use the project installer to change an NVIDIA driver. Driver management
remains a host responsibility.

## 2. Clone the repository

```bash
git clone -b simulation https://github.com/lilinanan/safe-aerial-manipulator-testbed.git
cd safe-aerial-manipulator-testbed
```

Verify:

```bash
git branch --show-current
git status
```

Expected branch:

```text
simulation
```

If the repository is private, authenticate first. One supported workflow is:

```bash
gh auth login
gh auth setup-git
```

You can also use your existing Git HTTPS or SSH credential setup.

## 3. Install host-level dependencies

Run:

```bash
sudo ./deploy/host/install_ubuntu22.sh
```

The script installs:

- core compilation tools
- ROS 2 Humble
- rosbag2
- Gazebo Harmonic / Gazebo Sim 8
- ros_gzharmonic
- colcon and rosdep
- Micro-XRCE-DDS Agent 2.4.2
- Xvfb / x11vnc / noVNC utilities
- VirtualGL on NVIDIA hosts

It intentionally does not modify:

- NVIDIA driver
- firewall rules
- SSH configuration
- cloud-provider services

### Verify host dependencies

```bash
./scripts/amsim setup --check-only
```

A healthy host prints only PASS lines and finishes with:

```text
[PASS] host prerequisites
```

## 4. Build the project workspace

Run:

```bash
./scripts/amsim setup
```

The setup performs the project-specific work:

1. resolves the host-local workspace paths;
2. clones PX4 if it is missing;
3. checks out the validated PX4 commit;
4. initializes the required PX4 SITL submodules;
5. creates the AMSIM Python virtual environment;
6. installs PX4 Python build dependencies;
7. builds `px4_sitl_default`;
8. builds the vendored validated `px4_msgs` snapshot;
9. installs the reviewed aerial-manipulator model into the PX4 Gazebo model path;
10. writes local path configuration to ignored `.amsim/local.env`.

The first run can take several minutes. In particular, PX4 and `px4_msgs`
must be compiled.

The setup script refuses to change PX4 commits when it detects unexpected local
changes in the PX4 tree.

## 5. Verify the installation

Run:

```bash
./scripts/amsim doctor
```

Expected final line:

```text
READY
```

Then run the non-flight readiness test:

```bash
./scripts/amsim start
./scripts/amsim check
./scripts/amsim stop
```

The readiness test checks PX4 telemetry, arm joint state, the ROS-to-Gazebo
bridge, and reviewed model zero configuration without issuing arming or flight
commands.

## 6. Remote visualization

### Headless remote server

Create a local virtual display:

```bash
./scripts/amsim desktop-start
```

Check it:

```bash
./scripts/amsim desktop-status
./scripts/amsim render-status
```

On the validated NVIDIA path, `render-status` reports:

```text
selected_backend: virtualgl
ready: true
```

The stack uses:

- Xvfb display `:1`
- VNC on `127.0.0.1:5901`
- noVNC on `127.0.0.1:6080`

These listeners are intentionally localhost-only.

To view noVNC from your own computer, tunnel the port using SSH or your cloud
provider's port-forwarding feature. For a normal SSH setup:

```bash
ssh -L 6080:127.0.0.1:6080 <user>@<server>
```

Then open in your local browser:

```text
http://127.0.0.1:6080/vnc.html
```

Do not expose VNC/noVNC ports directly to the public internet.

### Local Linux workstation

If the workstation already has a usable X display, skip `desktop-start` and
use the existing display.

## 7. First visual smoke test

```bash
./tests/smoke/runtime_visualization_smoke.sh
```

This validates:

- project doctor
- X/display availability
- render backend
- runtime startup
- readiness
- GUI attachment
- GUI detachment without killing the runtime
- clean runtime shutdown

Expected final line:

```text
[PASS] runtime + readiness + visualization smoke test
```

## 8. Run the validated scenarios

### Hover

```bash
./scripts/amsim run hover
```

The validated hover scenario:

- starts PX4 SITL and Gazebo
- opens Gazebo GUI
- records baseline topics
- arms
- takes off to 2.5 m
- waits for stable hover
- records the stationary interval
- lands
- confirms landed state and automatic disarm
- archives rosbag and ULog

### Arm motion

```bash
./scripts/amsim run arm_motion
```

The reviewed motion performs the validated joint motion while airborne and
returns to the reviewed pose before landing.

### Explicit headless mode

```bash
./scripts/amsim run hover --headless
```

Use headless mode only when visualization is intentionally not required.

## 9. Result directory

Runs are written under:

```text
runs/
```

A formal recorded run can contain:

```text
scenario.yaml
model_snapshot.sdf
run_manifest.json
result.json
recorder.log
bag/
px4.ulg
<scenario-specific files>
```

The manifest records the platform/PX4 commits and Git dirty status so that an
experiment can be traced back to the exact code state.

## 10. Configurable N-link arm

List model configurations:

```bash
./scripts/amsim model-list
```

Build the supplied 3-link model:

```bash
./scripts/amsim model-build x500_serial_3link_example
```

Start it:

```bash
./scripts/amsim start --model x500_serial_3link_example
./scripts/amsim gui-start
```

The public API discovers the active runtime's joint list automatically.

## 11. Useful commands

```bash
./scripts/amsim info
./scripts/amsim doctor

./scripts/amsim start
./scripts/amsim check
./scripts/amsim status
./scripts/amsim stop

./scripts/amsim desktop-start
./scripts/amsim desktop-status
./scripts/amsim desktop-stop

./scripts/amsim render-status
./scripts/amsim gui-start
./scripts/amsim gui-status
./scripts/amsim gui-stop

./scripts/amsim scenarios
./scripts/amsim model-list
```

## 12. Troubleshooting

### GitHub clone/fetch is unreliable

Some cloud networks occasionally show TLS/HTTP2 connection failures. Retry with
HTTP/1.1:

```bash
git -c http.version=HTTP/1.1 fetch
```

The project bootstrap already uses HTTP/1.1 for the large PX4 clone/submodule
operations.

### `doctor` says MicroXRCEAgent is missing

Re-run:

```bash
sudo ./deploy/host/install_ubuntu22.sh
```

### `render-status` is not ready

Check:

```bash
nvidia-smi
./scripts/amsim desktop-start
./scripts/amsim desktop-status
./scripts/amsim render-status
```

If `direct_renderer` contains `llvmpipe`, direct rendering is using Mesa
software rendering. On a remote NVIDIA machine, prefer VirtualGL when
`virtualgl_ready` is true.

### Runtime is already running

Inspect:

```bash
./scripts/amsim status
./scripts/amsim gui-status
```

Then stop owned processes:

```bash
./scripts/amsim gui-stop
./scripts/amsim stop
```

### Clean everything after an interrupted interactive session

```bash
./scripts/amsim gui-stop || true
./scripts/amsim stop || true
./scripts/amsim desktop-stop || true
```

## 13. Validated vs. not yet validated

Validated on the second clean host:

- Ubuntu 22.04 clean-host deployment
- PX4 SITL
- Gazebo Sim 8.15
- ROS 2 Humble
- reviewed 2-link model
- generated 3-link model
- runtime/readiness checks
- VirtualGL NVIDIA rendering
- Xvfb + VNC + noVNC
- hover scenario
- arm-motion scenario
- public Vehicle API
- public Arm API
- rosbag2 recording
- PX4 ULog archival
- clean process shutdown

Not yet claimed as validated:

- NVIDIA GPU / VirtualGL GUI inside Docker (headless Docker is validated)
- physical PX4 HIL hardware (software HIL transport is validated)
- physical UAV hardware

See `CLEAN_ROOM_VALIDATION_2026-10-02.md` for the detailed acceptance record.
