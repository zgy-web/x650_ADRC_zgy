# Safe Aerial Manipulator Testbed

A reusable PX4 SITL + Gazebo Harmonic + ROS 2 platform for aerial-manipulator simulation, visualization, control, and data recording.

> **Active simulation branch:** `simulation`
> The simulation platform is intentionally developed outside `main`.

<p align="center">
  <img src="docs/images/simulation_reviewed_underfold.png" width="48%" alt="Reviewed two-link aerial manipulator in Gazebo">
  <img src="docs/images/simulation_3link_overview.png" width="48%" alt="Configurable three-link aerial manipulator in Gazebo">
</p>

The current release has been clean-room validated on Ubuntu 22.04 with an NVIDIA RTX 3080 Ti. Validation covered PX4 SITL, Gazebo Sim 8.15, ROS 2 Humble, GPU rendering through VirtualGL, reviewed and generated manipulators, public Vehicle/Arm APIs, rosbag recording, and PX4 ULog archiving.

## What this repository provides

- PX4 SITL + Gazebo Harmonic runtime with a fixed PX4 commit.
- A reviewed two-link aerial-manipulator model.
- Configurable serial N-link manipulator generation.
- Stable public experiment APIs: `ScenarioContext`, `VehicleAPI`, `ArmAPI`, and observations.
- Separation between PX4-estimated state and optional Gazebo ground truth.
- Scenario discovery and user-defined scenario templates.
- Platform-managed rosbag + PX4 ULog recording.
- GPU Gazebo GUI on remote Linux servers.
- A clean-host installer for Ubuntu 22.04.
- A backend boundary reserved for future PX4 HIL support.

## Quick start

### 1. Clone the simulation branch

```bash
git clone -b simulation https://github.com/lilinanan/safe-aerial-manipulator-testbed.git
cd safe-aerial-manipulator-testbed
```

If the repository is private, authenticate to GitHub first, for example with `gh auth login`, or use your configured SSH credentials.

### 2. Install host dependencies

Validated target: **Ubuntu 22.04**.

```bash
sudo ./deploy/host/install_ubuntu22.sh
```

This installs ROS 2 Humble, Gazebo Harmonic / Gazebo Sim 8, ros_gz, rosbag2, build tools, Micro-XRCE-DDS Agent 2.4.2, and remote-visualization utilities.

The installer deliberately does **not** modify:

- NVIDIA driver
- firewall rules
- SSH configuration
- cloud-provider services

For GPU rendering, install a working NVIDIA driver before running the simulator. Verify with:

```bash
nvidia-smi
```

### 3. Prepare the project workspace

```bash
./scripts/amsim setup --check-only
./scripts/amsim setup
./scripts/amsim doctor
```

A healthy installation ends with:

```text
READY
```

The first full setup can take several minutes because PX4 SITL and the ROS message package are compiled.

### 4. Start visualization on a remote server

On a headless remote server:

```bash
./scripts/amsim desktop-start
./scripts/amsim desktop-status
./scripts/amsim render-status
```

A working NVIDIA path should report a VirtualGL renderer similar to:

```text
selected_backend: virtualgl
ready: true
gpu: NVIDIA ...
```

The VNC/noVNC listeners are localhost-only by design. Use an SSH or cloud-provider tunnel to view them; do not open the ports publicly.

If you already have a usable local X display, `desktop-start` is not required.

### 5. Run the validated examples

Stationary hover baseline:

```bash
./scripts/amsim run hover
```

Arm-motion experiment:

```bash
./scripts/amsim run arm_motion
```

Visualization is enabled by default for user-facing runs. Explicit batch mode:

```bash
./scripts/amsim run hover --headless
```

Each completed run is stored under `runs/` and can contain:

- scenario configuration snapshot
- model snapshot
- run manifest
- rosbag2 recording
- PX4 ULog
- recorder log
- scenario-specific result files

## Basic runtime commands

Start the reviewed two-link model:

```bash
./scripts/amsim start
./scripts/amsim check
./scripts/amsim gui-start
```

Inspect status:

```bash
./scripts/amsim status
./scripts/amsim gui-status
```

Stop cleanly:

```bash
./scripts/amsim gui-stop
./scripts/amsim stop
./scripts/amsim desktop-stop
```

## Configurable manipulators

List available models:

```bash
./scripts/amsim model-list
```

Build the supplied three-link example:

```bash
./scripts/amsim model-build x500_serial_3link_example
```

Start it:

```bash
./scripts/amsim start --model x500_serial_3link_example
./scripts/amsim gui-start
```

The runtime dynamically exposes all configured arm joints, for example:

```text
/arm/arm_joint_1/cmd_vel
/arm/arm_joint_2/cmd_vel
/arm/arm_joint_3/cmd_vel
/arm/joint_states
```

See `docs/CONFIGURABLE_MODELS.md`.

## Public experiment API

Experiments do not need to manipulate raw PX4/Gazebo topics directly.

```python
from amsim import ScenarioContext

with ScenarioContext() as ctx:
    ctx.vehicle.takeoff(2.0)

    ctx.arm.move_position(
        "arm_joint_1",
        -5.0,
        unit="deg",
        max_speed=1.0,
        speed_unit="deg/s",
    )

    obs = ctx.observe()
    print(obs.vehicle.z)
    print(obs.arm)

    ctx.vehicle.land()
```

By default, vehicle observations come from the PX4 estimate. Gazebo ground truth must be explicitly requested:

```python
obs = ctx.observe(ground_truth=True)
```

See `docs/PUBLIC_API.md`.

## Create a custom scenario

```bash
./scripts/amsim scenarios
./scripts/amsim new-scenario my_experiment
```

This creates a Python experiment and YAML configuration under `scenarios/`. Run it with:

```bash
./scripts/amsim run my_experiment
```

See `docs/CUSTOM_SCENARIOS.md` and `docs/RECORDING.md`.

## Validated software baseline

| Component | Validated value |
|---|---|
| OS | Ubuntu 22.04.3 LTS |
| Python | 3.10.x |
| ROS | ROS 2 Humble |
| Gazebo | Gazebo Sim 8.15.0 |
| ros_gz | 0.244.12-3jammy |
| rosbag2 | 0.15.17 |
| PX4 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Micro-XRCE-DDS Agent | 2.4.2 |
| Clean-room GPU | NVIDIA RTX 3080 Ti |
| Clean-room NVIDIA driver | 580.142 |

The exact validated `px4_msgs` interface snapshot is vendored in `vendor/px4_msgs`; it is intentionally not regenerated from the current PX4 `msg/` directory.

## Troubleshooting

Run these first:

```bash
./scripts/amsim setup --check-only
./scripts/amsim doctor
./scripts/amsim desktop-status
./scripts/amsim render-status
```

Typical issues:

**`doctor` reports missing ROS/Gazebo/MicroXRCEAgent**
Run:

```bash
sudo ./deploy/host/install_ubuntu22.sh
```

**GPU exists but rendering is not ready**
Check:

```bash
nvidia-smi
./scripts/amsim desktop-start
./scripts/amsim render-status
```

**Direct renderer says `llvmpipe`**
That is CPU software OpenGL. On a remote NVIDIA server, use the automatically selected VirtualGL backend when `virtualgl_ready: true`.

**A previous run was interrupted**
Clean owned processes with:

```bash
./scripts/amsim gui-stop
./scripts/amsim stop
./scripts/amsim desktop-stop
```

## Documentation

- [Detailed installation guide](docs/INSTALLATION.md)
- [Portability design](docs/PORTABILITY.md)
- [Clean-room validation report](docs/CLEAN_ROOM_VALIDATION_2026-10-02.md)
- [Public API](docs/PUBLIC_API.md)
- [Custom scenarios](docs/CUSTOM_SCENARIOS.md)
- [Recording](docs/RECORDING.md)
- [Configurable models](docs/CONFIGURABLE_MODELS.md)

## Current scope

Validated now:

- PX4 SITL
- Gazebo Harmonic
- ROS 2 Humble
- remote GPU visualization
- reviewed two-link manipulator
- configurable N-link manipulator
- public control/observation API
- scenario runner
- rosbag2 + PX4 ULog recording

Not yet claimed as validated:

- Docker packaging
- PX4 HIL
- physical UAV hardware

Do not edit `legacy/v3_reference`; it is preserved as the frozen historical reference workflow.
