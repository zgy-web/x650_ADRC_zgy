# Portability and clean-room deployment

## Target

A fresh Ubuntu 22.04 host should be able to clone the repository, satisfy the
documented host prerequisites, run the user-space setup, pass `amsim doctor`,
and execute a visualized scenario without editing source files or hard-coded
paths.

## Separation of responsibilities

Host layer:

- Ubuntu / kernel
- NVIDIA driver
- X display or remote desktop
- ROS 2 Humble system packages
- Gazebo Harmonic / Gazebo Sim 8
- ros_gz bridge
- Micro-XRCE-DDS Agent

Project layer:

- exact PX4 commit
- exact reviewed model snapshot
- exact px4_msgs snapshot
- Python runtime packages
- scenario/model configuration
- AMSIM runtime, public APIs, recording, and tests

This separation avoids baking a cloud-vendor NVIDIA driver into the project.

## Host workflow

~~~bash
git clone -b simulation https://github.com/lilinanan/safe-aerial-manipulator-testbed.git
cd safe-aerial-manipulator-testbed

sudo ./deploy/host/install_ubuntu22.sh
./scripts/amsim setup --check-only
./scripts/amsim setup
./scripts/amsim doctor
./scripts/amsim desktop-start   # remote/headless server
./scripts/amsim run hover
~~~

The full workflow was exercised on a clean second Ubuntu 22.04 host on
2026-10-02. The original golden development server remains preserved because
its historical PX4 tree must not be reset or cleaned.

## Path portability

Product code resolves the project/workspace from its own location and
`AMSIM_*` variables. The product source contains no required
`/root/autodl-tmp` path.

The setup writes host-local paths only to ignored
`.amsim/local.env`.

## px4_msgs

The exact 1.1 MB message package used by the validated platform is vendored at
`vendor/px4_msgs`. This avoids silently using a newer message schema on a
clean host.

## Visualization portability

`AMSIM_RENDER_BACKEND=auto` selects:

1. VirtualGL + NVIDIA EGL when that validated remote-server path is available.
2. direct X/OpenGL otherwise.

The clean-room server selected VirtualGL using its system NVIDIA EGL/GLVND
runtime (driver 580.142), confirming that the product does not require the
old AutoDL-specific copied EGL runtime.

## Clean-room acceptance

A second host is accepted only after:

1. `amsim setup --check-only` passes.
2. `amsim setup` completes.
3. `amsim doctor` reports READY.
4. runtime + visualization smoke passes.
5. `amsim run hover` succeeds with GUI, rosbag and ULog.
6. `amsim run arm_motion` succeeds.
7. configurable three-link model starts and exposes all joints.
8. public API flight smoke succeeds.
9. no PX4/Gazebo/Agent/GUI processes remain after shutdown.

Docker packaging remains a later layer; the current development host does not
have Docker installed, so Docker artifacts will not be presented as validated
until they are exercised on an appropriate host.

## Clean Ubuntu host installer

For an Ubuntu 22.04 host, the repository provides a host-layer installer:

~~~bash
sudo ./deploy/host/install_ubuntu22.sh
./scripts/amsim setup --check-only
./scripts/amsim setup
./scripts/amsim doctor
~~~

The installer does **not** modify the NVIDIA driver, firewall, SSH configuration,
or cloud-vendor services. It installs ROS 2 Humble, Gazebo Harmonic, ros_gz,
build tools, Micro-XRCE-DDS Agent 2.4.2, and optional remote-visualization tools.

For a headless remote server with an NVIDIA GPU, start the localhost-only
display/VNC layer with:

~~~bash
./scripts/amsim desktop-start
./scripts/amsim render-status
~~~

Then use `amsim gui-start` or any non-headless `amsim run ...` command.
The VNC/noVNC listeners bind to localhost only; expose them through an
authorized SSH/cloud tunnel rather than opening firewall ports.

The 2026-10-02 clean-room validation report is in
`docs/CLEAN_ROOM_VALIDATION_2026-10-02.md`.
