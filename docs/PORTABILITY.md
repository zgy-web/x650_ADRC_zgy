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
git clone <repository>
cd aerial_manipulator_sim
git switch simulation

./scripts/amsim setup --check-only
./scripts/amsim setup
./scripts/amsim doctor
./scripts/amsim run hover
~~~

The full bootstrap is intentionally not run on the golden development server
because its PX4 tree contains historical local/submodule state that must remain
untouched. It will be exercised on a clean second host.

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

The existing AutoDL server still selects VirtualGL and has been regression
tested after this abstraction.

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
