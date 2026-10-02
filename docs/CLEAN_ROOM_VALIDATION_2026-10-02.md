# Clean-room validation — 2026-10-02

Validated on a fresh Ubuntu 22.04.3 host with an NVIDIA GeForce RTX 3080 Ti
(driver 580.142). The source branch is `simulation`.

## Validated stack

- ROS 2 Humble
- Gazebo Harmonic / Gazebo Sim 8.15.0
- ros_gzharmonic 0.244.12-3jammy
- rosbag2 0.15.17
- Micro-XRCE-DDS Agent 2.4.2
- PX4 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- Python 3.10.12

## Acceptance results

- `amsim setup --check-only`: PASS after host dependencies were installed.
- `amsim doctor`: READY.
- Runtime/readiness smoke: PASS.
- GPU GUI: PASS using VirtualGL + system NVIDIA EGL.
- Localhost-only Xvfb + VNC + noVNC stack: PASS on ports 5901/6080.
- `amsim run hover`: PASS, including takeoff, stable hover, landing,
  automatic disarm, rosbag and ULog.
- `amsim run arm_motion`: PASS, including q1 0 -> -5 -> 0 deg,
  landing, rosbag and ULog.
- Generated 3-link model: PASS; all q1/q2/q3 command topics exposed.
- Public Arm API: PASS; q3 requested 10 deg and reached about 9.80 deg.
- Public Vehicle API: PASS; 2 m takeoff and landed/disarmed completion.
- GUI remained active during the 3-link public-API flight test.
- Owned PX4/Gazebo/Agent/GUI processes cleaned up after shutdown.

## Issues found and fixed

1. Recursive PX4 clone was too fragile and fetched unnecessary submodules.
   Bootstrap now initializes only the submodules required by the validated SITL build.
2. PX4 build requires its Python setup requirements before CMake configuration.
   Bootstrap now creates the project venv and installs those requirements first.
3. ROS interface generation could accidentally use the host Miniconda Python.
   Bootstrap now forces system ROS Python/colcon for the px4_msgs build.
4. The validated ROS px4_msgs schema is not identical to the pinned PX4 source
   message directory. The exact vendored snapshot is therefore retained.
5. VirtualGL no longer requires an AutoDL-specific copied EGL runtime when
   the host already provides a matching NVIDIA GLVND/EGL stack.
6. ScenarioContext now discovers joint names from the active runtime metadata,
   so direct public-API use works with configurable N-link models.
7. Run manifests record whether the platform and PX4 Git trees are dirty.

This clean-room report predates the Docker CI validation. Docker headless was
subsequently validated on 2026-10-02; see
`docs/DOCKER_VALIDATION_2026-10-02.md`.
