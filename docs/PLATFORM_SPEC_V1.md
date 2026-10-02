# Platform Specification V1

## Purpose

Provide a reusable, reproducible aerial-manipulator simulation platform for algorithm development before real-flight deployment.

## V1 scope

- PX4 SITL + Gazebo Harmonic + ROS 2 Humble.
- Reviewed two-link underfold manipulator model.
- Headless server operation with optional GUI attachment.
- Stable algorithm-facing API independent of Gazebo instance names.
- Separate PX4-estimated state from Gazebo ground truth.
- Standardized experiment lifecycle, logging, metadata, and health checks.
- Backend boundary that allows future replacement of PX4 SITL with a real PX4 flight controller in HIL.

## Non-goals for V1

- No claim of real-flight equivalence.
- No mandatory learning/RL stack.
- No direct dependency on research-only excitation/identification scripts.
- No requirement for a specific cloud vendor or GPU model.

## User workflow target

1. Install/build once.
2. Run `amsim doctor`.
3. Start a backend.
4. Run a named scenario from configuration.
5. Consume stable ROS topics/services from an algorithm.
6. Receive a self-contained run directory with configuration, logs, rosbag, PX4 ULog, model/build metadata, and result status.

## Portability contract

Runtime paths must be resolved from the project location or `AMSIM_*` environment variables. Product code must not assume `/root/autodl-tmp`, a particular hostname, or a particular NVIDIA driver version.

## Time contract

- SITL formal analysis uses Gazebo simulation time.
- Ground truth is evaluation-only by default.
- Algorithm examples use PX4 estimates unless explicitly declared otherwise.
- Future HIL mode will expose hardware timestamps through the same platform time abstraction.

## Frozen reference

`legacy/v3_reference` is a preserved copy of the last validated workflow and is not edited during productization.
