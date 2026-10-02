# Aerial Manipulator Simulation Platform

Productization branch of the validated PX4 + Gazebo aerial-manipulator workflow.

## Goals

- One stable user-facing CLI and ROS API.
- Headless-by-default operation on Linux servers.
- Reproducible PX4/Gazebo/ROS2 environment.
- Separate ground-truth and PX4-estimated state.
- Config-driven vehicle, arm, scenario, and logging behavior.
- PX4 SITL first, with a backend boundary reserved for future HIL.
- Preserve the validated V3 workflow as an immutable reference while refactoring.

## Current milestone

P0/P1 bootstrap: frozen reference, reviewed model asset, environment doctor, and portable path resolution.

~~~bash
./scripts/amsim info
./scripts/amsim doctor
~~~

Do not edit `legacy/v3_reference` during productization.
