# Algorithm API V1

This document defines the intended stable surface. Raw backend topics may remain available for debugging but are not the compatibility contract.

## Vehicle observations

- `/amsim/vehicle/estimate/odometry` — state available to algorithms intended for real deployment.
- `/amsim/vehicle/ground_truth/odometry` — Gazebo truth for evaluation/analysis only.
- `/amsim/vehicle/status` — armed, mode, landed, health.

## Vehicle commands

High level:
- arm/disarm
- takeoff/land
- hover

Setpoint level:
- position
- velocity
- attitude
- body rate

Low-level actuator/thrust interfaces are optional advanced APIs and must be explicit.

## Manipulator observations

- `/amsim/arm/joint_states`
- future extension: joint effort/current and tool state

## Manipulator commands

Prefer standard ROS messages:
- position/trajectory
- velocity
- optional effort in future backends

## Combined observation

A synchronized observation API should combine base estimate, arm state, status, and timestamps so algorithms do not need to manually synchronize many raw topics.

## Sensors and task data

Reserved namespaces:
- `/amsim/camera/*`
- `/amsim/contact/*`
- `/amsim/task/*`

## Backend rule

Algorithm code must not depend on PX4 SITL process IDs, Gazebo-generated model instance names, or raw simulator transport topics. Backend adapters translate stable API calls to SITL/HIL implementations.
