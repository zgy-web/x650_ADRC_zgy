# Configurable Manipulator Models

The platform supports two model classes:

- `reviewed`: frozen, hash-checked reference models used for regression and reproducible results.
- `generated`: YAML-defined serial manipulators built from a reviewed vehicle template.

The reviewed `x500_underfold` model is never overwritten by the generator.

## List models

~~~bash
./scripts/amsim model-list
~~~

## Build and validate a model

~~~bash
./scripts/amsim model-build x500_serial_3link_example
~~~

The generator validates the YAML, computes box-link inertia, emits SDF, and runs
Gazebo SDF validation.

## Select a model

Manual runtime:

~~~bash
./scripts/amsim start --model x500_serial_3link_example
~~~

Scenario configuration:

~~~yaml
model: x500_serial_3link_example
~~~

## Link count

`arm.links` is an ordered serial chain. Add or remove entries to change the
number of manipulator links. Each link gets its own revolute joint and velocity
command topic.

## Geometry

V1 supports box links:

~~~yaml
geometry:
  type: box
  length_m: 0.18
  width_m: 0.03
  height_m: 0.03
~~~

## Mass or density

Specify exactly one:

~~~yaml
mass:
  mass_kg: 0.10
~~~

or:

~~~yaml
mass:
  density_kg_m3: 617.28
~~~

For density mode the generator computes mass from box volume. For both modes it
computes the diagonal inertia tensor for a uniform box about its center of mass.

## Joint configuration

~~~yaml
joint:
  name: arm_joint_3
  initial_angle_deg: -30.0
  physical_limits_deg: [-90.0, 60.0]
  effort_limit: 6.0
  velocity_limit_rad_s: 2.0
  damping: 0.05
  friction: 0.01
  controller: velocity
~~~

The configured initial angle is baked into the initial geometry pose. Gazebo
joint coordinate zero therefore corresponds to that physical initial angle.
Physical limits are converted into simulator-relative limits automatically.

## Current validation status

The three-link example has been validated to:

- generate valid SDF,
- start through PX4 SITL,
- expose all three joints in `/arm/joint_states`,
- expose velocity command topics for all three joints,
- attach the GPU-rendered Gazebo GUI,
- accept and execute a command on `arm_joint_3`,
- shut down cleanly.

More complex geometry types, arbitrary joint axes per joint, payload/end-effector
blocks, sensors, and collision presets are planned extensions.
