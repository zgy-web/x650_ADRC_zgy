# Public Scenario API

The public API is the experiment-facing layer. User experiments should depend
on this layer instead of raw PX4 MAVLink, Gazebo transport topics, generated
model instance names, or simulator process details.

## Scenario context

~~~python
from amsim import ScenarioContext

with ScenarioContext() as ctx:
    ...
~~~

The context exposes:

- `ctx.vehicle` — vehicle operations backed by PX4.
- `ctx.arm` — manipulator operations backed by the stable ROS arm namespace.
- `ctx.observe()` — synchronized snapshot of vehicle estimate and arm state.
- `ctx.wait(sim_seconds)` — simulation-time wait.

## Vehicle API

~~~python
ctx.vehicle.wait_ready()
ctx.vehicle.takeoff(2.5)
ctx.vehicle.land()
~~~

Current V1 methods:

- `wait_ready(timeout_s=60)`
- `arm()`
- `takeoff(height_m=2.5, stable_s=3.0)`
- `land()`

PX4-estimated state is used for control-facing vehicle observations. Gazebo
ground truth is not silently substituted.

## Arm API

The API discovers joints from the selected model, so it is not limited to two
links.

~~~python
print(ctx.arm.joints)

ctx.arm.set_velocity(
    "arm_joint_3",
    5.0,
    unit="deg/s",
)

ctx.arm.move_position(
    "arm_joint_3",
    10.0,
    unit="deg",
    max_speed=8.0,
    speed_unit="deg/s",
)
~~~

Current V1 methods:

- `state()`
- `set_velocity(joint, value, unit="rad/s")`
- `move_position(...)`
- `stop(joint=None)`

The current position API closes a simple position loop over the simulator's
velocity-controlled joints. Later backends can replace this implementation
without changing experiment code.

## Observation API

~~~python
obs = ctx.observe()

print(obs.vehicle.x)
print(obs.vehicle.z)
print(obs.arm["arm_joint_1"].position_rad)
~~~

Ground truth is opt-in:

~~~python
obs = ctx.observe(ground_truth=True)
print(obs.ground_truth)
~~~

This explicit switch is intentional. Algorithms intended for real deployment
should not accidentally depend on Gazebo truth.

## Validated smoke tests

The public API has been exercised on the generated three-link model:

- joint discovery returned all three joints,
- `arm_joint_3` was commanded through `ArmAPI.move_position`,
- requested 10 deg and reached approximately 9.8 deg,
- vehicle estimate and opt-in ground truth were both readable,
- `VehicleAPI.takeoff(2.0)` reached airborne state,
- `VehicleAPI.land()` returned landed and automatically disarmed state,
- the flight API run was performed while the GPU-rendered Gazebo GUI was active,
- runtime and GUI shut down without residual simulator processes.
