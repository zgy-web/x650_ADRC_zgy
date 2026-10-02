# Custom Scenarios

The platform core owns PX4/Gazebo startup, readiness checks, visualization,
process cleanup, result metadata, and PX4 ULog archival.

Experiment authors should change scenario configuration or scenario code, not
the runtime core.

## Discover scenarios

~~~bash
./scripts/amsim scenarios
~~~

Built-in examples currently include `hover` and `arm_motion`.

## Create a new experiment

~~~bash
./scripts/amsim new-scenario my_test
~~~

This creates:

- `scenarios/my_test.py`
- `scenarios/my_test.yaml`

The YAML selects model/runtime options. The Python file contains experiment
logic using the public API.

A generated template looks like:

~~~python
from amsim import ScenarioContext

def main():
    with ScenarioContext() as ctx:
        ctx.vehicle.takeoff(2.5)

        # Optional manipulator motion:
        # ctx.arm.move_position(
        #     "arm_joint_1",
        #     -5.0,
        #     unit="deg",
        #     max_speed=0.5,
        #     speed_unit="deg/s",
        # )

        obs = ctx.observe()
        print(obs)

        ctx.wait(2.0)
        ctx.vehicle.land()

if __name__ == "__main__":
    main()
~~~

Run it with:

~~~bash
./scripts/amsim run my_test
~~~

The scenario name is discovered dynamically; it is not hard-coded into the CLI.

## Select a manipulator model

The scenario YAML can select either the reviewed reference model or a generated
configurable model:

~~~yaml
model: x500_underfold
~~~

or:

~~~yaml
model: x500_serial_3link_example
~~~

The selected model determines the discovered joint list exposed through
`ctx.arm.joints`.

## Ground truth

Ground truth is evaluation-only and must be requested explicitly:

~~~python
obs = ctx.observe(ground_truth=True)
~~~

Normal `ctx.observe()` returns PX4-estimated vehicle state plus arm state
without exposing Gazebo ground truth.
