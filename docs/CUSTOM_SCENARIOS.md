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

The YAML controls experiment configuration and whether visualization/data
recording are enabled. The Python file contains experiment-specific logic.

Run it with:

~~~bash
./scripts/amsim run my_test
~~~

The scenario name is discovered dynamically; it is not hard-coded into the CLI.

## Extension direction

The current user-scenario entry point is intentionally minimal while the
validated flight workflows remain unchanged. The next API layer will expose a
stable context object with Vehicle, Arm, Observation, timing, and logging APIs,
so user experiments can express logic without depending on raw PX4, Gazebo, or
transport topics.
