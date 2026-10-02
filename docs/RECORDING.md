# Platform-managed recording

Custom scenarios do not need to launch rosbag or copy PX4 ULogs themselves.

For a user scenario executed with:

~~~bash
./scripts/amsim run my_test
~~~

the platform owns the recording lifecycle.

## YAML

~~~yaml
record:
  rosbag: true
  px4_ulog: true
  ground_truth: true
~~~

- `rosbag`: enable the platform ROS 2 bag recorder.
- `px4_ulog`: archive the active PX4 ULog after normal PX4 shutdown.
- `ground_truth`: include Gazebo ground truth in the rosbag.

## Standard rosbag topics

The recorder captures:

- arm joint states,
- all selected-model arm command topics,
- PX4 local position,
- PX4 attitude,
- PX4 odometry,
- PX4 status,
- PX4 control mode,
- PX4 land detection,
- simulation clock,
- ground truth when explicitly enabled.

Arm command topics are discovered dynamically from the selected model. They are
allowed to appear after recording starts because user scenario code creates the
command publishers at runtime.

## Standard run directory

A custom scenario receives `AMSIM_RUN_DIR` and the platform stores:

- `scenario.yaml`
- `experiment.py`
- `model.yaml` for generated configurable models
- `model_snapshot.sdf`
- `bag/`
- `recorder.log`
- `px4.ulg`
- `result.json`
- `run_manifest.json`
- any additional files written by the experiment into `AMSIM_RUN_DIR`

## Validation

The recorder was validated on the generated three-link model. The resulting bag
contained all three arm velocity command topics, arm joint states, PX4 state,
clock, and ground truth, and the run also finalized its PX4 ULog successfully.
