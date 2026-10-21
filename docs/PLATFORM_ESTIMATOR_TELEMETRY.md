# Optional PX4 estimated-state telemetry

`ApiNode.enable_estimator_feedback()` adds ROS `/clock`, PX4 attitude, angular
velocity and estimator status flags to the existing API subscriptions. Calling
it twice does not create duplicate subscriptions. Default consumers retain their
original subscriptions. Each callback records a sequence number, monotonic wall
receipt time and the last received `/clock`; these are reception diagnostics,
not proof of synchronized physical sampling. A clock regression latches an error.

The scripts below target existing PX4 source commit
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`; they do not download software, build it,
or start a flight. Omit `--apply` to inspect changes first. Applied edits require
stopped PX4 and no pending parameter restoration, and save exact before/after
source files in a new output directory.

```bash
python scripts/configure_px4_estimator_telemetry.py --px4 /path/to/PX4-Autopilot --output /path/to/new/dds --apply
python scripts/configure_px4_upper_logging.py --px4 /path/to/PX4-Autopilot --output /path/to/new/logging --apply
ninja -C /path/to/PX4-Autopilot/build/px4_sitl_default -j4 px4
```

The DDS script enables angular velocity and limits it and attitude to 50 Hz.
Reinstall the selected custom airframe after rebuilding. The logging script
preserves the complete default topic list and removes downsampling for received
trajectory setpoints, local position, internal local-position setpoints and
attitude. A custom `logger_topics.txt` overrides that default profile; inspect
the actual ULog rather than assuming every update was recorded.

With an existing disarmed, landed runtime and the same ROS environment loaded:

```bash
PYTHONPATH=src:$PYTHONPATH python scripts/probe_px4_estimator_telemetry.py \
  --duration-wall-s 45 --output /path/to/new/telemetry_probe.json
```

This probe does not start/stop the runtime, arm, modify parameters or publish
commands. Its output is a diagnostic, not a flight acceptance certificate.
Runtime process readiness and Gazebo bridge readiness do not guarantee that
PX4 DDS discovery has finished. In the 2026-10-21 ground check the PX4 streams
appeared approximately 19 wall seconds after the first Gazebo clock sample;
the 45-second check observed all required streams and preserved disarmed/landed
state. The earlier 10-second check was incomplete and remains a failed diagnostic.

DDS rate limits are upper bounds. In this PX4 version `EKF2::PublishStatusFlags`
publishes about once per simulation second, or immediately when a flag changes.
Consumers must distinguish that heartbeat from continuous pose/rate streams.
They must also enforce frame, validity and reset guards before using estimates
for control; this platform extension alone does not implement those guards.

Portable regression tests require no ROS runtime or private geometry:

```bash
PYTHONPATH=src python -m unittest discover -s tests/unit -p 'test_api_estimator_telemetry.py'
PYTHONPATH=src python -m unittest discover -s tests/unit -p 'test_px4_estimator_telemetry.py'
PYTHONPATH=src python -m unittest discover -s tests/unit -p 'test_px4_upper_logging.py'
```
