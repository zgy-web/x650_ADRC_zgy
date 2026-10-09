# Flyarm HITL precheck — not an authorization to operate hardware

The current active `hardware` backend is **read-only**. PX4 v1.14.3 on the physical Pixhawk has not yet been verified in HITL mode. No Gazebo sensor injection or motor/actuator feedback to the real flight controller has been enabled.

The existing `amsim.hil.bridge` is a separate **command-capable** path: sends `HIL_SENSOR` and `HIL_GPS` to PX4, receives `HIL_ACTUATOR_CONTROLS` and publishes Gazebo motor velocity. Its runtime is not yet integrated with the new safety gate. Do not connect its MAVLink endpoint to the real Pixhawk.

`amsim.hil.safety_gate.check_hitl_gate` is a *pure preflight decision helper* with conservative explicit predicates. It is not a runtime watchdog, and passing the function in a synthetic unit test does not authorize hardware operation. Before physical HITL, integrate fail-closed monitoring and safe output fallback into the real bridge, verify physical actuators disconnected, prove firmware HITL support and the HIL heartbeat flag, verify timestamps and coordinate frames, and test loss-of-link behavior on an isolated bench.

Future adapter design: `sitl` obtains state via PX4 ROS 2; `hardware` obtains state via serial/MAVLink local receiver; a separately enabled `hitl` action channel consumes Gazebo sensor messages, sends HITL sensor messages to the verified PX4, and routes received actuator messages **only** to Gazebo. No automatic switching of actuator output is permitted. Public internet SSH telemetry path is observation-only and is not a deterministic control loop.
