# Flyarm flight-controller backend selector (v1)

This stage is **read-only observation selection**, not closed-loop HITL.

- `sitl`: subscribe to PX4 SITL ROS 2 topics `/fmu/out/vehicle_attitude` and `/fmu/out/sensor_combined`.
- `hardware`: read/replay the safely received Pixhawk COM3 CSV (PX4 v1.14.3) on AutoDL.
- Both modes emit a source-labeled `State` record, preserve source clock vs PC/server clock and reject `control_enabled` or `hil_injection_enabled` flags.
- There is **no actuator publication, flight arming, or sensor injection** in this selector.
- Existing `VehicleAPI` and `amsim.hil.bridge` are separate command-capable interfaces and must not be called from hardware observation mode.

Commands, from project root:

```bash
PYTHONPATH=src python3 -m amsim.hil.backend_selector --config configs/flight_controller_hardware.json --output /root/autodl-tmp/flyarm-sitl-check/hardware_backend_sample.json
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/ros2_px4_ws/install/setup.bash
PYTHONPATH=src python3 -m amsim.hil.backend_selector --config configs/flight_controller_sitl.json --duration 5
```

SITL mode requires active Gazebo/PX4 SITL + MicroXRCEAgent and ROS 2 setup; hardware mode currently uses archived CSV, **not a live bidirectional controller**. Later local-machine deployment can implement live hardware adapter behind the same State schema after independent acceptance tests.

Safety gating before true HITL: verify PX4 HITL capability/version, identify the flight-controller bootloader target, disable physical actuator outputs and disconnect propellers, verify frame and timestamp conventions, build a watchdog with stale-data cutoff, prove Gazebo sensor->Pixhawk->actuator feedback on a bench and measure end-to-end latencies. Do not bridge command topics across backend modes implicitly.

## v2: live hardware receive mode (read-only)

Start once on AutoDL (from project root):

```bash
PYTHONPATH=src python3 -m amsim.hil.backend_selector \\
 --config configs/flight_controller_hardware.json --hardware-live --duration 50 \\
 --output /root/autodl-tmp/flyarm-sitl-check/hardware_live_run.json
```

This binds only `127.0.0.1:18765` on AutoDL. From Windows open the SSH tunnel:

```powershell
ssh -N -T -o ExitOnForwardFailure=yes -L 127.0.0.1:18765:127.0.0.1:18765 -p 39852 root@connect.weste.seetacloud.com
```

Use the *existing* Windows `flyarm_pixhawk_forward.py` with QGroundControl closed; it warms up 5 s then forwards 30 s. Recheck current AutoDL public SSH port after instance restarts. The receiving process exits when Windows closes the connection, so restart it before each recording.

Only one receiver may bind port 18765; don't run the old `receive_pixhawk_tcp.py` concurrently.

Safety: live hardware mode is receive-only, not Gazebo-Pixhawk HITL, does not relay MAVLink actuator commands, and cannot infer one-way delay from unsynchronized clocks. The new selector is not automatically wired into the existing command-capable `VehicleAPI`.
