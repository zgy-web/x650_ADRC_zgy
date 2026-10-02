# PX4 HIL Plan

V1 ships with PX4 SITL. The architecture reserves an autopilot backend boundary so a real PX4 flight controller can later replace the SITL process without changing algorithm-facing APIs.

Future HIL loop:

1. Gazebo produces virtual IMU/GNSS/barometer/magnetometer data.
2. Sensor data is delivered to the real PX4 flight controller using a supported HIL transport.
3. Real PX4 computes actuator outputs.
4. Actuator outputs are fed back into Gazebo vehicle dynamics.
5. ROS-facing vehicle/arm APIs remain unchanged above the backend adapter.

HIL implementation is intentionally deferred until SITL productization and API regression tests are stable.
