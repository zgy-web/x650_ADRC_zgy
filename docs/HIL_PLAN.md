# PX4 HIL 状态

HIL 已不再只是规划项。当前 `simulation` 分支已经实现了 Gazebo ↔ MAVLink
HIL 软件桥接路径，并提供 `amsim hil-*` CLI。

当前数据链路：

1. Gazebo 产生虚拟 IMU / GNSS / barometer / magnetometer 数据。
2. `amsim.hil.bridge` 将其转换为 MAVLink `HIL_SENSOR` / `HIL_GPS`。
3. PX4 HIL 飞控计算控制输出。
4. `HIL_ACTUATOR_CONTROLS` 被转换为 Gazebo `motor_speed`。
5. 机械臂仍由 ROS 2 ↔ Gazebo bridge 控制。
6. 如果提供额外 Micro-XRCE-DDS 串口，可继续暴露 PX4 ROS 2 接口，为复用
   现有 Vehicle/Observation API 提供基础。

软件 fake-PX4 回环测试已通过。真实飞控验证仍要求物理 PX4 通过串口/USB
连接到运行主机。

完整使用说明见：

- `docs/HIL.md`
