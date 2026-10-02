# PX4 HIL 使用与验证

本平台现在提供独立的 PX4 Hardware-in-the-Loop 软件链路。目标是让 Gazebo
继续承担动力学与传感器仿真，而真实 PX4 飞控运行姿态估计、控制器和混控。

## 数据路径

```text
Gazebo IMU / Magnetometer / Barometer / GPS
                  │
                  ▼
          amsim.hil.bridge
                  │ MAVLink
                  ▼
           真实 PX4 飞控
                  │
          HIL_ACTUATOR_CONTROLS
                  ▼
          amsim.hil.bridge
                  │
                  ▼
        Gazebo motor_speed
```

机械臂仍通过 ROS 2 ↔ Gazebo bridge 控制。

如果还希望在 HIL 模式下继续使用现有 `ScenarioContext.vehicle` /
PX4 ROS 2 观测接口，需要给飞控额外提供 Micro-XRCE-DDS 通道。推荐使用第二条
UART/USB 串口，避免和 MAVLink HIL 共用同一个串口设备。

## 飞控侧要求

真实飞控应使用与项目兼容的 PX4 固件，并启用：

```text
SYS_HITL = 1
```

修改后必须重启飞控。

PX4 自带 HIL airframe `1001_rc_quad_x.hil` 会设置 `SYS_HITL=1`。
实际使用前应根据目标机架确认 airframe、输出映射和飞控板型号。

HIL 测试期间不要连接真实电机/桨叶，或确保硬件输出处于物理安全状态。
本项目的 HIL bridge 使用 `HIL_ACTUATOR_CONTROLS` 驱动 Gazebo 电机，不需要
真实旋翼产生推力。

## 1. 找到 MAVLink 串口

例如：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

典型 USB 飞控可能是：

```text
/dev/ttyACM0
```

## 2. 只读检查

不会发送解锁或飞行动作：

```bash
./scripts/amsim hil-doctor \
  --mavlink /dev/ttyACM0 \
  --baud 921600
```

要求 HIL flag 已经打开：

```bash
./scripts/amsim hil-doctor \
  --mavlink /dev/ttyACM0 \
  --baud 921600 \
  --require-hil
```

成功结果会显示：

```text
autopilot_px4: true
hil_enabled: true
```

## 3. 启动 HIL

只使用 MAVLink HIL：

```bash
./scripts/amsim hil-start \
  --mavlink /dev/ttyACM0 \
  --baud 921600
```

检查：

```bash
./scripts/amsim hil-status
```

停止：

```bash
./scripts/amsim hil-stop
```

## 4. 带 ROS 2 PX4 接口的 HIL

如果第二条串口连接到飞控的 Micro-XRCE-DDS client：

```bash
./scripts/amsim hil-start \
  --mavlink /dev/ttyACM0 \
  --baud 921600 \
  --xrce-serial /dev/ttyUSB0 \
  --xrce-baud 921600
```

这时 supervisor 会同时启动：

- Gazebo physics/sensors
- MAVLink HIL bridge
- ROS ↔ Gazebo arm/truth bridge
- Micro-XRCE-DDS Agent

从而为后续复用现有 Vehicle/Observation API 提供接口基础。

## 5. 软件 HIL 回环验证

仓库提供一个不需要真实飞控的 transport smoke test：

```bash
python3 tests/smoke/hil_software_smoke.py
```

它使用一个假的 MAVLink PX4 endpoint，实际验证：

- Gazebo IMU → `HIL_SENSOR`
- Gazebo NavSat → `HIL_GPS`
- `HIL_ACTUATOR_CONTROLS` → Gazebo `motor_speed`

2026-10-02 在 clean-room 服务器上的一次结果：

```text
HIL_SENSOR: 259
HIL_GPS: 11
HIL_ACTUATOR_CONTROLS received by bridge: 49
PASS
```

这证明软件 HIL 数据通路闭环工作，但**不能替代真实飞控验证**。

## 当前验证边界

已验证：

- HIL Gazebo world 启动
- 传感器到 MAVLink HIL 数据转换
- HIL actuator 到 Gazebo motor command
- HIL runtime start/status/stop
- 软件 fake-PX4 回环

尚需真实硬件完成：

- 实体 PX4 heartbeat / HIL flag
- 实体 PX4 接收 HIL_SENSOR/HIL_GPS
- 实体 PX4 输出 HIL_ACTUATOR_CONTROLS
- 如使用 ROS API：第二条 XRCE 链路
- 实体飞控下起飞、悬停、降落闭环

当前 clean-room 服务器未发现 `/dev/ttyACM*` 或 `/dev/ttyUSB*`，因此不能在
没有物理飞控连接的情况下把“真实 PX4 HIL”标记为已验证。
