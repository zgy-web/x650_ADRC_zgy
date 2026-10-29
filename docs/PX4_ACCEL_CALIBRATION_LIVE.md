# Pixhawk 六面加速度计校准：本机实时引导

本工具让操作者根据 FMU Debug 实际输出连续完成六面校准，避免在每个方向之间等待聊天回复。它运行在电脑上，不访问 NX。校准由 PX4 自身执行并保存；本工具只发送一次 `commander calibrate accel`，不自动重试。

## 使用范围

- 当前适配：Pixhawk 6X Pro、固件 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- 接口：FMU Debug，默认 `COM13`，57600 baud；串口设置沿用 `px4_debug_console.py`。
- 现场条件：全部拆桨、电调动力关闭、飞控低压供电；机架可稳定保持六个不同方向。
- 当前台架额外核对：未解锁、停机标志有效、DShot 输出全零、动力电压低于 1 V，`MOT_POLE_COUNT=24`。
- 不同时运行其他占用 Debug 串口的程序。

## 操作

在仓库根目录运行，`--output` 必须是尚不存在的目录：

```powershell
python -X utf8 -u scripts/px4_accel_calibration_live.py --esc-power-off --operator-ready --output artifacts/px4_accel_cal_YYYYMMDD/live_v1
```

1. 打开终端打印的本机网址，默认 `http://127.0.0.1:8766/`。
2. 机架正放静止，确认网页显示“实时连接正常”，点击一次“开始六面校准”。
3. 程序先读取飞控状态、备份校准参数并重新检查，再发送唯一一次校准命令。
4. 黄色表示飞控正在采集该面，保持静止；绿色表示实际收到该面完成消息，可以换面。无需逐面返回聊天。
5. 推荐顺序为正放、左侧朝下、倒置、右侧朝下、机头朝下、机头朝上；飞控也允许其他顺序，页面按实际识别结果更新。方向以飞控机体坐标为准。变绿后开始平稳换面，不要继续静止在刚完成的面上；当前固件重复识别已完成的面会累计失败计数，最终退出。
6. 六面完成后恢复正放、静置。等待参数回读完成，再进行独立的静置 IMU 复测。

轻微姿态偏差允许存在，但六个方向应明显不同，采样期间必须静止。机架支撑应牢固，翻转时不要拉扯 USB 和 Debug 接线。加速度计六面校准与水平姿态校准是不同操作，本工具不会改写板载安装角以“归零”台面误差。

## 停止与失败处理

关闭网页不会停止已经在飞控运行的校准。若现场需要立即中止，应保持电调动力关闭，断开飞控低压供电；随后确认飞控重启状态，再决定是否重新开始。不要以刷新网页或重复点击代替中止操作。

重复点击不能再次发送校准命令。通信中断、命令是否执行不确定、观察超时或缺少六面完成记录时，程序不会自动重发。记录为未确认成功，不推定原参数未改变。完成、失败或取消的明确消息收到后，程序尝试回读参数；否则只保存现有证据并关闭自己的串口。

## 结果与验收

- `status.json`：实时进度、开始前后的状态及参数、实际修改项、完成标志。
- `calibration_stream.bin`：校准期间收到的原始串口字节。
- `console.json`：检查和回读命令的响应，用于复核。
- `server.json`：本地页面地址和串口名称。

`calibration_success=true` 只表示收到六个不同面的完成消息及 PX4 校准成功消息。还须分别核对 `post_readback_complete`、新的校准参数是否合理、静置后的三套 IMU/估计器偏置，以及后续断电重启的保存结果。该字段不代表已消除预检警告或可飞行。

验收复测可以使用已有的只读工具，必须先确认校准线程结束、串口关闭，并把机架正放静置：

```powershell
python -X utf8 scripts/px4_imu_pole_check.py --esc-power-off --rounds 13 --output artifacts/px4_accel_cal_YYYYMMDD/post_stationary_v1
```

## 实现对应

| 文件 | 作用 |
| --- | --- |
| `scripts/px4_accel_calibration_live.py` | 单次启动、只读前置检查、校准消息解析、结果回读、本地 HTTP 服务 |
| `scripts/px4_accel_calibration_live.html` | 六面中文提示及实时进度；不从页面开放任意串口命令 |
| `tests/unit/test_px4_accel_calibration_live.py` | 六面完整性、重复消息、失败状态、单次发送和错误固件拒绝 |
| `scripts/px4_imu_pole_check.py` | 已有状态检查和校准后静置观测 |

HTTP 服务仅监听 `127.0.0.1`，开始请求核对来源和单次会话令牌。页面应在校准前打开并保持连接；后台程序不会自行开始。程序不修改 NX 配置，也不发送解锁、Offboard、执行器或电机命令。

参考：[PX4 加速度计校准说明](https://docs.px4.io/v1.17/en/config/accelerometer)、[当前固件六面校准实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/accelerometer_calibration.cpp)、[当前固件姿态识别](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/calibration_routines.cpp)。
