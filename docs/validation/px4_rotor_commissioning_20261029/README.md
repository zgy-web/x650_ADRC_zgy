# 四旋翼真实台架：通电前读取

## 已完成及尚未完成

2026-10-29，用户重新接好 FMU Debug 后，经 COM13、57600 baud 读取真实 Pixhawk 6X Pro。完整的最新只读结果为 [inventory_readonly.json](inventory_readonly.json)。读取开始时间为 UTC 05:08:43.778（北京时间13:08:43.778）；开始／结束飞控时间戳分别为293284437／295317437微秒，均处于未解锁、非 Offboard 状态。

实际读取四路 DShot 输出均为0，AUX1–4 对应 M1–4，输出反向与可逆标志均为0，DShot600启用。成功遥测帧为0、CRC错误0；当时用户确认电调动力断开，因此没有将“未收到遥测”记作接线失败。

用户随后确认 AUX1右前、AUX2左后、AUX3左前、AUX4右后与现在接线一致。这是现场人员的接线声明；数据文件中的 `physical_wiring_verified=false` 保留，因为**尚未通过实际旋转证明位置映射**。四台电机的实际转向均尚待点动验证。

该时点累计：参数写入0、执行器命令0、NX访问0。本目录不是实际飞行或动力台架通过证明。

## 历史差异及失败保留

| 本地原始目录（仓库 artifacts/ 下） | 结果 |
|---|---|
| `px4_rotor_commissioning_20261028/baseline_v1` | 未收到 NSH 提示符，未执行配置写入或电机指令 |
| `px4_rotor_commissioning_20261028/baseline_v2` | 读取成功；当时 AUX2=103、AUX3=102 |
| `px4_rotor_commissioning_20261029/baseline_v1` | 未找到 COM13，未发送电机指令 |
| `px4_rotor_commissioning_20261029/baseline_v2` | 用户重新接回 FMU Debug 后读取成功；AUX2=102、AUX3=103 |
| `px4_rotor_commissioning_20261029/baseline_v3` | 补读输出反向、推力曲线、可逆参数与实际可逆标志，均为0 |

本工具未改变映射，不能从这些记录推断是谁或哪个工具改变了 AUX2／3。使用当天读数及用户确认准备后续测试。

原始串口记录包含设备唯一标识，保留在电脑的上述目录，不随此文发布；其内容哈希由 inventory 文件保存。最新 `console.json` SHA-256 为 `c8c64a4747e90e7ed4041c818c55edfbdcb73bafb06c2185efcf897ed83fdb2d`。这里的哈希用于关联本地证据，不代表克隆仓库即可获得原始串口流。

## 软件验证

执行以下测试，33项通过：

```powershell
python -X utf8 -m pytest tests/unit/test_px4_motor_preflight_readonly.py tests/unit/test_px4_motor_single_pulse.py -q
```

模拟串口测试覆盖未知参数、危险命令拒绝、配置与通道变化、重复／过期点动许可、异常停止、未观察到输出、读到其他通道输出等情况。真实只读测试只运行了 `px4_motor_preflight_readonly.py`；`px4_motor_single_pulse.py` 已运行默认计划模式，没有使用 `--execute`。

下一步按 [单电机点动流程](../../PX4_ROTOR_COMMISSIONING.md)进行右前 M1 的一次最小运行输出、飞控1秒超时试验；动力接通与现场准备确认后才发送。实际位置、转向和停止效果需现场反馈，输出回读不能代替物理观察。
