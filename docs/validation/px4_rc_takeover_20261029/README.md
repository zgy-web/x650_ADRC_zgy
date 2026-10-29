# Pixhawk遥控停机与失联：无动力实机复测（2026-10-29）

本次完成了真实飞控上的SF解锁/锁定、SB禁止解锁与已解锁输出切断、未解锁和已解锁状态的RC断联/恢复检查。**五项飞控逻辑检查通过，电调动力全程关闭。未验证旋转电机的物理停转、空中失联降落或NX Offboard到人工的接管，不能作为飞行放行。**

使用Pixhawk 6X Pro，固件提交`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；TX16S MK3/ELRS→TELEM2 CRSF；电脑经COM13、57600 baud读取NSH。用户实际操作遥控器，电脑程序只读。未改飞控参数、未发送电脑解锁/模式/电机指令、未访问NX，也未再进行加速度计校准。

## 结果与证据

| 工况 | 正式记录 | 完整采样轮数（另有最终快照） | 实际观察 |
|---|---|---:|---|
| SB有效时拒绝解锁 | [kill_arm_inhibit_v1.json](kill_arm_inhibit_v1.json) | 73 | 捕获新的RC解锁请求和明确拒绝文字；一直未解锁、kill=True、输出0 |
| 未解锁断联/恢复 | [rc_loss_restore_v1.json](rc_loss_restore_v1.json) | 73 | 有效→丢失/无效→有效；一直未解锁、kill=True、输出0 |
| SF解锁/锁定 | [sf_arm_disarm_v1.json](sf_arm_disarm_v1.json) | 49 | 实际armed=True，四路输出109/113/109/113；SF返回后按RC开关原因锁定、输出0 |
| SB已解锁停机 | [sb_kill_disarm_v1.json](sb_kill_disarm_v1.json) | 49 | armed且有输出→armed且kill=True/输出0→按kill-switch原因锁定 |
| 已解锁断联/恢复，重测 | [armed_rc_loss_v2.json](armed_rc_loss_v2.json) | 73 | 实际解锁→RC丢失→failsafe=True、nav_state=12（Descend）→起飞前超时锁定→重连恢复；全程kill=False |

[summary.json](summary.json)保存逐项判据，所有正式记录的`observed_logic_pass=True`仅表示以上逻辑观察通过。**原[armed_rc_loss_v1.json](armed_rc_loss_v1.json)不参与验收**：用户随后确认中途误拨SB，记录也存在kill变化；该轮完整保留，隔离条件判据为False。用户要求重测后，v2期间始终保持SB中档，没有新的kill请求；最终停机操作在v2记录结束后才执行。

已解锁失联重测中的关键快照如下。时间是飞控启动后的`vehicle_status.timestamp`，不等于人工拨杆时刻，各话题也是顺序读取，不能据此测量毫秒级反应时间。

| 飞控时间约(s) | arming_state | nav_state | failsafe | kill | 解释 |
|---:|---:|---:|---|---|---|
| 1990.870 | 2 | 1 | False | False | SF已使飞控解锁，Altitude |
| 1995.944 | 2 | 1 | False | False | 本轮读取中RC已丢失，人工输入无效 |
| 1996.869 | 2 | 12 | True | False | 在仍解锁时进入Descend保护状态 |
| 2001.890 | 1 | 1 | False | False | 已按起飞前超时自动锁定，输出0 |
| 2021.676 | 1 | 1 | False | False | RC恢复有效，没有自动重新解锁 |

这里的Descend是无位置控制的下降模式，不能写成已验证有位置控制的Land。最后锁定的控制台原文是`Disarmed by auto preflight disarming`；起飞前超时与RC失联保护是不同机制，本轮没有证明空中失联会自动锁定，更没有实测空中下降效果。当前`NAV_RCL_ACT=3`的配置意图与无定位条件下实际可用的模式必须分开描述。

## 两个需要保留的实现发现

**上电已有SB下档，不等于实际kill已置位。** [baseline_v2.json](baseline_v2.json)中遥控kill请求为ON，实际kill却为False；用户操作中档→下档后，[sb_down_v1.json](sb_down_v1.json)确认kill=True。当前[ManualControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/manual_control/ManualControl.cpp)首次收到开关输入时初始化前值，后续检测到变化才发布对应动作。因此台架流程应在动力关闭、SF锁定和最低油门条件下操作一次SB中档→下档，并读取实际kill；不能只看拨杆位置。失联后的旧开关值也不等于当前有效输入。

**本版锁定原因按运行时事件枚举解释。** [Commander.hpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/Commander.hpp)采用`events::px4::enums::arm_disarm_reason_t`，实际枚举见[pinned_events_enums.json](pinned_events_enums.json)，原始下载地址和散列在清单中。其中2=rc_switch、7=auto_disarm_preflight、8=kill_switch，与同提交[VehicleStatus.msg](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleStatus.msg)的部分编号不同。SB记录的8与控制台`Disarmed by kill-switch`一致；失联记录的7与控制台起飞前超时文字一致。

最初按消息文件将8解释为起飞前超时的判断已撤回；本次修正的是电脑验收解析器，未改飞控固件或消息包。解析器限定当前固件身份，加入原因表与官方事件枚举快照的一致性测试；以后更换固件需要重新核对。

## 可复现方法

电脑只读记录器：[px4_rc_safety_observer.py](../../../scripts/px4_rc_safety_observer.py)。先检查固件、未解锁、非Offboard、零输出和动力关闭交叉证据，再输出READY。用户之后执行约定动作。运行窗口限制1–90秒，串口固定查询白名单，不接受任意命令。记录后关闭端口，原始采样文字和PC采样区间保留。

```powershell
python -X utf8 -u scripts/px4_rc_safety_observer.py --port COM13 --esc-power-off --seconds 90 --output artifacts/rc_checks/new_unique_run
```

命令标志不能替代现场断电确认。USB单独供电冷启动时电池话题可能从未发布，这会记录为未知，并要求操作者确认电调动力关闭；不会伪造为0 V。记录器不会替操作者解锁、锁定或执行应急停机，也不应在ESC上电条件下使用。`complete=True`只表示记录完成。

离线审计器：[px4_rc_safety_summary.py](../../../scripts/px4_rc_safety_summary.py)。重读原始文字，检查实际状态、请求来源、顺序、未知字段、结束状态及本版原因枚举。不同话题不是同一时刻的原子快照，因此只判断可观察的状态顺序，不给出反应延迟认证。

```powershell
python scripts/px4_rc_safety_summary.py --report docs/validation/px4_rc_takeover_20261029/armed_rc_loss_v2.json --case armed_rc_loss --output artifacts/rc_checks/independent_loss_audit.json
python -m pytest tests/unit/test_px4_rc_safety_observer.py tests/unit/test_px4_rc_safety_summary.py -q
```

拒绝将以下情况误判通过：只有解锁请求、没有实际解锁；自动锁定冒充SF锁定；旧请求冒充新实验；只有人工输入无效、没有完整RC失联证据；失联发生前已经锁定；SB干预混入独立失联实验；原因表使用消息定义中的错位编号。记录期间持续低速查询不是控制循环性能测试。

最终上述两组软件测试合计**32 passed**，包含六份真实记录的重算（五份正式通过、一份有SB干预的记录正确拒绝）、官方运行时原因表核对和归档字节散列校验。

## 结束状态与剩余工作

[final_parked_v1.json](final_parked_v1.json)独立回读：arming_state=1、armed=False、kill=True、SF锁定、SA上档、油门=-1、RC有效、9路DShot观察值均为0，COM13已关闭。用户确认电调动力保持关闭。映射、24极参数及已有校准保持。

这次没有把“遥控接管”整项标为完成：当前无有效水平位置，NX Offboard控制流也未运行。现有`COM_RC_OVERRIDE=1`没有开启Offboard摇杆接管；后续须用真实有效的控制流单独验证SA主动退出、NX中断和失联组合。旋转电机的物理停转另需拆桨、受控动力供电及现场观察，不能用本轮软件输出归零替代。

证据：[evidence_manifest.json](evidence_manifest.json)列出完整采样报告/盘点文件的原始散列和代码身份；[operator_observations.json](operator_observations.json)记录操作范围、误拨排除及重测。完整串口console含`ver all`硬件唯一编号，保留在电脑`artifacts/px4_rc_takeover_20261029/`，本目录发布无该编号的原始报告字节。早期[baseline_v1.json](baseline_v1.json)的无NSH连接失败也保留，没有当作硬件通过记录。
