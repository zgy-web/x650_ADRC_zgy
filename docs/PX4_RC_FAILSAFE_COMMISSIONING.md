# TX16S MK3 → Pixhawk 6X Pro：遥控接管与失联检查

本页记录真实硬件的无动力台架步骤。2026-10-29已确认四个摇杆轴、三个开关的原始通道，完成飞控映射，并验证停机标志切换、关遥控器后的失联识别及重连恢复。**以下摇杆、映射和失联检查均在未解锁、电调动力关闭条件下进行；尚未验证旋转电机的实际停机、空中失联降落或NX Offboard到人工控制的接管。** 最早盘点曾发现电调仍有动力，用户随后关闭动力再进行上述操作。原始结果与范围见[本轮验收记录](validation/px4_rc_commissioning_20261029/README.md)。

## 接线和软件边界

- 遥控器：RadioMaster TX16S MK3；图中ELRS为3.6.3 CN 2G4、500 Hz、Wide。接收机为用户所述BAYCK 2.4G，网页显示ELRS 3.5.2、Unified_ESP8285_2400_RX；这个通用固件目标不能作为具体商品型号。
- 接收机四线接TELEM2。当前PX4的CRSF驱动已在`/dev/ttyS4`运行，`RC_CRSF_PRT_CFG=102`；本轮未改串口或刷固件。接线按TX→RX、RX←TX、供电、共地核对。实际RC输入已在飞控收到。
- 电脑通过COM13、57600 baud连接FMU Debug，接线见[旋翼检查指南](PX4_ROTOR_COMMISSIONING.md)。所有新日志都写电脑。本轮没有连接、部署或修改NX。
- 四桨拆下、电调动力关闭、飞控和接收机保留低压供电。软件的0 V/未连接读数用于交叉检查，不能替代现场人员断开电调动力的确认。

## 已核对的摇杆与开关

| 操作 | 原始通道读数 | 飞控解释/状态 |
|---|---:|---|
| 左杆向下到底 | CH3=1001 | `throttle=-1`，最低油门 |
| 右杆向上到底 | CH2=2000 | `pitch=+1`，前推/低头方向的人工输入 |
| 右杆向右到底 | CH1=2000 | `roll=+1`，向右横滚的人工输入 |
| 左杆向右到底，油门保持中间 | CH4=2000 | `yaw=+1`，从机顶看顺时针偏航输入 |
| SA上/中/下 | CH6=1011/1503/1988 | 三档，对应PX4模式槽1/4/6 |
| SB上/中/下 | CH7=1011/1503/1988 | 上/中不触发停机，下档触发停机 |
| SF两档 | CH5=1011/1988 | 屏幕约-100%锁定、+100%请求解锁 |

布局是Mode 2（美国手）。上述摇杆结果验证物理动作→通道→人工输入，不等于飞行器已实际作出对应运动，也不是全部端点的重新校准。正俯仰人工输入与正俯仰姿态角不要混淆，参见[当前固件消息定义](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/ManualControlSetpoint.msg)。

SF的物理“上/下”口述曾不清楚，正式操作以已经实读的CH5百分比为准，暂不写成“SF上就是锁定”。用户确认现有Mixes源为CH5=SF、CH6=SA、CH7=SB，本轮未重建发射机混控。[ExpressLRS Wide模式](https://www.expresslrs.org/software/switch-config/)的CH5是两档解锁通道。

## 写入的固定配置

| 参数 | 原值 | 新值 | 用途 |
|---|---:|---:|---|
| RC_MAP_KILL_SW | 0 | 7 | SB/CH7紧急停机，最先配置并核对真实`actuator_armed.kill` |
| COM_FLTMODE1 | -1 | 1 | SA上档：Altitude，定高人工控制 |
| COM_FLTMODE4 | -1 | 2 | SA中档：Position，定位控制 |
| COM_FLTMODE6 | -1 | 7 | SA下档：Offboard，为后续NX控制预留 |
| RC_MAP_FLTMODE | 0 | 6 | 以CH6选择模式槽 |
| RC_MAP_ARM_SW | 0 | 5 | SF/CH5解锁开关，最后配置 |

模式枚举来自[当前PX4配置源码](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/module.yaml)。槽2、3、5保持原来的未分配；没有修改四个基本轴的校准、方向或输出分配。SB阈值`RC_KILLSWITCH_TH=0.75`和SF阈值`RC_ARMSWITCH_TH=0.75`均沿用实读值。

模式切换的功能分两层验收：本轮确认SA上档选择`nav_state=1`，中档选择`nav_state=2`；位置估计仍无效，未证明定位控制可运行。SA下档原始通道已核对，Offboard参数已回读，但本轮没有在映射后切入Offboard，未接入NX控制流。`COM_RC_OVERRIDE=1`保持原值，它没有开启Offboard的摇杆自动接管；不能声称“摇杆一动就能从NX接管”。后续应分别验证模式拨杆接管及是否启用摇杆接管策略。

SB下档是切断电机输出的紧急功能，不能当作普通降落。当前已验证下档`kill=True`，中档/上档`kill=False`，再回下档`kill=True`；全程未解锁、输出0。SF配置后仅验证锁定档`arm_switch=3`，没有拨动到请求解锁档进行实际解锁测试。

## 可复现代码与运行范围

[px4_rc_preflight_readonly.py](../scripts/px4_rc_preflight_readonly.py)只允许固定状态和参数查询。`collect()`核对固件、未解锁、非Offboard和零输出，保存独立目录，不覆盖历史记录。`read_complete`表示读取完成，不能用它代表RC有效或全部验收通过。

```powershell
python -X utf8 scripts/px4_rc_preflight_readonly.py --port COM13 --output artifacts/rc_checks/read_001
```

[px4_rc_switch_setup.py](../scripts/px4_rc_switch_setup.py)是本台架的一次性固定配置程序，默认只显示计划，不开串口：

```powershell
python -X utf8 scripts/px4_rc_switch_setup.py
```

执行前需要现场断开电调动力，保持SF/CH5低、SA/CH6低、SB/CH7高、最低油门及其余三轴居中。程序检查这些原始值和有效性，同时要求未解锁、零输出、未连接的电池观察；它不能检测桨是否已拆。

```powershell
python -X utf8 scripts/px4_rc_switch_setup.py --port COM13 --execute --esc-power-off --output artifacts/rc_checks/setup_001
```

入口与检查：`validate_inputs()`验证信号新鲜度和拨杆位置；`snapshot()`核对真实状态；`applied_prefix()`限制恢复范围；`read_parameter()`对受异步警告干扰的读取最多重试三次，只重读、不重发写入；`apply()`先建立并回读停机状态，再写其余参数。脚本只允许这六条固定参数写入及保存，不提供任意参数、解锁、电机或轨迹命令。

如果执行中断，保留已经应用的设置和报告，先重新读取现场状态；只有确认仍是同一计划已完成的连续前缀，才使用新输出目录和`--resume`继续。程序不会自动撤销停机映射，也不会因回读含糊而再次发送前一条写入。参数写入本身会让PX4内部处理停机、锁定或模式请求，不能把“电脑没有发送commander命令”解释为飞控没有状态变化。

`manual_control_switches`枚举为NONE=0、ON=1、MIDDLE=2、OFF=3；ON不是2。软件测试直接对照仓库的PX4消息定义。最终49项RC软件测试通过：

```powershell
python -X utf8 -m pytest tests/unit/test_px4_rc_preflight_readonly.py tests/unit/test_px4_rc_switch_setup.py -q
```

## 失联判断和结果边界

本次在SF锁定、SB停机、电调动力关闭的条件下关闭遥控器，真实飞控报告`rc_lost=True`、`rc_failsafe=True`、`manual_control_setpoint.valid=False`及`manual_control_signal_lost=True`。重新开机、ELRS显示C后，四项恢复正常，飞控继续未解锁、停机标志保持、输出0。

`input_rc.timestamp`在失联后仍可更新，必须同时看`timestamp_last_signal`和失联标志；原始通道值也可能停留在最后收到的值。当前[CRSF驱动](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/rc/crsf_rc/CrsfRc.cpp)没有更新`rc_total_frame_count`，它为0不能判作没收到RC。`manual_control_switches`在输入不变时约1 Hz重发，也不能要求其每次快照都具有RC主输入的更新频率。

配置的`COM_RC_LOSS_T=0.5 s`不是本次测出来的失联延迟。手工关机后离散回读没有捕获准确的关机时刻，本次只确认检测和恢复。未解锁状态下`vehicle_status.failsafe=False`不能反驳`manual_control_signal_lost=True`；空中失联动作要在对应模式、状态估计和动力条件下另行验收。

## 尚未通过的项目

1. 加速度计偏置：配置时出现`High Accelerometer Bias`预检警告。先在机架静止条件下读取传感器/估计状态，再决定是否需要校准；不通过禁用检查来绕过它。
2. 真实解锁、旋转电机停机及遥控接管：本轮未执行。旋翼空载方向已在此前验证，但它不能代替遥控停机的实测。
3. 位置信息及NX Offboard：当前`local_position_invalid=True`、`offboard_control_signal_lost=True`，待定位/控制流准备完成后再验证主动接管、NX退出、数据陈旧和通信中断。
4. 失控策略：实读`NAV_RCL_ACT=3`（Land）、`COM_RCL_EXCEPT=0`、`COM_FAIL_ACT_T=5 s`、`COM_OBL_RC_ACT=0`（Position）。本轮没有修改，也没有证明这些策略在当前无定位条件下适合实飞；需结合后续动捕和人工接管方案复核。
5. 电机极数和遥测：MN4014 KV330厂家规格为18N24P，当前`MOT_POLE_COUNT=14`尚未修改，机械RPM还不能作为已标定结果；详见[厂家参数](https://www.ligpower.com/product/mn4014-kv330-motor-navigator-type.html)。电流/电压标定及ESC CRC根因仍待检查。CRSF接收机CRC与DShot电调遥测CRC是两条不同链路，不能混为一个计数。

离开本轮台架时保持SF的CH5=-100%、SA上档、SB下档、油门最低和电调动力关闭。未进行飞控断电重启，因此参数的断电保持尚未实测。
