# 2026-10-29真实遥控通道、开关与失联检查

## 本轮结论

真实TX16S MK3 / BAYCK ELRS接收机通过TELEM2 CRSF向Pixhawk 6X Pro提供有效遥控输入。四轴为Mode 2，物理摇杆动作及方向与PX4人工输入相符。三个开关源是SF→CH5、SA→CH6、SB→CH7；已在飞控配置解锁、三档模式及紧急停机映射。动力关闭、未解锁条件下，SB停机标志切换以及遥控器关闭/重开的失联识别和恢复均有真实回读。

全套使用方式、参数含义和后续阶段见[操作指南](../../PX4_RC_FAILSAFE_COMMISSIONING.md)。下列结论**不包括实际解锁、电机旋转后的停机、NX Offboard接管或空中失控保护动作**。检查结束时未解锁，CH5低、SA上、SB下、最低油门，`kill=True`，九路DShot状态输出均0。49项软件测试通过，全部为无硬件单元测试。

## 证据组织

[manifest.json](manifest.json)列出23份原样复制的结构化报告、源路径、文件SHA-256及相应本地控制台日志SHA-256。[state_summary.json](state_summary.json)是由21次只读盘点提取的通道、人工输入、开关和状态字段；不使用`read_complete`直接判定硬件通过。原始控制台串口字节可能包含设备唯一标识，只留在电脑`artifacts/px4_rc_commissioning_20261029/`，没有复制入本目录，也没有写入NX。

当前固件固定为`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，板型FMUv6X、PX4 1.17.0。用户描述的物理动作及Mixes源见[operator_observations.json](operator_observations.json)。控制台能验证输入与标志，无法自行识别现场哪一根摇杆被推动；两类证据分开保留。

## 通道确认

| 记录 | 用户操作及输入 | 回读 |
|---|---|---|
| [baseline_v4_connected](baseline_v4_connected.json) | ELRS显示C | 16通道、source14、人工输入有效、LQ100 |
| [left_vertical_down_v1](left_vertical_down_v1.json) | 左杆下拉 | CH3=1001，throttle=-1 |
| [right_vertical_up_v1](right_vertical_up_v1.json) | 右杆上推 | CH2=2000，pitch=+1 |
| [right_horizontal_right_v1](right_horizontal_right_v1.json) | 右杆右推 | CH1=2000，roll=+1 |
| [left_horizontal_right_v1](left_horizontal_right_v1.json) | 左杆右推，油门居中 | CH4=2000，yaw=+1 |
| [sa_middle_v1](sa_middle_v1.json) | SA中 | CH6=1503 |
| [sa_down_v1](sa_down_v1.json) | SA下 | CH6=1988 |
| [sa_up_sb_middle_v1](sa_up_sb_middle_v1.json) | SA上、SB中 | CH6=1011，CH7=1503 |
| [sb_down_v1](sb_down_v1.json) | SB下 | CH7=1988 |
| [sb_up_sf_down_v1](sb_up_sf_down_v1.json) | SB上、口述SF下 | CH7=1011，CH5仍1011；此时未据口述确定SF方向 |
| [sf_other_high_v1](sf_other_high_v1.json) | SF换另一档，屏幕CH5=+100% | CH5=1988 |
| [config_safe_positions_v1](config_safe_positions_v1.json) | CH5=-100%、SA上、SB下 | CH5=1011、CH6=1011、CH7=1988，油门1001，电池未连接/0V |

上述仅抽样一侧摇杆端点和中心，没有完成整套摇杆重新校准。SF以CH5百分比标识锁定/解锁，不推断物理上下方向。三档SA和SB数值均是原始RC数值，不是实际PWM输出测量。

## 配置与恢复过程

16:09（北京时间）运行[switch_setup_v1](switch_setup_v1.json)，只发送了`RC_MAP_KILL_SW:0→7`。飞控回复写入，并产生停机预检日志；接着的参数读取同时夹入异步`WARN [health_and_arming_checks]`，旧解析器将整个回复判为未知。脚本因此停止，没有继续写入解锁/模式映射，也没有自动回退停机。

[after_partial_setup_v1](after_partial_setup_v1.json)独立确认映射已经是7、`kill_switch=1`、`actuator_armed.kill=True`。这是诊断脚本回读处理问题，不是飞控拒绝了停机映射。

修复采用有界的**只读重试**，继续保留严格解析，绝不因未知回读重发写入。同时核对当前PX4消息定义，修正新脚本对开关枚举的错误假设：实际ON=1、OFF=3，不能把ON当作2。最终测试直接对照仓库消息定义，并增加异步警告、部分写入恢复和非法恢复状态的回归用例。

16:11运行[switch_setup_v2](switch_setup_v2.json)，使用`--resume`核对前缀确实只有停机映射已应用，从第二项继续，共补写五项：`COM_FLTMODE1=1`、`COM_FLTMODE4=2`、`COM_FLTMODE6=7`、`RC_MAP_FLTMODE=6`、`RC_MAP_ARM_SW=5`。随后六项全部读回目标值，当前模式为定高、SF锁定、SB停机。已发出`param save`并正常返回控制台，未做飞控断电重启验证，故`powercycle_persistence_verified=False`。

两次程序合计六条固定参数写入，无重复写入；一次保存。没有显式解锁、电机、轨迹或commander模式命令，没有NX访问。映射生效会触发PX4内部停机/锁定/模式处理，这与外部发送命令的计数分开解释。

## 映射后的真实检查

| 记录 | 观察 | 范围 |
|---|---|---|
| [kill_release_middle_v1](kill_release_middle_v1.json) | SB中：kill_switch=3、kill=False；SF锁定 | 停机标志解除，非旋转电机测试 |
| [kill_release_up_mode_middle_v1](kill_release_up_mode_middle_v1.json) | SB上仍kill=False；SA中：slot4、nav_state2 | 定位模式选择已传达，但位置估计无效 |
| [pre_link_loss_v1](pre_link_loss_v1.json) | SA上slot1、nav_state1；SB下kill=True；SF锁定 | 断联前安全状态 |
| [radio_off_v1](radio_off_v1.json) | rc_lost/rc_failsafe=True，manual.valid=False，manual_control_signal_lost=True | 断联被识别，仍未解锁/kill=True/输出0 |
| [radio_reconnected_v1](radio_reconnected_v1.json) | rc_lost/rc_failsafe=False，manual.valid=True，manual_control_signal_lost=False | 重连恢复，同时保持锁定/kill/零输出 |

关机和回读由人工配合，不能用相隔约24秒才读取到的状态声称失联检测用了24秒，也不能把参数0.5秒当成本次实测延迟。`input_rc.timestamp_last_signal`在断联时停止推进；通道值可能保留旧值，但有效性标志已清除。重连时`switch_changes`从之前7变成1，不能把该计数当作跨断联始终单调的事件总数。

最初[baseline_v1](baseline_v1.json)曾显示电调有动力，因此先让用户断开；[baseline_v2](baseline_v2.json)因COM13不在而失败，保留无写入结果；[baseline_v3](baseline_v3.json)恢复串口后动力关闭但RC尚未连接；随后baseline_v4首次确认有效RC。没有删除这些失败/未就绪记录。

## 尚待解决

- 配置日志出现`High Accelerometer Bias`，需要静止状态下读取IMU/估计器并排查，不能跳过预检。
- 实际解锁、真实旋转后的停机、NX到人工接管和飞行中的失联动作未验收。
- 本轮没有切入映射后的Offboard；位置估计无效，NX控制流未接入，动捕维护中。模式参数的存在不表示控制条件已满足。
- `COM_RC_OVERRIDE=1`、`NAV_RCL_ACT=3`、`COM_FAIL_ACT_T=5`、`COM_OBL_RC_ACT=0`等保留原值。后续需要为NX/动捕路线明确选择接管和失联策略。
- MN4014 KV330厂家规格24极，而现有`MOT_POLE_COUNT=14`未修改；机械RPM、ESC电压/电流仍未完成标定。此前DShot遥测CRC故障的根因未隔离。本轮CRSF计数曾从0到1，后续保持1，不能将它与ESC CRC混为一谈。
