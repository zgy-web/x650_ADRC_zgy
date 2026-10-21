# X650 / MN4014 KV330：悬停联调动力配置

本配置用于无臂 X650 的 Gazebo／PX4 悬停联调。用户已指定 MN4014 KV330、T-MOTOR 15×5CF 桨、6S 30000 mAh 电池、配套电调，并确认本轮整机质量取 3.02 kg。电调具体型号尚未提供。本次使用固定 22.2 V 动力曲线，不模拟电池压降或续航。

本目录发布数值配置与准备方法。历史飞行证据单独归档，不作为源码附件；新运行必须独立核验模型、飞行参数及完整起降记录。原始 X650 URDF 与全部 mesh 由用户从外部路径提供，`prepare_x650_flight.py` 只接受其固定审核源 SHA256，不是任意 X650 的通用动力适配器。

## 数据与拟合

用户选中的五个数据点与 [T-MOTOR 官方 MN4014 KV330 页面](https://store.tmotor.com/product/mn4014-kv330-motor-navigator-type.html) 的 **22.2 V、15×5CF** 表格一致。没有采用页面中 KV400、16×5.4 或 17×5.8 桨的数据。

| 厂家油门 | 转速 rpm | 拉力 gf | 电流 A |
|---:|---:|---:|---:|
| 50% | 3900 | 830 | 3.6 |
| 65% | 4600 | 1150 | 5.9 |
| 75% | 5100 | 1430 | 7.8 |
| 85% | 5600 | 1690 | 10.1 |
| 100% | 6000 | 1920 | 11.9 |

换算 `T = gf × 0.00980665 N`、`ω = rpm × 2π/60 rad/s`，以最小二乘、零截距拟合 `T = kT ω²`：

- `kT = 4.828453605732144e-5 N/(rad/s)²`。
- 五点拟合 RMSE 为 **0.1620 N**，最大相对误差约 **1.79%**。
- 转速上限为 **6000 rpm = 628.3185307179587 rad/s**，来源是所选组合表的最高测点。
- 拟合模型最大单电机推力约 **19.062 N**；表中实际最高点为 **18.829 N**。二者差异来自拟合残差。
- 3.02 kg 平均悬停需求约 **7.404 N/电机**，预测约 **3739.4 rpm**。这比最低已测转速 3900 rpm 低约 4.12%，属于外推，尚不是低油门实测标定。
- 转速归一值约 0.6232，推力归一值约 0.3884。厂家油门、PX4 推力指令和仿真转速指令不是同一个量；PX4 输出映射需独立配置。

完整数据与残差保存在 [propulsion_fit.json](propulsion_fit.json)。

## 显式估计项

反扭矩／推力比 `moment_constant=0.016 m`、上升／下降时间常数 `0.0125/0.025 s`、旋翼阻力系数 `1e-4`、滚转阻力矩系数 `1e-6`，均采用 [Gazebo 8 MulticopterMotorModel 参考默认值](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc)，不是 MN4014 实测值。`rotor_velocity_slowdown_sim=10` 是数值仿真设置。此版本可检查闭环是否接通及基本悬停，不能用来宣称实机偏航响应、抗扰性能、续航或极限机动已经准确复现。

## 旋翼结构与惯量处理

原始 CAD 将每个 0.18 kg 组件整体作为转动刚体，包含电机外形和桨。派生模型将电机外形固定到机体，只转动桨叶外观；用位于旋转轴上的轴对称薄盘表示旋转惯量，避免 CAD 的偏心和非轴对称惯量随着转角产生假振动。

薄盘质量采用 [T-MOTOR P15×5 官方单桨质量参考](https://store.tmotor.com/product/polish-carbon-fiber-15x5-prop.html)的 21 g（页面公差 ±1.5 g），半径 0.1905 m、厚度 0.001 m。实际安装桨未称重，电机转子钟罩惯量未独立辨识；这是明确的等效模型。薄盘惯量由几何公式计算，原 CAD 剩余质量及惯量归入机体，通过一阶矩和惯量张量反算机体惯性参数。

生成器逐项检查：

- 全机质量保持 **3.02 kg**。
- 全机 COM 和零关节角时的惯量张量保持不变。
- 全部 281 个 STL、562 处视觉／碰撞引用及初始几何位置保持不变。
- 电机外形固定，桨叶转动；轴对称旋翼的 COM 和惯量不随转角改变。
- 原始 URDF 未修改，派生源拥有独立 SHA256 和报告。

源 CAD 的 ±1.57 rad 限位改成连续关节；速度上限改为所选组合的 628.3185 rad/s。源 `effort=10 Nm` 仍作为数值约束保留，它不是声称实际电机轴扭矩为 10 Nm。Gazebo 使用速度执行模型，真实转子陀螺效应和电机电磁力矩尚未标定。

## 仿真编号

用户授权先建立仿真编号。以下按 +X 机头方向、从机体上方俯视定义，尚不声称与实物接线一致：

| PX4 名称 | Gazebo 索引 | 位置 | CAD link | 转向 |
|---|---:|---|---|---|
| Motor 1 | 0 | 前右 | link_004 | CCW |
| Motor 2 | 1 | 后左 | link_002 | CCW |
| Motor 3 | 2 | 前左 | link_003 | CW |
| Motor 4 | 3 | 后右 | link_005 | CW |

编号图和对应几何审计由 [编号渲染脚本](../../../scripts/render_x650_motor_numbering.py) 从外部原始 URDF 生成，记录源哈希、变换和电机位置。生成图片与审计 JSON 不随源码提供。该审核 URDF 相对电机中心对角距离为 650.274 mm，渲染不得缩放几何凑成 650 mm。

PX4 airframe ID 为 `4999`。几何分配使用相对整机 COM 的 FRD 坐标，不能直接把 URDF 的 FLU 坐标填入 `CA_ROTORn_P*`。生成报告包含每台电机的对应坐标与 KM 符号。

## 生成与绑定

在包含 Gazebo 和现有 Python 依赖的服务器执行：

```bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
/root/autodl-tmp/amsim-venv/bin/python3 scripts/prepare_x650_flight.py \
  --source /root/autodl-tmp/x650_urdf_source_20261017/robot.urdf \
  --output generated/x650_mn4014_v1_source \
  --airframe-id 4999 \
  --native-import
```

该命令不启动 PX4 或发送飞行指令。它创建派生 URDF、资源副本、飞行配置和 `preparation_report.json`，再调用平台原生导入／绑定流程，得到 `amsim_urdf_x650_mn4014_v1_flight`。派生源已转换到 Z 朝上，不能再额外应用上一版的根变换。已有输出目录会被拒绝覆盖，换版本时应使用新的输出目录和明确的模型版本。

此目录的 [flight_profile.yaml](flight_profile.yaml) 保存同一参数配置；[requirements.json](requirements.json) 同时保留尚未确认的硬件项。准备脚本或配置存在不代表已经通过悬停，实际服务器试验结果应另行保存。

## PX4 输出与启动

[4999_gz_x650_mn4014_v1](4999_gz_x650_mn4014_v1) 对应已检查的 PX4 提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。安装脚本检查提交与 airframe ID 冲突，拒绝覆盖不同内容；不启动飞行、不重编译或下载依赖。现有 PX4 构建后可能重建 ROMFS，因此重新构建后应再次执行安装。

```bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
/root/autodl-tmp/amsim-venv/bin/python3 scripts/install_x650_airframe.py \
  --px4 /root/autodl-tmp/PX4-Autopilot
```

此版本 `GZMixingInterfaceESC` 直接将 PX4 输出作为 rad/s 发送。设置 `THR_MDL_FAC=1`、输出 MIN=0、MAX=628 后，推力指令 `t` 映射到 `ω=628√t`，因此 `T=kT·628²·t`。628 是整数参数可表示且不超过 6000 rpm 模型上限的转速，实际单电机上限约 19.04265 N，悬停推力初值 `MPC_THR_HOVER=0.3888125521`。这与约 0.623 的归一转速不同。

四个 `CA_ROTORn_CT` 均取 19.0426484684、`KM=±0.016`。该 PX4 提交会删除全部元素绝对值不超过 0.05 的控制轴；若误用 CT=1，则偏航效能仅 0.016，会被清零。四电机使用相同 CT 尺度，其共同尺度在多旋翼归一分配中消去。此处几何参数按整机 COM 的 FRD 坐标计算，不能套用旧 x500 的臂长。

airframe 设置初始 root 高度 0.5 m，以避开 CAD 起落架最低点约 −0.370 m 与地面重叠。首轮使用 PX4 多旋翼默认控制增益作为调试起点，不代表已完成 X650 PID 整定。结构、方向和输出参数在启动时显式设置；每次正式实验仍须保存 PX4 实际参数和 ULog。

派生模型完成原生绑定、airframe 安装及参数核对后，服务器可通过既有平台启动：

```bash
./scripts/amsim run x650_hover_v1 --headless
```

协议为正常解锁起飞 2 m、稳定确认、5 s 预保持、20 s 测量、5 s 后保持、正常降落并确认解除武装。只对 20 s 测量段统计真实位置和姿态误差；起降过程相对固定悬停点的距离不能当作跟踪误差成绩。位置 RMSE≤0.25 m、最大误差≤0.50 m、最大倾角≤15°是首次接入检查的门槛，不是实机精度承诺。

场景记录全程 `flight_samples.csv`、稳态子集 `hover_samples.csv`、CSV 哈希、模型哈希、原始时间戳、ROS bag 和 PX4 ULog。视频使用真实 X650 网格与 Gazebo 记录姿态重放，见 [视频说明](../../../docs/X650_HOVER_REPLAY.md)；在飞行日志产生前不会生成“悬停成功”视频。
