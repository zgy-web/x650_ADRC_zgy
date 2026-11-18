# 本机 X650 与双电机闭环机械臂导入记录

日期：2026-11-09（Asia/Shanghai）。工作分支：`simulation`。

## 当前结果

两个模型已分别接入本机 Gazebo Harmonic 环境。X650 完成实际起飞、悬停、降落和日志归档；机械臂完成插件编译、静态检查、Gazebo 加载和 ROS 通信。

尚未把机械臂连接到 X650：用户已指定机腹安装，安装位置和零位朝向尚待提供。机械臂闭环动态精度未通过，X650 本次悬停也未达到原协议的全部精度要求，不能将当前结果称为完整组合机飞行验收。

| 项目 | 实测结果 |
|---|---|
| X650 源 URDF 哈希 | 与专用准备工具要求完全一致 |
| X650 派生模型、动力绑定、airframe 4999 | 完成 |
| X650 SDF 与 100 步静态 Gazebo 加载 | 通过 |
| X650 飞行 | 完成起飞、20 秒测量、降落、自动上锁 |
| X650 测量段三维位置 RMS | 0.32949 m，超过原 0.25 m 门槛，结果保留为失败 |
| X650 最大位置误差 / 最大倾角 | 0.45528 m / 0.35776°，这两项通过 |
| 机械臂 C++ 插件、4646 个静态构型检查、SDF 校验 | 通过 |
| 机械臂正常重力动态检查 | 初始检查时闭环误差 1.189 mm，超过 0.5 mm 门槛 |
| 机械臂零重力检查 | 1.245 mm，未通过 |
| 机械臂 0.1 ms 步长、正常重力检查 | 1.143 mm，未通过 |

机械臂三次验证在开始动作前即拒绝，因此没有完成五阶段运动验证；没有放宽检查阈值，也没有通过重置关节位置伪造闭环。

## 输入文件及结构差异

X650 原件：`../x650机架无臂/robot.urdf` 和同目录 `meshes/`。

原件 SHA256：`643f5c1d8a57eeffa822b044cba7b9ec6b1a0d72186b8e524fb0d081be3943bb`。

机械臂工程：`../parallel_arm_humble_harmonic/parallel_arm_ws/`。其中：

- `src/parallel_arm_sim/reference/robot_export.urdf`：原始几何与惯量来源。
- `reference/zero_pose_calibration.json`：实体机械零位与原 URDF 的转换关系。
- `models/parallel_arm/model.sdf`：已校准的实际仿真模型。
- `src/LoopClosure.cc`：闭环约束及独立误差监测插件。
- `config/controllers.json`：两主动关节的力矩 PID 和限位。

该机械臂有两个主动电机、被动连杆和闭环约束，总质量 0.35 kg。它与仓库已有的串联二连杆不是同一个机构。单独将参考 URDF 转成 SDF 会遗漏运行时闭环、校准和控制插件，因此本次使用所提供工程的 SDF 与插件进行接入。原 URDF 和 STL 未修改。

## X650 如何导入

以下命令已在仓库根目录执行；生成目录已有内容时准备脚本会拒绝覆盖。

```bash
../amsim-venv/bin/python3 scripts/install_x650_airframe.py \
  --px4 ../PX4-Autopilot

../amsim-venv/bin/python3 scripts/prepare_x650_flight.py \
  --source '../x650机架无臂/robot.urdf' \
  --output generated/x650_mn4014_v1_source \
  --airframe-id 4999 --native-import

PYTHONPATH=src ../amsim-venv/bin/python3 \
  simulations/x650_hover_v1/run_case.py --dry-run
```

准备工具先核对原件 SHA256，然后在新目录派生模型：把原 Y 朝上的坐标转换为 Z 朝上，分离旋翼与不随旋翼转动的结构，按原工具规则保持整机质量、质心和惯量，输出动力配置，再调用原生 `gz sdf` 转换和飞行绑定器。

使用的是仓库 MN4014 KV330 / 15×5CF / 6S / 3.02 kg 仿真预设。推力曲线来源与反扭矩、动态响应等估计项见 `configs/vehicles/x650_no_arm/README.md`。导入不能替代实物动力标定。

输出：

```text
generated/x650_mn4014_v1_source/
  robot.urdf
  flight_profile.yaml
  preparation_report.json
generated/imported_models/amsim_urdf_x650_mn4014_v1/
generated/imported_models/amsim_urdf_x650_mn4014_v1_flight/
```

查看静态模型：

```bash
./scripts/amsim model-preview amsim_urdf_x650_mn4014_v1_flight --static
```

执行输出的完整 `open_command`，保留其中的独立 `GZ_PARTITION`。关闭预览后，再运行飞行实验：

```bash
./scripts/amsim run x650_hover_v1
# 不开窗口：
./scripts/amsim run x650_hover_v1 --headless
```

本次结果目录：`runs/x650_hover_v1_20261109_161913_105405/`。其中包括 `result.json`、`flight_samples.csv`、参数前后读回、rosbag 和 PX4 ULog。约 0.326 m 的 Z 轴 RMS 是此次三维位置误差的主要分量，原因尚未隔离，未修改 PID 或降低验收标准。

## 机械臂如何接入

已执行所提供工作区的构建脚本，编译 `libparallel_arm_loop.so` 并安装 ROS 包。新增项目入口 `scripts/parallel-arm`，自动读取本机 `.amsim/local.env`，使用 Harmonic 专用桥接，加载机械臂工作区，设置独立 ROS domain 37 和 Gazebo partition `amsim_parallel_arm`。

在仓库根目录的终端 A：

```bash
./scripts/parallel-arm start
# 无窗口：./scripts/parallel-arm start gui:=false
```

这是固定底座独立机械臂，不启动 PX4。终端 B 可执行原始验证：

```bash
./scripts/parallel-arm check
```

当前闭环误差会使检查失败；命令行会显示实际原因并保存报告。处理完闭环问题并通过检查后，运动接口是：

```bash
./scripts/parallel-arm command --big 10 --small 5 --duration 5 --hold 3
```

命令单位为度，实际话题单位为弧度；这是关节角度目标和力矩 PID，不是原平台 `ArmAPI` 的关节速度接口。指令程序会拒绝超限的闭环状态。

在终端 A 按 Ctrl+C 关闭仿真。X650 平台启动器会检查已有 Gazebo 进程，两个独立测试请依次运行。

为排查数值问题，在机械臂 launch 文件中增加了可选 `step_size` 参数，默认仍读取原配置 0.0005 s；原 launch 文件备份在 `.amsim/model_import_20261109/original_arm_sim.launch.py`。使用 `step_size:=0.0001` 的检查未改善到验收阈值。改动后可通过 `./scripts/parallel-arm build` 重新构建。

Gazebo 官方说明通用 DetachableJoint 系统不支持闭环拓扑：https://gazebosim.org/api/sim/8/detachablejoints.html 。当前自定义插件直接创建对应约束组件，能发出请求不代表闭环已被准确求解；本机检查保留了实际失败结果。是否与该实现限制直接相关，尚未完成根因证明。

## 机腹安装需要提供的尺寸

以四个旋翼轴心构成的正方形中心为原点，机头方向为 +X，左侧为 +Y，上方为 +Z。本次导入的四个旋翼位置为 `(±0.22990664, ±0.22990664, 0)` 米，与该参考一致。

请描述机械臂底座安装面中心：

1. 向前或向后偏多少 mm；
2. 向左或向右偏多少 mm；
3. 比旋翼轴心平面低多少 mm；
4. 大臂、小臂零位分别朝哪里；
5. 若有新增安装板，提供厚度和质量（未确定时显式保留未知）。

可以提供装配截图并标出机头、安装面及距离，也可直接给装配 CAD/URDF。当前机械臂独立零位大臂向上，机腹安装时的翻转方向需要明确。

组合时将移除机械臂的 `base_to_world` 固定连接，建立 X650 机身到机械臂底座的固定安装连接，保留内部闭环和两电机控制，并重新计算整机质量与惯量及飞控分配。两者裸模型质量合计 3.37 kg，不包含新增安装板。组合模型还需检查机身/起落架/地面净距，并独立验证固定底座动作、带臂悬停和运动耦合。

## 本次证据

- `.amsim/model_import_20261109/`：安装、加载、运行和诊断日志。
- `runs/parallel_arm_import_20261109/report.json`：正常重力，原步长失败记录。
- `runs/parallel_arm_zero_gravity_20261109/report.json`：零重力失败记录。
- `runs/parallel_arm_small_step_20261109/report.json`：更小步长失败记录。
- `runs/x650_hover_v1_20261109_161913_105405/result.json`：完整 X650 起降及精度结果。
