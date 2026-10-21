# URDF 模型导入与飞行配置

平台现在将自定义机体接入分为两个独立步骤：先导入 URDF 的结构、质量、惯量和几何资源，供 Gazebo 检查；再用显式的飞行配置绑定 PX4、电机与传感器。URDF 导入本身不会生成推力系数、电机顺序或 PID 参数。

已有 `x500_underfold` 模型和已验证的 PID＋前馈控制包保持原有入口。自定义模型也不会覆盖它们。

## 1. 检查输入，不启动仿真

在项目根目录运行：

```bash
./scripts/amsim model-inspect /path/to/robot.urdf
./scripts/amsim model-inspect /path/to/robot.zip
```

输入支持 `.urdf` 文件和包含 URDF、网格资源的 `.zip`。检查会报告链接、关节、质量、惯量、资源引用、初始关节位置和根坐标变换等信息。此步骤不要求 Gazebo，也不会赋予模型飞行能力。

压缩包内有多个 URDF 时，使用 `--urdf-entry path/in/archive.urdf` 明确选择目标文件；平台不会猜测应使用哪一个。

可以显式给出根坐标变换和 `package://` 资源的本地目录：

```bash
./scripts/amsim model-inspect /path/to/robot.urdf \
  --root-xyz 0 0 0 \
  --root-rpy 0 0 0 \
  --package my_robot_description=/path/to/my_robot_description
```

`--package NAME=PATH` 可以重复。平台只从已给出的本地包目录解析资源，不自动下载 ROS 包。变换量使用米、弧度；旋转采用 `Rz(yaw) Ry(pitch) Rx(roll)`。默认全部为零。

根变换会应用到根链接的几何、惯量姿态和一级关节坐标，定义导入模型的机体坐标。它不是 Gazebo 中的出生位置，也不是飞行时要保持的姿态。原始 URDF 和网格文件保留在导入记录中。

当前导入器支持树状的 `fixed`、`continuous`、`revolute`、`prismatic` 关节及本地 STL／DAE／OBJ 网格。遇到不支持的结构、模糊的资源路径、外部材质／纹理依赖或不合法的惯量时会明确报错；不会静默丢弃这些内容。压缩包路径穿越、符号链接和过大的资源也会被拒绝。

## 2. 转换并保存一个独立模型

```bash
./scripts/amsim model-import /path/to/robot.zip --name my_airframe \
  --root-xyz 0 0 0 \
  --root-rpy 0 0 0
```

`model-import` 同样支持 `--package NAME=PATH`。模型保存到：

```text
generated/imported_models/amsim_urdf_my_airframe/
  model.urdf
  model.sdf
  model.config
  import_manifest.json
  meshes/
  source/
```

转换调用当前机器的原生 Gazebo `gz sdf`，并要求 SDF 校验成功。建议在已经配置好的仿真服务器上执行。缺少 `gz` 时不会用自制转换器替代或伪造转换成功。

`import_manifest.json` 包含源文件和发布资源的 SHA256、模型结构、质量、坐标变换与转换记录。此时 `kind` 为 `urdf_import`，`flight_ready` 为 `false`。这是预览和后续配置的基础，不是 PX4 可直接起飞的声明。

相同名称用于标识一个确定的导入结果；源文件、资源或变换发生变化时，应使用新的名称，或按命令提示处理已存在的结果，避免覆盖之前实验的模型。

## 3. 在 Gazebo 中预览

```bash
# 生成固定模型的预览世界与启动命令
./scripts/amsim model-preview generated/imported_models/amsim_urdf_my_airframe \
  --height 0.5 --static

# 有限步数的无界面加载检查，适合服务器
./scripts/amsim model-preview generated/imported_models/amsim_urdf_my_airframe \
  --height 0.5 --static --check --iterations 100
```

`--height` 是预览出生高度，单位为米。`--static` 只用于预览时固定模型；不修改原始导入资产。去掉 `--static` 后，几何模型会参与物理仿真，但没有电机动力的模型会受重力下落。

不带 `--check` 时，命令返回预览世界路径 `world` 和 `open_command`，由用户在有图形界面的环境中执行给出的完整命令打开 Gazebo；不会自动启动 PX4，也不会自动拉起桌面窗口。命令显式设置独立的 `GZ_PARTITION`，形式为 `env GZ_PARTITION=<预览专属编号> gz sim ...`；请保留此前缀，避免预览模型的话题干扰其他仿真。无界面检查也使用预览专属 partition。

`--check` 覆盖 SDF 语法校验和有限步数的 Gazebo 加载。动力参数标定和稳定飞行还需要单独的实验记录。

## 4. 本次 X650 无臂模型

输入文件为 `x650机架无臂.zip`。以下是从文件读出的内容：

| 项目 | URDF／压缩包声明 |
|---|---|
| URDF | `robot.urdf`，机器人名 `generated_robot` |
| 链接 | 5 个：`base_link` 及 4 个旋翼链接 |
| 关节 | 4 个根链接直接连接的 `revolute` 关节 |
| 质量 | `base_link` 为 2.3 kg，每个旋翼链接为 0.18 kg，总计 3.02 kg |
| 网格 | 压缩包内 281 个 STL 文件 |
| 旋翼关节 | `link_002_joint`、`link_003_joint`、`link_004_joint`、`link_005_joint` |
| 对应链接 | `link_002`、`link_003`、`link_004`、`link_005` |
| 旋翼角度限制 | 每个关节为 −1.57～1.57 rad |
| 旋翼速度／力矩限制 | 每个关节声明为 5 rad/s、10 N·m |

这些质量与惯量是模型文件中的数值，不能据此认定已完成实物称重或惯量标定。`link_002` 等名称也不代表 PX4 的电机编号。

从关节变换看，原模型的旋翼轴朝向源坐标的 +Y。以下显式变换可将它们转向导入模型的 +Z，并把旋翼平面移到根坐标的 `z≈0`：

```bash
./scripts/amsim model-inspect '/path/to/x650机架无臂.zip' \
  --root-rpy 1.5707963267948966 0 0 \
  --root-xyz 0 0 -0.17095298

./scripts/amsim model-import '/path/to/x650机架无臂.zip' --name x650_no_arm \
  --root-rpy 1.5707963267948966 0 0 \
  --root-xyz 0 0 -0.17095298

./scripts/amsim model-preview generated/imported_models/amsim_urdf_x650_no_arm \
  --height 0.5 --static
```

这组参数来自几何坐标检查，仍需要确认实体机的前向是否对应导入模型的 +X。这里只完成模型朝上与原点位置的约定，不能自动证明机体满足 FLU，也不能确定电机接线顺序、转向或推力。

该文件尚未提供可用于飞行的电机参数及分配配置。因此本次模型按几何导入和预览处理。有限角度与 5 rad/s 关节限制也需要在动力接入前核实。

这组导入示例使用模型名 `amsim_urdf_x650_no_arm`。原始 URDF 与全部网格由使用者从外部路径提供；历史转换日志和部署证据单独归档，不随源码发布。新导入须保存本次源文件、资源和派生模型的哈希及加载结果，不能把原模型的检查结论自动继承给另一份 URDF。

## 5. 显式绑定 PX4 飞行配置

拥有真实的参数来源、已经配置好的 PX4 airframe 和确认过的电机对应关系后，运行：

```bash
./scripts/amsim model-bind-flight \
  generated/imported_models/amsim_urdf_my_airframe \
  /path/to/my_airframe.flight.yaml
```

这会生成独立的 `amsim_urdf_my_airframe_flight` 目录，保留原来的几何模型。已存在的飞行派生目录不会被覆盖。

绑定需要显式提供：

- 根链接 `base_link` 和 `body_frame: FLU`。这表示配置者已确认前、左、上三个轴的约定；当前 PX4 接口不接受另一个根链接名称。
- 当前 PX4 安装中已经配置的 `px4_sys_autostart`。不能任意填入 x500 的 airframe 编号来代替自定义机体分配。
- 每个旋翼的电机编号、关节名、链接名、旋转方向、推力／反扭矩／阻力参数和电机动态参数；电机编号必须唯一且从 0 连续排列。
- `controlled_joints` 列表。无臂机体使用 `[]`；旋翼不能出现在这里。目前只允许旋转关节（源 URDF 的 `revolute`／`continuous`），因为现有 ArmAPI 的位置和速度接口使用角度单位。`prismatic` 关节可用于几何导入和预览，但不能绑定到此 ArmAPI。
- `notes` 参数来源说明，应记录试验、厂家资料、接线确认及 PX4 airframe 配置依据。

绑定器会检查旋翼关节轴和旋翼链接的局部 +Z 推力方向都沿机体 +Z。单独旋转模型外观不能替代这两项检查。倾转旋翼、非根链接直接连接的旋翼等结构目前不在此绑定接口的支持范围内。

原 URDF 的有限角旋翼，只有对应条目明确给出 `allow_continuous_conversion: true`，才会在新 SDF 中把角度上下限改成 ±1e16，并记录原始限位。这一选项**不会修改速度或力矩限制**。

Gazebo 电机模型的实际关节转速为：

```text
最大仿真关节转速 = max_rot_velocity / rotor_velocity_slowdown_sim
```

绑定时会检查这个值不超过源 URDF 和转换 SDF 中的有限关节速度限制。例如，X650 当前的 5 rad/s 不能因为改了角度限制就被忽略。发生冲突时应依据真实模型修正源限值或电机配置、重新导入；不要仅为通过检查任意调大减速系数。

派生模型会加入 Gazebo 电机系统、所需关节速度执行器、关节状态发布及真值里程计。机载 IMU、磁力计、气压和导航卫星传感器的仿真配置来自已验证的 `x500_underfold` 资产；**只复制传感器配置，不复制质量、惯量、电机系数或 PID**。已有插件／传感器的源模型会被拒绝，以避免重复执行。

派生 manifest 的 `flight_ready: true` 仅表示显式配置完整且通过绑定校验；同时保留 `flight_validated: false`。PX4 分配是否正确、实际升力、悬停稳定性和控制性能仍需分别验证。

### 不可直接执行的配置模板

以下模板故意保留所有尚未确认的映射与系数为 `null`。它会被验证器拒绝，不能直接用来飞行。对四旋翼应补全四个独立条目，不能根据链接名称自行猜测 PX4 电机编号或转向。

```yaml
schema_version: 1
type: px4_multicopter
base_link: base_link
body_frame: FLU  # 只有确认实体机前/左/上方向后才能使用此断言
px4_sys_autostart: null  # 当前 PX4 安装中已配置的机型编号
controlled_joints: []
notes: null  # 参数来源、单位、接线/编号/转向确认及 airframe 配置依据

rotors:
  - motor_number: null  # 四个条目应分别对应确认后的 0、1、2、3
    joint: null         # 从真实关节名与 PX4 编号对应关系填写
    link: null
    turning_direction: null  # cw 或 ccw；按 Gazebo 电机系统约定确认
    allow_continuous_conversion: false
    motor_constant: null
    moment_constant: null
    max_rot_velocity: null
    time_constant_up: null
    time_constant_down: null
    rotor_drag_coefficient: null
    rolling_moment_coefficient: null
    rotor_velocity_slowdown_sim: null
  # 另外三个旋翼也需提供上述全部字段；不可复用未经确认的数值。
```

## 6. 与当前 PID＋前馈控制包的关系

`flyarm-pid-ff 1.0.0` 固定了先前验证的 `x500_underfold` 两关节机械臂几何、质量参数、PID、约束及前馈模型。导入一个新 URDF 不会自动推导出该控制包的新参数，也不会把无臂 X650 当作原来的两关节飞机运行全身协同控制。

对 X650 的合理顺序是：确认几何和坐标系，补全动力及 PX4 分配配置，验证电机方向和升力，再单独整定与验证悬停。将来加上自有机械臂后，需要按新的运动学、质量和执行器接口接入协同控制，并重新建立参数与实验基线。

完成飞行配置与前述检查后，无臂机体应使用 `./scripts/amsim new-scenario <实验名>` 建立独立实验，在新 YAML 中选择自己的 `_flight` 模型，再通过 `ScenarioContext` 的 `vehicle` 接口编写飞行步骤。现有内置 `hover` 和旧两关节实验仍按原机械臂编写，不能仅替换它们的 `model` 字段来运行无臂模型。无臂 API 的 `arm.joints` 为 `[]`、观测中的 `arm` 为 `{}`；平台不等待不存在的机械臂反馈，也不会把旋翼当作机械臂关节发命令。

导入器和绑定器的单元测试用于检查文件处理、参数映射与拒绝条件；Gazebo 加载和飞行验证需以对应运行日志为准。文档中的命令说明接口，不构成已完成所有实验的声明。
