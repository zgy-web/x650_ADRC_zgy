X650 + 闭环连杆机械臂：可分发 URDF 模型包
========================================

一、请发送整个压缩包

真正修改后的主文件是 urdf/x650_closed_chain.urdf。
它不是原始CAD URDF的简单重命名：已经包含物理关节树、惯量、碰撞体，
以及Gazebo的闭环、IMU、旋翼动力、传感器和状态发布扩展。

URDF不能单独包含STL网格或可执行插件，因此请保留以下配套文件：
  urdf/                 修改后的URDF（唯一模型结构来源）
  meshes/               九个原始CAD网格
  src/                  闭环/关节伺服和IMU兼容插件源码
  CMakeLists.txt        本机编译插件，无需发送其他电脑的.so文件
  config/               Bullet世界、PX4 airframe和动力配置
  scripts/              构建、Gazebo演示、验证、AMSIM导入工具
  scenario/             完整飞行与算法接口示例
  evidence/             本次交付验证的摘要
  PROVENANCE.json       原始来源哈希与主要改动
  SHA256SUMS            分发文件校验值

包内不含PX4源码、ROS安装、虚拟环境或历史大日志。无需复用原电脑的目录。
不要只发送URDF后宣称对方已经获得可运行的闭环动力学模型。

二、适用环境

目标：与原项目相同的Ubuntu 22.04、Gazebo Harmonic（gz-sim8）环境。
本机验证版本：Gazebo Sim 8.15.0、SDFormat 14.9.0、gz-physics 7.9.0。
必须安装Bullet Featherstone物理后端，以及gz-sim8/gz-plugin2/
gz-transport13/gz-msgs10的C++开发依赖、CMake和C++17编译器。
已有同一完整开发环境通常无需新增依赖；若CMake报缺包，先补齐对应开发依赖。
目标电脑本地编译插件；x86_64与ARM等不同架构不要互传已有.so。
不支持直接当作Gazebo Classic插件使用；其他引擎/版本需要重新验证。

仅运行Gazebo机械臂演示不需要ROS 2或PX4。
完整飞行另需已有AMSIM仓库、ROS 2 Humble、项目Python依赖、DDS Agent，
以及已经编译的PX4 SITL，PX4提交必须为：
  d6f12ad1c4f70ad3230afd7d86e971421e02fef4

三、最短操作流程：只查看模型和运行机械臂

1. 将整个压缩包解压到任意可写目录，进入x650_closed_chain_gazebo目录。
   如果当前另有飞行实验，请先正常结束，避免抢占同名控制话题。

2. 可选，检查文件完整性：
     sha256sum -c SHA256SUMS

3. 构建并运行带界面的固定基座演示：
     python3 scripts/demo.py

   第一次会编译插件，并调用gz sdf -p转换修改后的URDF。
   无需手动调整源URDF中的路径，脚本会按当前解压位置解析网格。
   Gazebo中机体固定，机械臂自动执行小范围关节运动；这不是飞行。
   点击播放或使用默认自动播放。关闭时在启动终端按Ctrl+C。

4. 运行无界面的10秒物理检查：
     python3 scripts/demo.py --check

   应看到 "ok": true。结果保存在本包的runs/<时间>/check.json。
   同时检查关节实际运动和闭合误差；不会把“关节锁死但误差小”误判为通过。
   刚构建过且未改文件时，可加 --skip-build 跳过重新构建。

四、导入同一AMSIM环境，运行PX4飞行

使用具备该项目Python依赖（包括PyYAML）的Python解释器。下面路径是占位符，
需要改成对方电脑实际路径；也可将python3替换为其amsim-venv/bin/python3。

在本包目录执行：
  python3 scripts/install_amsim.py \
    --repo /path/to/safe-aerial-manipulator-testbed \
    --px4 /path/to/PX4-Autopilot

安装会：
  - 从本包URDF重新转换模型，并在目标电脑编译插件。
  - 创建独立模型 amsim_urdf_x650_portable_v1_flight。
  - 创建独立世界 x650_portable_v1，使用Bullet和0.5 ms步长。
  - 安装编号4982的PX4 airframe。
  - 创建 scenarios/x650_portable_flight.yaml 与 simulations/x650_portable_v1。
  - 按目标电脑位置解析本地插件路径，并生成资源哈希清单。
  - 不解锁、不起飞，不覆盖已有模型或实验场景。
  - 若发现旧版GUI把世界名称写死为default，只修正该已知检查，
    并保存visualization.py.before_x650_portable备份；其他GUI实现不重写。

随后进入对方的仓库目录：
  cd /path/to/safe-aerial-manipulator-testbed
  ./scripts/amsim doctor
  ./scripts/amsim run x650_portable_flight --headless

带界面运行：
  ./scripts/amsim run x650_portable_flight

过程为正常起飞1.5 m、Offboard悬停约21秒并驱动两个关节、正常降落和自动上锁。
平台自动保存CSV、ROS bag、PX4 ULog和result.json，并关闭本次运行时。
需要飞行平台，不要用gz sim单独打开URDF来代替这个流程。

停止操作
  完整飞行中优先让短实验正常结束，或在启动终端按Ctrl+C。
  若场景已经结束而窗口/后台仍保留，可在仓库目录执行：
    ./scripts/amsim gui-stop
    ./scripts/amsim stop
  stop会停止仿真，不等同于对实机发送降落命令。

安装器保留已有目标文件。重复安装若提示“Preserving existing destination”，
代表已存在同名模型/场景，请直接运行，不要为绕过检查删除未知文件。
如果4982已被其他airframe占用，应先选择并统一修改一个空闲编号，不能覆盖别人的配置。
PX4重新编译可能重建ROMFS；按项目的airframe安装方法恢复本包配置。

五、URDF做了哪些调整

原URDF的mimic辅助链改为：
  base_link -> upper_arm_Link                         主动关节upper_arm_joint
  base_link -> fore_crank_Link                       主动关节fore_crank_joint
  fore_crank_Link -> fore_long_link                  被动关节C_passive
  fore_long_link -> fore_arm_Link                    被动关节D_passive
  fore_arm_Link -> B_closure_proxy                   被动关节B_passive
  B_closure_proxy <-> upper_arm_Link                 运行时物理固定约束

URDF本身只能表达树，最后一条闭环边由随包插件建立。删除插件会使机构断开。
两个主动关节不是串联二连杆的肩/肘角；它们是基座上的两个电机角度。
使用用户确认的OB=230 mm、BD=50 mm，以及CAD中的OC=50 mm、CD约230 mm。
将B代理连杆坐标原点放在B关节处，同时变换其惯性原点，保持原质量属性。
大臂与代理的质量/惯量为原大臂的95%与5%，总质量约3.005081 kg，不额外加质量。
四个旋翼坐标系对齐，视觉与惯性坐标作对应变换，不旋转错放CAD网格。
mesh路径统一为package://x650_closed_chain_gazebo/meshes/...。
插件引用统一为libx650_cad_arm.so，分发源文件不包含原电脑的绝对路径。

六、控制接口

飞机侧：PX4 Offboard三轴速度参考，保留PX4速度/姿态/角速度环。
机械臂侧：速度指令积分为目标位置，力矩PD驱动实际关节。
命令超过0.25仿真秒未更新时停止积分并保持目标，不是让关节自由下垂。

ROS 2话题（完整AMSIM场景）：
  /arm/upper_arm_joint/cmd_vel           std_msgs/Float64，rad/s
  /arm/fore_crank_joint/cmd_vel          std_msgs/Float64，rad/s
  /arm/joint_states                     sensor_msgs/JointState，rad/rad/s
  /ground_truth/odometry                真值，仅用于评估与监督
  /fmu/out/vehicle_local_position_v1     PX4 NED估计状态

算法开发：安装后编辑 simulations/x650_portable_v1/algorithm.py。
当前是位置P基线，不含ADRC。step接收ENU位置/速度、主动关节状态和目标，
输出ENU飞机速度及关节速度；run_flight.py负责坐标转换、通信和起降。
不要把旧x500串联二连杆控制器直接用于本闭环机构。

七、模型范围和必要说明

1. 旋翼使用项目MN4014 KV330、15x5CF、22.2V近似参数，未重新实测标定。
2. 关节力矩PD和±10 N·m限幅是仿真执行器，未辨识为真实TS20动态。
3. 碰撞使用简化盒体，未验证完整CAD自碰撞或接触作业。
4. 本机DART对该闭环结构发生锁止，因此本包固定使用Bullet Featherstone。
5. 本机Bullet后端未提供正确的平移加速度反馈；ImuAcceleration.cc在Physics
   更新后根据世界速度差计算加速度，交给原Gazebo IMU处理重力和噪声。
   仅支持当前IMU位于base_link原点的配置。切换引擎或升级版本必须重新评估此插件。
6. 任意更改关节坐标、质量、惯量、电机布局、插件或世界配置，都需要重新验证。
7. 已通过的小范围悬停/机械臂运动不证明全工作空间、奇异位形或实机安全。

八、常见问题

“只有URDF，模型看不到”：缺meshes目录，或未由构建脚本解析package路径。
“找不到libx650_cad_arm.so”：没有本地编译，或启动时未设置插件搜索路径。
  使用demo.py/安装器会自动配置，不能拿别人的build目录替代本地编译。
“机械臂不动/闭环断开”：确认Bullet、闭环插件、关节名称和启动流程一致。
“Gazebo server is not visible”：旧GUI可能只接受default世界；本包安装器处理已知旧检查。
“已有仿真运行”：先结束原会话，不要同时运行两个完整PX4实例。
“安装后移动解压目录”：已安装模型和插件已复制到仓库，不再依赖解压目录；
  但整个仓库/PX4目录迁移后需重新解析本机路径。

九、验证记录

evidence/validation_summary.json区分：URDF原生转换、闭环运动、异路径构建和完整飞行。
另一台电脑仍应先运行自己的--check，再进行完整飞行测试。
静态显示成功、gz sdf -k通过、插件能加载，都不能单独代替物理运动或飞行验证。

本次分发版实测结果：
  原生URDF转换与异路径重编译通过；两处路径的转换后SDF哈希相同。
  闭环运动测试通过，最大闭合误差约0.00138 mm。
  新的完整PX4飞行通过：正常起降和自动上锁；约21秒任务窗口，
  位置RMSE约4.34 cm，最大位置误差约7.29 cm，最大倾角约2.43度。
  此结果来自分发URDF重新转换后的模型，不是照搬旧SDF的验证记录。
