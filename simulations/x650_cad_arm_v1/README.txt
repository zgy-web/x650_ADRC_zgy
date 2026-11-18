X650 CAD 总成：固定基座闭环机械臂试验

飞行算法开发请阅读同目录 FLIGHT_GUIDE.txt；本文件仅描述固定基座试验入口。

用途
  将用户提供的 X650+机械臂 URDF 转为可受力运动的 Gazebo 模型。
  当前机体固定在世界坐标系，旋翼固定，不启动 PX4，不包含碰撞体。
  用于验证闭环机构及控制接口；不能作为空中 ADRC 或接触实验的验收。

启动（从仓库根目录执行）
  ../amsim-venv/bin/python3 simulations/x650_cad_arm_v1/launch.py

带预设运动的 GUI
  ../amsim-venv/bin/python3 simulations/x650_cad_arm_v1/launch.py --exercise

无界面 10 秒物理测试
  ../amsim-venv/bin/python3 simulations/x650_cad_arm_v1/launch.py --check

每次启动重新生成 runs/x650_cad_arm_<时间>，不覆盖以前的结果。
启动器编译本目录的 C++ 插件；默认使用 Bullet Featherstone。
不要直接省略引擎参数运行 world.sdf，默认 DART 在本次闭环结构上发生锁止。

手动控制
  在另一个终端复制启动器打印的 export GZ_PARTITION=...。
  gz topic -t /x650_cad_arm/upper/position -m gz.msgs.Double -p 'data: 0.15'
  gz topic -t /x650_cad_arm/crank/position -m gz.msgs.Double -p 'data: 0.10'
  单位 rad，目标相对于 CAD 导出时的零位。--exercise 模式会覆盖手动指令。
  /x650_cad_arm/joint_states 发布 Gazebo JointStatePublisher 的模型消息。
  暂未建立这组模型的 ROS 2 bridge；这些命令是 Gazebo Transport 命令。

物理结构
  O：两个基座驱动关节所在的同轴位置。
  C：曲柄与细长连杆的被动铰链。
  D：细长连杆与小臂的被动铰链。
  B：小臂与大臂的闭合铰链。
  用户确认 OB=230 mm、BD=50 mm；OC=50 mm 来自 URDF，CD 原始轴距约230 mm。
  两个主动关节、三个被动关节，共两个独立运动自由度。
  删除两个无实体的 mimic 辅助连杆，不把它们人为赋予质量。
  B 用被动转动关节连接代理刚体，代理刚体再与大臂建立物理固定约束。
  将大臂原质量与惯量的5%分配给同位姿代理刚体，95%留在大臂；
  两者合计保持原质量、质心和惯量。组合总质量约3.005081 kg。
  轴向对齐及尺寸校准最大原点调整约3.53微米；原始网格、URDF不修改。
  原始源、网格及插件哈希和校准数据见每次的 preparation.json。

执行器
  两关节使用力矩 PD：Kp=40、Kd=0.6、力矩限幅±10 N·m。
  目标角度变化率限制0.3 rad/s；保留URDF主动关节位置限位。
  这是试验用力矩伺服，不是辨识过的真实 TS20 执行器模型。

验证
  采用重力、0.5 ms物理步长，依次零位保持、单轴运动、双轴运动、返回。
  独立用连杆世界位姿计算B点闭合误差及轴线夹角。
  同时要求两主动关节实际位移超过0.1 rad，防止约束锁止被误判为成功。
  门槛：闭合误差<=0.5 mm、轴线误差<=0.005 rad、稳定段跟踪误差<=0.05 rad。
  CSV记录实际位置、速度、目标和输入力矩；check.json保存验收结果。

已完成的证据（2026-11-17）
  runs/x650_cad_arm_20261117_123906_658585：DART闭环锁止，验收失败。
  runs/x650_cad_arm_20261117_123953_784681：DART关闭闭环，能运动但闭合误差237 mm，失败。
  runs/x650_cad_arm_20261117_124021_708748：Bullet闭环通过，999个样本，9.995秒。
    最大闭合误差1.140532微米；最大轴误差4.94e-7 rad。
    主动关节运动幅度0.303056 / 0.250440 rad。
    稳定段最大跟踪误差0.003994 / 0.000434 rad。
  这些结果只覆盖本次小范围轨迹，不证明全工作空间、碰撞或空中稳定性。
  runs/x650_cad_arm_20261117_124128_929348：Bullet关闭闭环负对照，
    最大闭合误差276.671 mm，验收如预期失败，说明监测能够识别断开的机构。

后续飞行接入
  需要单独建立旋翼转动/动力、传感器、PX4 airframe、通信和碰撞模型；
  在解除基座固定后重新验证质量惯量、闭环和悬停，不能直接移除固定关节即宣称可飞。
  原有 x500 串联二连杆控制器不能直接套用于这个闭环机构。
