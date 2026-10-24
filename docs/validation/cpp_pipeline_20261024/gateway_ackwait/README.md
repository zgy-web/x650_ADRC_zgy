本目录保留“同一输出时刻有界等待消费确认”方案的软件验证历史。用户随后要求正式 tracking 改为最新有效指令流与异步反馈，因此这份证据明确 `deployment_eligible=false`，没有用于新的物理悬停，也不表示流式方案已经实现或验收。

独立服务器工作区 `/root/autodl-tmp/flyarm_gateway_ackwait_20261024` 的 Release 构建及 5 项 CTest 通过。真实 ROS 私有域、模拟消费消息、loopback UDP 的 7 项检查通过：延迟确认、永久缺确认、错误 counter、等待中 RuntimeStop、等待中 SIGINT、消费状态过期、source 过期。延迟确认用例的两次正常发送间隔为 25.786047 ms，保持同一原定输出时刻，没有延长原命令租期。原有 PX4 PVA 编码与退出 transport 检查也通过：候选 1–4 均收到消费确认，7 次发送对应 7 次 receipt，停止零命令被消费。

前两个测试脚本尝试分别在 ROS uint64 setter 类型和 Python 闭包变量作用域上出错；原始日志与未完成报告均保留。第三版修正测试脚本后一次完成全部 7 项，测试期间 gateway 二进制没有变化。消费确认仅证明模拟插件消费了速度命令，不证明关节到达了目标位置。

ZIP 内含本次 gateway 源码快照、测试脚本、构建日志、原始 dispatch/send/receipt 消息、等待时序和失败尝试。`bounded_wait_proof.json` 逐项绑定服务器原件的字节数与 SHA256；取回时已验证全部 107 个条目。ZIP 的 SHA256 为 `842644c902ce629e439df337b1d6d54c3c8877ac298b2d380e2b518e83adcc5b`。原二进制仍保留在服务器独立工作区，SHA256 为 `7658209b65487297d23a1ba06b4ed12c03b29002e90aaa8ba2634cf84b9ccf3c`。

这些结果只证明本方案的软件契约，既不证明真机实时性，也不把有界等待方案确立为 MPC 必须采用的执行方式。
