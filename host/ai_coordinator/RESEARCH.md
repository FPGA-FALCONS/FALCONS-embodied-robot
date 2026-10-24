# 架构依据与成熟模块核验

查阅日期：2026-10-24。先查论文与维护者资料，再实施。本模块没有复现论文的完整机器人实验，以下是采用的分工原则及具体工程映射。

后续整合前补查的Dobby、Inner Monologue、OK-Robot、ROS-LLM与低成本语音视觉机器人，以及代码核验、证据等级和C1b/FPGA实际接口缺口，见[多模态任务链路论文核验与整合方案](../../doc/多模态任务链路论文核验与整合方案_20261024.md)。该记录不新增板端闭环或模型性能结论。

| 原始来源 | 核心依据 | 本项目采用方式 | 未实现的原论文部分 |
| --- | --- | --- | --- |
| [SayCan：Do As I Can, Not As I Say](https://arxiv.org/abs/2204.01691)，2022 | 语言建议需落到机器人实际具备的技能与环境证据 | 四类目标与动作白名单；任务执行等设备回执，模型不能自由输出底盘命令 | 未训练技能价值函数，不称SayCan算法复现 |
| [SayPlan](https://proceedings.mlr.press/v229/rana23a.html)，CoRL 2023 | 高层语言规划与低层路径执行分离，并利用执行反馈检查计划 | RK3588管理任务阶段；FPGA保留基础导航与运动控制；“区域搜索完毕”与“单帧未检测到”分开 | 无三维场景图和多层建筑规划 |
| [Behavior Trees in Robotics and AI](https://arxiv.org/abs/1709.00084)，Colledanchise、Ögren | 模块化任务组合，明确成功、失败与运行状态 | 当前任务有限，采用显式有限状态机，保留条件备选与返回；可扩展为行为树 | 没有引入完整BT运行库 |
| [BehaviorTree.CPP异步动作](https://www.behaviortree.dev/docs/guides/asynchronous_nodes/) | 阻塞节点会卡住整棵树，长动作应异步并可停止 | 模型独立进程；协调循环独立运行；取消、超时、旧结果拒收有明确路径 | 未把Python状态机宣称BT.CPP实现 |
| [Nav2 BT Navigator](https://api.nav2.org/nav2-rolling/html/md_nav2_bt_navigator_README.html) | 已有导航系统以动作接口及状态管理长时任务 | 采用任务ID、动作ID、结果关联与取消，留清楚的FPGA适配边界 | 板端没有安装ROS2/Nav2，不占用新的依赖体系 |

## 实际复用的成熟模块

- [Sherpa-ONNX SenseVoice](https://k2-fsa.github.io/sherpa/onnx/sense-voice/pretrained.html)：支持中文离线识别，官方提供Linux/aarch64和RK3588相关使用资料。本轮CPU运行，避免额外占用视觉NPU；不把官方基准当本板测量。
- 已在项目验证的ALSA `arecord`、`voice_bridge`音频选择／重采样／领域纠错。USB麦克风未到货，实际采集尚未验证。
- llama.cpp：复用并复制板端已有构建产物到本部署目录，独立端口与私有进程运行专项GGUF；不占用Claude Code的服务端口，不中止其进程。
- 已有v17 640 FP16 RKNN及RKNNLite：沿用板上模型、驱动和runtime，逐类NMS、RGB letterbox、固定输出合同；不升级系统、不替换竞赛权重。
- Linux标准串口与libusb：可选适配，端口／VID/PID必须显式选定；没有驱动解绑、固件刷写或USB复位。

## 决策

本轮采用“异步事件协调＋有限状态机＋隔离模型适配器”，保持小型标准库控制面；复用成熟推理模块。任务少、FPGA承担基础功能，当前不引入完整ROS2/Nav2或大型Agent框架。该选择是针对现有板端环境的工程判断，不是论文证明了本项目性能。

进一步实现巡航找物时，参考视觉伺服的图像空间/位置空间分工与OpenCV地面平面投影，并通过Git只读核对当前远端`ec759b161f378fcf545a836fe8bed63295efe1a2`的导航/避障和坐标合同；此前本地旧顶层的结论不能代表该远端。新代码与证据边界见[全栈入口](FULLSTACK.md)。

验证范围分层：主机单元测试、板端Linux接口测试、板端真实模型回放、真实USB麦克风／FX2／FPGA／车辆。前三层的成功不能替代第四层验收。
