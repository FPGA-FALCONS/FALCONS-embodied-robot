# FALCONS-embodied-robot

高云ACG720四驱差速机器人主工程，GW5AT-138B、50MHz。模块库已内嵌到`vendor/verilog-reusable-modules`并在本项目维护，旧源库不再更新。

前探已改为从定位中心点开始，格数和车宽由参数整定页在线配置，默认 4 格 / 180 mm，车宽步长 10 mm；取消车前缘偏移和额外侧向余量。51 台 / 9012 项回归通过，本次未重新构建或烧录。见 [验证及页面预览](doc/前探配置运行时化验证_20261025.md)。

Dijkstra 存储已优化为 3 位桶状态、8 位饱和转弯计数、2048 条共享队列，正式接口不变。51 台 / 8958 项回归、58 图综合网表回放通过，原57图航点完全一致。相同主线基线重新构建后，物理 BSRAM **307→204/340（60%）**；系统 Fmax **50.027→50.036 MHz**，像素 **49.459→49.093 MHz**，两域同域违例为0，全局仍有58/18个setup/hold违例。时序余量仍薄。见 [存储优化验证](doc/Dijkstra存储优化验证_20261025.md)。本轮未烧录。

## 先看哪里

| 目的 | 入口 |
| --- | --- |
| **完整AI模块、运动桥与队友SRAM下载（2026-10-24）** | [统一交付入口](doc/全栈上板与队友下载_20261024.md)、[八个AI/主机模块](host/README.md)、[运动桥生产整合](doc/运动桥生产接入验证_20261024.md)、[模型与资产](host/MODELS.md) |
| **AI / 摄像头叠框 / Qwen 文字 / 寻物导航联调** | [开发指南与分工](doc/AI与整车联调开发指南_20261023.md)、[可运行开发包](host/vision_bridge/README.md)、[本轮验证](doc/AI联调交付验证_20261023.md)；观察与回放可运行，实车 AI 闭环待接入 |
| ★ **交接 / 接手（3 分钟上手）** | [交接单](doc/交接单.md) |
| 手柄/电平转换的独立验证（不开电机） | [手柄LED上板测试](doc/手柄LED上板测试.md)（2026-10-17 做过；排查手柄问题时可复跑） |
| 手柄通道、针号、标定与失联限制 | [HT10A接线与验收](doc/HT10A接线与验收.md) |
| 2026-10-17 那轮的审查发现、测试与构建证据 | [审查与上板验证](doc/审查与上板验证_20261017.md) |
| 整车烧录、屏幕、IMU与电机操作 | [整车上板指南](doc/上板指南_v1.md) |
| 功能与交互的最高依据 | [需求原文](doc/使用FPGA实现的差速小车功能设计.md) |
| 实现细节、硬件规划 | [实现说明](doc/使用FPGA实现的差速小车功能设计_ai优化版.md)（冲突时服从需求原文） |
| 参数与遗留任务 | [参数总表](doc/参数总表与待定清单.md)、[设计待办](doc/设计落实待办.md) |
| 导航侧接口与**控制层**（方案 + RTL 已实现） | [导航控制方案](doc/导航控制方案.md) |
| 雷达建图与避障规划模块（已接入，待实车验证） | [模块说明](vendor/verilog-reusable-modules/modules/radar_navigation/README.md)、[完整交接文档](vendor/verilog-reusable-modules/modules/radar_navigation/docs/delivery_handoff.md) |
| 自动导航整车接入 | [详细接入方案](doc/自动导航接入方案.md)：最小闭环、多目标调度、地图与路线双缓冲、新增避障执行器，原非避障调试保留；暂不显示原始点云；[验证记录](doc/自动导航接入验证.md) |
| 雷达坐标翻转修复 | [根因、UART 与网表回放及本轮构建](doc/雷达坐标翻转修复验证_20261021.md)：修改初始向量乘法表达，接口和周期不变；已固化到 Flash，时序仍未达标 |
| 减少斜墙附近的小转弯 | [直行优先 BFS 验证](doc/BFS直行优先验证_20261022.md)：室内地图真实 RTL 对照、周期、资源及布线后时序；接口不变 |
| 路线居中修正 | [最小间隙、端点与参数验证](doc/路线居中修正验证_20261022.md)：包含两端点，累计各侧最小值，默认参数 4 / 20 / 4 |
| 本轮整车时序优化 | [流水与并行优化验证](doc/Fmax流水优化验证_20261022.md)：34 台 / 3191 项通过，两域违例显著减少，仍未达标 |
| 早先整车时序优化 | [优化方案](doc/Fmax优化方案_20261021.md)、[前四项验证](doc/Fmax优化验证_20261021.md)：系统 / 像素同域违例清零，余量仍薄，DDR 相关时钟违例待核对 |
| 历史过程与完整文档索引 | [工作记录](doc/工作记录.md)、[文档索引](doc/README.md) |

## 当前状态（2026-10-23）

**功能面**：中文 **6 页**界面（运行数据显示 / 摄像头 / 雷达建图 / 避障导航 / 非避障导航 / 参数整定）、
HT10A 六路 PWM 遥控、四路轮速 PID 闭环、IMU 偏航链路（含开机零点标定）、航位推算 `odom`、
原非避障控制 `nav_ctrl`、独立避障点位控制 `nav_point_ctrl`、雷达建图（12.8×12.8 m、50 mm/格）/ 直行优先 BFS / 多目的地与双缓冲显示、参数整定页（12 项在线可调）、
OV5640 → DDR3 帧缓存、12 列 CSV 串口上报。

**最新源码**：补画车辆到当前控制目标的水平／竖直线，忽略横向偏差。新增显示输出 `target_axis_y`，控制器六信号接口和其他方向逻辑不变；复用路线双缓冲。详见[补画验证](doc/当前段轴线补画验证_20261023.md)。已于 13:16 完成新构建，系统 / 像素同域违例清零，详见下表；尚未下载。

**上板验证**：
- 2026-10-17：控制链路（遥控 → 差速 → 电机）与摄像头画面在实物跑通；极性重构经需求方实测确认
- 2026-10-20：参数整定页上板验证通过
- 最新 Flash 为雷达坐标修复版 `11114a2`（当时 SRAM User Code `0x00008C99`）。2026-10-21 写入并校验成功，耗时 142.36 秒，覆盖此前 `0x0000D3BA` 版。
- 尚未确认断电重启后的实际运行；板上版本的系统 / 像素 Fmax 仍为 47.167 / 46.768 MHz，与下述新构建分开记录。

**最近完整构建**（2026-10-23 13:16，源码 `e5f5950`，`build.tcl` 677.3 秒）：
- 本轮为补画当前任务轴线后的完整重建；约束和构建设置与上一版相同。
- 综合 Logic **36770**（29712 LUT + 7058 ALU）、FF **17455**、BSRAM **123**；较上版 Logic +404、FF +53。
- 布线后 Logic **37086**、FF **17455**、BSRAM **144/340**、DSP **237/298**；较上版 Logic +406、FF +53，BSRAM / DSP 不变。
- 系统 Fmax **48.958→50.056 MHz**，像素 **48.686→49.462 MHz**；同域 setup **34/1→0/0**，hold 均 0，达到 50 / 49 MHz 约束。
- 最差余量系统 **+0.022 ns**、像素 **+0.191 ns**；系统余量仍薄。全局 setup / hold **58/18**，DDR 相关问题仍在，不代表全工程收敛。
- 位流、输入指纹和完整对比见[本轮构建记录](doc/当前段轴线补画验证_20261023.md)。未烧录。

**仿真**：35 台完整回归及补充定向回归通过，最终 **3234 项**；六页实际扫描像素指纹不变。新增毫米 / 厘米换算连续 int32 检查及 Gowin 网表回放，图层优先级 131072 组合对照。运动闭环仍为 98000033 周期 / 22 轮规划。室内规划保留 30 场景 / 531 项、UART 实物包与原非避障回归。

**未完成 / 已知限制**（完整清单见 [交接单 §8](doc/交接单.md)）：
- RK3588 通路未定；ToF 急停模块未做（阈值 300 mm 仅占位）；避障自动运动和显示已接线，尚未做本轮实车验证
- 导航层参数（`kp_yaw` / `ki_yaw` / `kd_yaw`）只在仿真取过初值，**需上车标定**（参数整定页可在线调，掉电丢）
- 起步加速度限幅未做（`S_ALIGN` / `S_TRACK` 仍是阶跃给定，等效约 1.9 m/s²；方案假设 500 mm/s²）
- 安全相关实测未做：关闭发射机后接收机是否继续输出旧值、断线行为、S1/S4、低速落地
- 三条时序改写路线（A1 位宽压缩、B1' 除法移位分解、方案 A 列号解耦）**实测均为负优化、已全部回退**，
  方法论教训见 [工作记录 §76.7](doc/工作记录.md)

> **本文件只放“现在是什么样”。** 逐条的功能演进史（UI 九处修复、字库放大、极性重构、坐标换基、
> 导航层落地、时序优化全过程）在 [工作记录](doc/工作记录.md) §30~§77；接手细节与硬约束在
> [交接单](doc/交接单.md)。三处不再重复叙述。


## 构建与测试

```powershell
# 全量应用 / 集成自检（30 台，2620 项，含按真实时间推进的运动模型）
# 默认用 ModelSim（本机 D:\Apps\modeltech64_10.6d\win64）
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1
# 没有 ModelSim 的机器用 Icarus
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1 -Simulator Icarus

# 主工程全量：impl/pnr/falcons.fs（实测 7~11 分钟 = 综合约 2 + 布局布线 5~9）
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -GowinSh '<IDE/bin/gw_sh.exe>'
# 只综合（约 2 分钟，只更新资源/告警报告，不动位流）
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Tcl build_syn.tcl -GowinSh '<IDE/bin/gw_sh.exe>'
# 只布局布线（约 5~9 分钟；只改了 .cst/.sdc 时用，RTL 比网表新会被拒）
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Tcl build_pnr.tcl -GowinSh '<IDE/bin/gw_sh.exe>'
# 独立手柄：impl/rc_led_test/impl/pnr/rc_led_test.fs
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Tcl build_rc_led.tcl -ArtifactDirectory 'impl/rc_led_test/impl/pnr' -GowinSh '<IDE/bin/gw_sh.exe>'

python tools/check_pin_constraints.py
```

下载方法见各上板指南。`.fs`、仿真库、波形和日志都是本地产物，不提交Git；队友拉取后需要本机构建。工具默认路径集中在`tools/lib_paths.ps1`，也可通过脚本参数覆盖，避免将某台电脑路径散写进代码。

## 目录职责

| 目录/文件 | 职责 |
| --- | --- |
| `rtl/top` | 整车及手柄测试顶层 |
| `rtl/rc` | 本车遥控配置、输入适配、解锁与LED观察逻辑 |
| `rtl/motor` | 差速和PID应用层 |
| `rtl/debug` | 整车命令/CSV及独立RC诊断上报 |
| `rtl/ui`、`rtl/common` | UI与字库、复位、按键 |
| `vendor/verilog-reusable-modules` | 可复用模块唯一实现，保留独立包结构 |
| `deps/modules.list` | 主工程依赖包清单 |
| `scripts/vendor_sources.tcl` | 派生源文件和约束清单；测试目标可选择现有依赖的子集 |
| `constr` | 主工程唯一管脚与时序约束；测试约束从这里生成 |
| `sim` | 应用与集成测试；包内单测仍在vendor各包的sim目录 |
| `tools` | 构建、下载、字库生成、引脚检查等工具 |
| `doc` | 需求、执行指南、参数、证据和历史工作记录 |
| `build.tcl` / `build_rc_led.tcl` | 整车 / 独立手柄全量构建入口（综合 + 布局布线） |
| `build_syn.tcl` / `build_pnr.tcl` | 只综合 / 只布局布线入口；公共源与约束清单在 `scripts/build_common.tcl` |

## 维护约定

1. 用户明确指令优先。宏观功能与交互以需求原文为准；实现侧不改写需求来迁就代码。
2. 复用模块只维护项目内`vendor`的一份实现；旧源库保持归档。板级取值放应用层，模块保持独立。
3. 改RTL后跑`sim/run_sim.ps1`；改顶层后重新综合与布局布线。改vendor还要跑对应包单测。仿真不能代替实物，生成位流不能代替时序复核。
4. `.cst`与`.sdc`成对使用，引脚逐针核对硬件手册，不按相邻管脚猜测。测试位流与整车位流用途不同。
5. `.ps1`使用UTF-8 BOM与CRLF，其它文本按`.gitattributes`保持LF；读取中文显式指定UTF-8。源码改后必须重建，不使用旧位流冒充新实现。
6. 任务完成包含文档同步：README和工作记录每次更新；参数、接口、测试、操作和目录内容改变时同步对应说明。证据以工具实际输出为准，参数值与状态不在多处复制。
7. 保留已有用户改动和历史。未经授权不做破坏性删除、强制回退或强制推送，不把本地产物加入仓库。
8. 推送需要用户明确授权。本轮用户要求在现有`main`整理、不新建分支；远端是否同步以`git status -sb`为准，提交信息用`git log`查询。
9. **一次任务一次提交**：功能改动与它引发的文档更新写在同一个`git commit`里，不拆成"先提交代码、再补一个文档或hash同步提交"。提交前先把文档里会**自我指涉**的字段（当前HEAD hash、`ahead/behind`、本次提交标题）写成不随该提交变化的形式 —— 否则提交完成即过时，必然被迫补第二次提交。发现遗漏且尚未推送时用`git commit --amend`补齐，不追加新提交；已推送的不擅自amend。

模块来源与维护细节见[vendor/ORIGIN.md](vendor/ORIGIN.md)；过去每轮的资源、时序和测试数量保留在工作记录中，不在首页并排列作“当前值”。

## 2026-10-18：遥控器与ToF更新入口

**以后仅更新本仓库main**，不再向旧模块库或其他仓库推送。本次同步保留现有整车通道映射、设备设置和控制行为。

- [当前遥控/ToF TODO](TODO_RC_CAR.md)、[同步与回归记录](doc/遥控与ToF模块同步_20261018.md)
- [四轴10秒轮换数码管](vendor/verilog-reusable-modules/modules/rc_pwm_receiver/examples/acg720_rc_4ch/README.md)
- [CH5/CH6独立LED](vendor/verilog-reusable-modules/modules/rc_pwm_receiver/examples/acg720_rc_6ch_led/README.md)：已仿真，P&R及实板按键映射待验证。
- [TOF200C/VL53L0X](vendor/verilog-reusable-modules/modules/tof_vl53l0x/README.md)、[SAFE/WARN/STOP数码管](vendor/verilog-reusable-modules/modules/tof_vl53l0x/examples/acg720_tof_debug/README.md)：独立实板已由用户确认，尚未接入整车电机仲裁。

独立观察例程采用138C及自身接线；主工程当前138B设置与J6槽位映射保持不变，整车接线继续以[HT10A接线与验收](doc/HT10A接线与验收.md)为准。
