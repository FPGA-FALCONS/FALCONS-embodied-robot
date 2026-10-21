# FALCONS-embodied-robot

高云ACG720四驱差速机器人主工程，GW5AT-138B、50MHz。模块库已内嵌到`vendor/verilog-reusable-modules`并在本项目维护，旧源库不再更新。

## 先看哪里

| 目的 | 入口 |
| --- | --- |
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
| 历史过程与完整文档索引 | [工作记录](doc/工作记录.md)、[文档索引](doc/README.md) |

## 当前状态（2026-10-21）

**功能面**：中文 **6 页**界面（运行数据显示 / 摄像头 / 雷达建图 / 避障导航 / 非避障导航 / 参数整定）、
HT10A 六路 PWM 遥控、四路轮速 PID 闭环、IMU 偏航链路（含开机零点标定）、航位推算 `odom`、
原非避障控制 `nav_ctrl`、独立避障点位控制 `nav_point_ctrl`、雷达建图（12.8×12.8 m、50 mm/格）/ BFS / 多目的地与双缓冲显示、参数整定页（12 项在线可调）、
OV5640 → DDR3 帧缓存、12 列 CSV 串口上报。

**上板验证**：
- 2026-10-17：控制链路（遥控 → 差速 → 电机）与摄像头画面在实物跑通；极性重构经需求方实测确认
- 2026-10-20：参数整定页上板验证通过
- **Flash 里固化的是 2026-10-20 20:55 那版**（User Code `0x00000C26`）；SRAM 仍为 18:05 避障接入版（User Code `0x0000E2B8`），未固化；本次 50 mm 修复版未下载

**本轮构建**（2026-10-21 19:31，`build.tcl` 1023.9 秒）：
- 综合：Logic 36527（29651 LUT + 6876 ALU）、Register 17119、BSRAM 122。
- 布线后：Logic 36874、Register 17119、BSRAM 143/340、DSP 221/298。
- Fmax：系统 **44.154 MHz**（要求 50）、像素 **43.687 MHz**（要求 49），**时序未收敛**；同域 setup 334 条。
- 同圈命中优先、跨圈确认清障和 12.8 m / 50 mm 地图已生成新位流，尚未下载。构建前的同圈修复独立提交 `4569b60`；其在途布线按用户要求中止。报告、基线比较及新位流指纹见[接入验证](doc/自动导航接入验证.md)。

**仿真**：新增雷达建图、避障选点、BFS、点位控制、显示双缓冲及实际运动闭环；28 台 / 2323 项全回归通过，周期见[接入验证记录](doc/自动导航接入验证.md)。原非避障 UI、控制器和对应测试原样保留。

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
# 全量应用 / 集成自检（28 台，2323 项，含按真实时间推进的运动模型）
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