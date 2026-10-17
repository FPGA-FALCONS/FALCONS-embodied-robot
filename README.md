# FALCONS-embodied-robot

高云ACG720四驱差速机器人主工程，GW5AT-138B、50MHz。模块库已内嵌到`vendor/verilog-reusable-modules`并在本项目维护，旧源库不再更新。

## 先看哪里

| 目的 | 入口 |
| --- | --- |
| ★ **交接 / 接手（3 分钟上手）** | [交接单](doc/交接单.md) |
| 明天先测试手柄与电平转换，不开电机 | [手柄LED上板测试](doc/手柄LED上板测试.md) |
| 手柄通道、针号、标定与失联限制 | [HT10A接线与验收](doc/HT10A接线与验收.md) |
| 当前审查发现、测试和构建证据 | [2026-10-17审查与上板验证](doc/审查与上板验证_20261017.md) |
| 整车烧录、屏幕、IMU与电机操作 | [整车上板指南](doc/上板指南_v1.md) |
| 功能与交互的最高依据 | [需求原文](doc/使用FPGA实现的差速小车功能设计.md) |
| 实现细节、硬件规划 | [实现说明](doc/使用FPGA实现的差速小车功能设计_ai优化版.md)（冲突时服从需求原文） |
| 参数与遗留任务 | [参数总表](doc/参数总表与待定清单.md)、[设计待办](doc/设计落实待办.md) |
| 历史过程与完整文档索引 | [工作记录](doc/工作记录.md)、[文档索引](doc/README.md) |

## 当前状态（2026-10-17）

- 已实现：中文菜单与数据页、HT10A输入、四路速度闭环、IMU偏航链路、整车12列CSV。
- ★ **2026-10-17 首次上板实测通过（关键里程碑）**：手柄遥控四轮行驶（前后/左右方向已标定）、
  模式拨杆两档切换、油门回中解锁、摇杆切换菜单高亮、确认键切页、**停车不再误判堵转** —— 均实测正常；
  显示屏能正常出图。**控制链路已在实物上跑通。**
- ★ **摄像头画面已上板验证通过（2026-10-17 21:35）**：OV5640 → DDR3 帧缓存 → **菜单右侧区域实时显示**，
  画面正常、无撕裂；上电默认停在摄像头页（需求原文 §17）。详见 `doc\工作记录.md` §29。
  ⚠ **Flash 里仍是旧固件（无摄像头）**，含摄像头这版**只在 SRAM、掉电即失** → 要留住需重烧 SRAM 或固化。
- 手柄：CH2油门、CH4转向、CH5确认、CH7 select、CH6模式；CH8闲置。只有两个按钮，没有返回键。
  接收机通道不等于J6针号，接线只查上表文档。
  ★ **按键功能分配是暂定值**（CH5=确认 / CH7=select / CH6=模式），后续按实际操作习惯调整；
  改动点：`falcons_top.v` 的 `u_rc` 例化 + `ht10a_input.v` 的 `CH_*`（零起始槽位索引）。
- 方向标定（2026-10-17 实测得出）：`THROTTLE_REVERSE=1` + `STEER_REVERSE=1`。
  ⚠ **A/B 通道与物理左右轮的映射与文档假设不一致**（`INV_DIR_VEC` / `ENC_INV_DIR_VEC` 仍是旧飞线标定值），尚未整体重标。
- **已知问题：UI 布局/呈现仍有很多问题**（能出图，但排版不对）—— 需求方 2026-10-17 明确"以后再修改"。
  ★ 已修其一（2026-10-17 晚）：**每个文字 label 左侧的"竖排小白点"** —— 根因是文字段门控与点阵数据差一个节拍，
  且"段外"读到的不是空白而是 `ui_text_rom[0]` 首字「摄」。修复后零像素位移，新增 3 条黄金参考回归。
  详见 `doc\工作记录.md` §30。其余 UI 问题仍在，等需求方逐条描述现象。
- 新增独立手柄测试顶层：方向/模式LED、半秒事件指示、原始脉宽CSV；电机13个输出恒0。
- 当前回归：**9个测试台 / 244项检查通过**（Icarus 与 ModelSim 双引擎均已验证）。
- 未完成：UI 重做、ToF/雷达/导航集成、整车顶层集成测试、`INV_DIR_VEC`/`ENC_INV_DIR_VEC` 重标。
- ★ **时序口径（接入摄像头后必须"分区"看，不能只看总数）**：
  - **应用逻辑（电机/遥控/UI）未被污染** —— 逐条核查：25 条最差 Setup 路径里**属于它的 0 条**；
  - 摄像头/DDR3 子树：Setup Violated **76**、Hold Violated **1458**，**全部落在 `u_cam` 内部**
    （DDR3 PHY 的 `u_dqs/HOLD`、`dll_step_base`、`fifo_ddr3_adapter/rd_data_fifo` 读写指针）；
  - 根因：Gowin DDR3 IP 内部时钟 `fclkdiv/CLKOUT` 未 `create_clock`（`TA1132`）—— 示例工程同样如此且已实板验证；
  - ⚠ **不许**用 `set_false_path` 把它盖掉来粉饰数字。（接入前基线：4 Setup / 13 Hold / 8 Recovery。）
- **仍未做的实测（安全相关）**：关闭发射机后接收机是否继续输出旧值、断线行为、S1/S4、低速落地。

## 构建与测试

```powershell
# 全部应用/集成自检；工具未加入PATH时先配置Icarus
powershell -ExecutionPolicy Bypass -File sim/run_sim.ps1 -Simulator Icarus
# ModelSim机器使用同一清单，每个测试独立进程
powershell -ExecutionPolicy Bypass -File sim/run_sim.ps1 -ModelSim '<ModelSim win64目录>'

# 主工程：impl/pnr/falcons.fs
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -GowinSh '<IDE/bin/gw_sh.exe>'
# 独立手柄：impl/rc_led_test/impl/pnr/rc_led_test.fs
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Tcl build_rc_led.tcl -ArtifactDirectory 'impl/rc_led_test/impl/pnr' -GowinSh '<IDE/bin/gw_sh.exe>'

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
| `build.tcl` / `build_rc_led.tcl` | 整车 / 独立手柄构建入口 |

## 维护约定

1. 用户明确指令优先。宏观功能与交互以需求原文为准；实现侧不改写需求来迁就代码。
2. 复用模块只维护项目内`vendor`的一份实现；旧源库保持归档。板级取值放应用层，模块保持独立。
3. 改RTL后跑`sim/run_sim.ps1`；改顶层后重新综合与布局布线。改vendor还要跑对应包单测。仿真不能代替实物，生成位流不能代替时序复核。
4. `.cst`与`.sdc`成对使用，引脚逐针核对硬件手册，不按相邻管脚猜测。测试位流与整车位流用途不同。
5. `.ps1`使用UTF-8 BOM与CRLF，其它文本按`.gitattributes`保持LF；读取中文显式指定UTF-8。源码改后必须重建，不使用旧位流冒充新实现。
6. 任务完成包含文档同步：README和工作记录每次更新；参数、接口、测试、操作和目录内容改变时同步对应说明。证据以工具实际输出为准，参数值与状态不在多处复制。
7. 保留已有用户改动和历史。未经授权不做破坏性删除、强制回退或强制推送，不把本地产物加入仓库。
8. 推送需要用户明确授权。本轮用户要求在现有`main`整理、不新建分支；远端是否同步以`git status -sb`为准，提交信息用`git log`查询。

模块来源与维护细节见[vendor/ORIGIN.md](vendor/ORIGIN.md)；过去每轮的资源、时序和测试数量保留在工作记录中，不在首页并排列作“当前值”。
