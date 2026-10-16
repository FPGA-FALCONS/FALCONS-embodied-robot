# FALCONS-embodied-robot

FPGA 四驱差速小车（具身机器人）的**主工程**。板卡：高云 **GW5AT-138B**（ACG720 开发板）。

> 远程仓库：**`https://github.com/HITSZ-FPGA-FALCONS/FALCONS-embodied-robot.git`**（分支 `main`）
> 与 `class\car_fpga` 的分工：`car_fpga` 是**试验场**（每条链路先在那里上板验证），
> 本工程是**正式项目** —— 采用"**模块库内嵌 vendor 快照**"，依赖被钉死在一份可复现的 commit 上。
> 方案依据：`class\模块库内嵌方案_反馈.md`；工作区切换背景：`class\工作区切换_交接单.md`。

> ★ **功能设计文档（AI 扩写版）**：`doc\使用FPGA实现的差速小车功能设计_ai优化版.md`（2026-10-15 从
> `class\` 纳入，**纳入时 652 行、此后多次修改，现 678 行**）—— **实现层面的主要依据**：赛题与范围、
> J1~J9 外设接口总表、供电预算、时钟域与复位策略、模块清单、开发路线与验收判据、待确认信息清单。
> 它**不是**宏观行为口径的最高依据（见下方铁律 5）。
>
> ★★ **需求原文**：`doc\使用FPGA实现的差速小车功能设计.md`（**37 行**）= 需求方自己写的功能清单 + UI 交互初稿，
> 是**需求原文，即宏观设计与功能行为的最高依据**（2026-10-16 自工程根用 `git mv` 移入 `doc\`，保留历史）。

> ★ **工作记录**：`doc\工作记录.md` —— 本轮做了什么、怎么验证的、踩了哪些坑、为什么这么决策，
> 连"哪些是实测数据、哪些是推断"也一并标明，便于回溯与交接。

---

## 〇、当前状态一览（2026-10-16）

| 项 | 状态 |
| --- | --- |
| 进度 | 骨架 + 中文四页界面（含实时数值）+ 遥控输入（**HT-10A：5 个控件 / 6 路接口**）+ 四路速度闭环 + IMU 偏航角链路 + 串口 CSV 上报（12 列）；**上板前补全 A/B 六项已完成**；**2026-10-16 按需求原文与需求方裁定整改**（删自动档、两段式切页、数据页 9 字段、信号有效性掩码、字表 `ST_CURSOR`）；**2026-10-16 晚合并队友的 HT-10A 遥控分支**（配置层 `ht10a_input` + 手动解锁守卫 `rc_drive_guard` + 修掉"推油门即失能"的功能错误）；ToF/雷达/**导航页**为预留 |
| 仿真 | **6 个自检 tb / 203 项检查全 PASS**（ht10a_input 83 / rc_input 32 / chassis_ctrl 23 / num_text 39 / ui_render 22 / speed_report 4；`sim\run_sim.ps1` 一条命令，`-Simulator ModelSim\|Icarus` 双引擎）；★ **顶层集成测试仍缺**（`falcons_top` 不在仿真源列表里） |
| 综合 | **ERROR 0**；比特流 `impl\pnr\falcons.fs`（**34.2 MB**、**2026-10-16 20:51:52**、SHA256 `4FA40D37…3493`，未入库） |
| 资源 | Logic **9112**/138240（7%）、Register **4227**/139095（4%）、BSRAM 9/340（3%）、DSP 43/298（15%）、IOLOGIC 8/285（3%） |
| 时序 | **SETUP 路径 0 条负 slack（最差 +0.032 ns）/ REMOVAL 0 条（+0.269 ns）**；HOLD **13** 条（最差 −1.026 ns，sys→pixel 同步器第一级）；RECOVERY **8** 条（最差 −5.983 ns，复位释放 → HDMI 245 MHz 串行域）。★ 报告头部 `Numbers of Setup Violated Endpoints` 与路径表**口径不一致，本轮未查清、待全量 `report_timing`** —— **不要写成"0 违规端点"**；性质见 `doc\工作记录.md` §20.7、§21.4 |
| 模块库 | `vendor\` 里 10 个包 / 144 文件；来源 commit `c9b8025`（**2026-10-15 起本项目直接维护**，见 `vendor\ORIGIN.md`） |
| Git | 首次提交 `bb9ea07`（199 文件 / 72107 行）；当前 `main` = **`43be57a`**（全项目审查修复 + 重新综合）—— 相对远程 **ahead 1、未推送**（实测 `git status -sb` = `## main...origin/main [ahead 1]`）；远程跟踪已建立；队友的 `feat/ht10a-channel-integration`（基点 `701144b`）**已合并进 `main`** |
| 上板 | **尚未做过**（本轮所有结论来自编译、仿真、综合与时序报告） |

---

## 一、四条铁律

| # | 规则 | 为什么 |
| --- | --- | --- |
| 1 | **`vendor\verilog-reusable-modules\` 里的模块就是本工程的代码** —— 直接改、直接提交；**改完必须跑 `sim\run_sim.ps1` 回归** | 模块是整车的一部分（电机驱动 / IMU 驱动 / 遥控捕获 / HDMI 显示都在里面），改了它们整车行为就变了 |
| 2 | **源库 `verilog-reusable-modules` 已归档只读**：不再同步、不再维护，也不要动它 | 只留作开发历史与将来的复用出口；本工程构建 / 仿真 / 综合**只读 `vendor\`**，与它无关 |
| 3 | **`.ps1` 必须是 UTF-8 with BOM**；读中文文件必须显式指定 UTF-8；改顶层后必须跑一次综合；引脚约束逐针抄硬件手册 | 分别是：PS 5.1 按 GBK 读会乱码+语法错、`Get-Content` 会静默数错行、仿真过 ≠ 能综合、引脚错位看起来像"模块坏了" |
| 4 | **没有明确指令，不做破坏性动作**：不删文件/目录、不回退（`reset --hard` / `clean -fd` / `checkout -- .`）、不覆盖工作区外文件、不擅自 push | 这类操作**难以追溯与恢复**。**"解释原因 / 讨论方案 / 问意见"都不等于授权** —— 动手前先把"删什么、影响什么、怎么恢复"列出来等确认 |
| 5 | ★ **宏观设计与功能行为一律以 `doc\使用FPGA实现的差速小车功能设计.md`（需求原文，37 行）为准**；`..._ai优化版.md`（**678 行**）是实现层面的主要依据，但它对需求的复述**不保证逐字等价**，**冲突时以需求原文为准** | 2026-10-16 的教训：实现时主要对照了 AI 扩写版并自行做了若干"等效"决策（把摇杆切菜单做成按键切页、数据页改布局、把 `ui_compositor`/`menu_ui` 合并），**需求原文里这些行为是写死的**。**实现与需求原文不一致 = 缺陷，不是另一种设计**；要改的是实现，不是需求 |

---

## 二、目录结构

```text
FALCONS-embodied-robot\
├─ vendor\verilog-reusable-modules\   ★ 模块库内容（10 个包 / 144 个文件 / 2.8 MB）
│    ├─ ORIGIN.md                     ★ 来源记录 + 改模块的纪律（2026-10-15 起本项目直接维护）
│    └─ modules\<包>\{rtl, sim, constraints, examples, README.md, ...}
├─ deps\
│    └─ modules.list                  ★ 本工程【编译进工程】的包清单（build.tcl 从它派生）
├─ tools\
│    ├─ lib_paths.ps1                 所有路径的唯一定义处（被其它脚本 dot-source）
│    ├─ build.ps1                     一键构建（调 gw_sh）
│    ├─ run_all_modules.ps1           跑 vendor 里各模块自带的测试（改了模块后回归用）
│    ├─ gen_font_rom.py               中文字库生成器（字表 → 点阵 ROM，数据内嵌）
│    ├─ font_chars.txt                UI 文案字表（76 条）
│    ├─ program_flash.ps1             位流下载/固化（封装 programmer_cli）
│    └─ check_pin_constraints.py      ★ 引脚约束逐针复验（对手册 PDF 原文比对）
├─ scripts\vendor_sources.tcl         Tcl 侧源文件清单（从 modules.list 派生）
├─ rtl\
│    ├─ top\falcons_top.v             整车顶层（例化为主；★ 目前还含启动门禁、切页、跨域同步、
│    │                                 数值换算 —— 与功能设计文档"顶层只连线"的要求不符，待整改）
│    ├─ rc\ht10a_input.v             ★ 配置层：HT-10A 槽位映射与标定参数（`CH_*`、`SIG_VALID_MASK = 6'b011111`）
│    ├─ rc\rc_input.v                 适配层：归一化 / 两档模式 / 按钮 / 菜单拨动 / 信号掩码；★ 事件门控真源 `cursor_ready`
│    ├─ rc\rc_drive_guard.v           解锁守卫：油门回中解锁一次后保持（切档 / 丢信号 / 急停 / S1 撤销）
│    ├─ motor\chassis_ctrl.v          差速分解 + 4×增量式 PID；speed_pid.v（搬自 car_fpga）
│    ├─ ui\ui_render.v                中文界面渲染；num_text.v 数值格式化；dyn_text_ram.v 跨域字符缓冲
│    ├─ debug\uart_cmd_rx.v           串口命令解析（搬自 car_fpga）；speed_report.v（顶层 `u_report` 已例化，12 列 CSV @50 Hz）
│    └─ common\                       按键去抖、复位同步、字库与字符串 ROM
├─ constr\falcons.cst / .sdc          板级与电气约束（成对给出；60 个 IO_LOC）
├─ sim\                               6 个自检测试台 + run_sim.ps1 批量入口
├─ doc\                               文档目录（索引与优先级见 doc\README.md）
│    ├─ 使用FPGA实现的差速小车功能设计.md          ★★★ 需求原文（宏观设计与功能行为的最高依据，37 行）
│    ├─ 使用FPGA实现的差速小车功能设计_ai优化版.md  ★ AI 扩写版（实现层面主要依据；冲突时以需求原文为准）
│    ├─ HT10A接线与验收.md                        ★★ HT-10A 接线表 / 标定参数 / 实物验收（**接线唯一真源**）
│    ├─ HT10A验证记录.md + HT10A验证数据.json     合并前的分支基线（历史证据；当前口径见本 README 与工作记录）
│    └─ 工作记录 / 上板指南_v1 / 参数总表与待定清单 / 模块库接口速查 / 设计落实待办 / README
├─ build.tcl                          主构建脚本（产物 impl\pnr\falcons.fs）
└─ logs\                              脚本日志（.gitignore 已忽略）
```

---

## 三、快速开始

```powershell
cd "D:\Programs\FPGA Projects\FALCONS-embodied-robot"

# ★ 本机执行策略禁止直接跑 .ps1，所有脚本都要加 -ExecutionPolicy Bypass

# 1) 跑仿真（6 个自检 tb，每个开独立 vsim 进程）—— 改了代码或 vendor 里的模块后必跑
powershell -ExecutionPolicy Bypass -File sim\run_sim.ps1

# 2) 改了 vendor 里某个模块 → 顺手跑那个模块自带的测试
powershell -ExecutionPolicy Bypass -File tools\run_all_modules.ps1 -Packages motor_tb6612

# 3) 构建（Gowin 综合 + 布局布线）
powershell -ExecutionPolicy Bypass -File tools\build.ps1
#    等价于手写：
#    & "D:\Apps\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build.tcl

# 4) 改了界面文案后重新生成字库
python tools\gen_font_rom.py

# 5) 烧录（上板第一步；详见 doc\上板指南_v1.md 第一节）
powershell -ExecutionPolicy Bypass -File tools\program_flash.ps1 -Action Cables    # 列下载器
powershell -ExecutionPolicy Bypass -File tools\program_flash.ps1 -Action Sram      # ★ 先只下 SRAM
powershell -ExecutionPolicy Bypass -File tools\program_flash.ps1 -Action Program   # 验证无误再固化
```

✅ **最近一次完整构建**（**2026-10-16 20:51**，含本轮整改 + 队友 HT-10A 分支合并 + 其后的全项目审查修复）：**ERROR 0**、比特流 `impl\pnr\falcons.fs`（**34.2 MB**）、
Logic 9112（7%）/ BSRAM 9 块 / DSP 43；仿真 **6 个 tb / 203 项全 PASS**。
该位流**晚于最后一次 RTL 改动** ✓ 可以直接烧。三份历史位流留作对照：
`impl\pnr\falcons_before_fixreview_20261016_2013.fs`（审查修复前，20:13 综合）、
`impl\pnr\falcons_before_ht10a_merge_20261016_1836.fs`（合并前）、
`impl\pnr\falcons_before_cdcfix_20261016_0221.fs`（复位修正前）。
上板步骤、按键/LED/遥控映射、**启动门禁行为**、IMU 现象判据、已知限制见 **`doc\上板指南_v1.md`**；
**HT-10A 接线表、标定参数与实物验收清单见 `doc\HT10A接线与验收.md`**（以那份为准）。

---

## 四、vendor 的来源与维护方式

**2026-10-15 起，`vendor\` 里的模块由本工程直接维护**，不再有"从外部模块库同步"这件事。

| 项 | 值 |
| --- | --- |
| 来源 | `git@github.com:HITSZ-FPGA-FALCONS/verilog-reusable-modules.git` |
| 最后同步的 commit | **`c9b8025`**（`c9b8025589dcde7c3d5f261ec28d7358d08d9953`，2026-10-14） |
| 内容 | 10 个包 / 144 个文件 / 2.8 MB（包结构完整：`rtl` + `sim` + `constraints` + 各包 README） |
| 维护方式 | 直接改、直接提交；**改完必跑 `sim\run_sim.ps1`** |

**为什么改成直接维护**：这批模块只有本工程一个消费者。此时"库"的复用属性不存在，而原先的同步机制
（`VENDOR.lock` + `deps\vendor_manifest.sha256` + `tools\sync_vendor.ps1` + `tools\check_vendor.ps1`
+ 六步升级手册）的成本大于收益 —— 这 5 个文件已于 2026-10-15 删除（需要时可 `git checkout <提交> -- <路径>` 恢复）。

**源库的去向**：`D:\Programs\FPGA Projects\verilog-reusable-modules` **原地存档、只读** ——
不再同步、不再维护，但保留模块的开发历史，以备将来复用或考古。本工程的构建、仿真、综合**只读 `vendor\`**。

> 改模块的具体纪律见 **`vendor\ORIGIN.md`**。

---

## 五、vendor 内容概览

已就位的 10 个包：`motor_tb6612`、`imu660ra`、`gyro_tilt_comp`、`yaw_integrate`、`uart`、`sync_fifo`、
`hdmi_1024x600`、`rc_pwm_receiver`、`m10_radar`、`ov5640_capture`。

**当前编译进工程 8 个**（见 `deps\modules.list`）：`motor_tb6612`、`imu660ra`、`gyro_tilt_comp`、
`yaw_integrate`、`uart`、`sync_fifo`、`hdmi_1024x600`、`rc_pwm_receiver`；
其余 2 个（`m10_radar`、`ov5640_capture`）已就位、待接入。

### 模块自带测试的最近一次结果（2026-10-15 实测）

`tools\run_all_modules.ps1` 跑 12 个入口（每个独立进程）：**PASS 11 / COMPILE-ONLY 1 / FAIL 0 / SKIP 0**。

`gyro_tilt_comp`（单元 9 用例 + 与 imu660ra 的集成测试）/ `imu660ra` / `motor_tb6612`（5 个测试平台）/
`yaw_integrate`（11 用例）/ `uart` / `sync_fifo` / `hdmi_1024x600` / `ov5640_capture` / `m10_radar` 均 PASS；
`m10_radar/run_tests.do` 为 COMPILE-ONLY；`rc_pwm_receiver` PASS（包自带 `run_test.ps1` 需要 iverilog，
本机未装，脚本**自动回退**到 ModelSim 的 `.do` 入口）。

> 日志在 `logs\`（已忽略）。ModelSim 在管道下输出的中文会变成 `?`，判定靠英文 `PASS`/`FAIL`。

---
## 六、硬件与工具环境

| 项 | 值 |
| --- | --- |
| 板卡 | 高云 **GW5AT-138B**（ACG720 开发板），工程器件 `GW5AT-LV138PG484AC1/I0` |
| 系统时钟 | 50 MHz（管脚 **Y18**） |
| Gowin EDA | `D:\Apps\Gowin\Gowin_V1.9.12.03_x64\`（命令行入口 `IDE\bin\gw_sh.exe`） |
| ModelSim | `D:\Apps\modeltech64_10.6d\win64`（`vlib` / `vlog` / `vsim`） |
| 源库（已归档，不参与构建） | `D:\Programs\FPGA Projects\verilog-reusable-modules` —— 只读存档，本工程只读 `vendor\` |
| 串口 | 板载 USB 转串口：TX=**M15**，RX=**J21**，115200 8N1 |
| 板载按键 | S0=**F15**（复位）、S1=**A20**（接管档内停机锁存）、S2=**B20**（计数+堵转+航向角清零）、S3=**A21**（备用翻页）、S4=**B21**（急停锁存；★ 实物是否为板上复位键待核，见 `doc\上板指南_v1.md`） |
| IMU 接线 | **J2** 排针：SCL→**A14**、SDA→**A13**、SA0/SDO→**A16**、CS→**A15** |
| 遥控 | **J6**：**D19 E19 E18 F18 F20 F19**（**6 路接口，实际用 5 路**：2 摇杆 + 确认 + select + 模式拨杆；**J6.6 / F19（bit5）闲置**，已由 `SIG_VALID_MASK = 6'b011111` 排除在"信号有效"判定外；**J6.4 / F18 是 select**）。★ **接线真源 = `doc\HT10A接线与验收.md` §2**（接收机 CH ≠ J6 槽位；`CH_*` 是**零起始的槽位索引**） |
| 雷达 | RX = **C14**（J3.3）—— ★ **绝不能接 A15**，那是 J2.6 IMU CS 且与 J1 同网络 |
| 电机驱动 | WHEELTEC **D24A**（2×TB6612FNG），经主板 **J4/J5** 排线直连 |
| 电机 | MG513P30_12V ×4（减速比 30；堵转 3.2 A；空载 366 rpm） |
| 编码器 | GMR **512 PPR**（由实测反推，见下），装**电机轴**；4 倍频 → **61440 计数/车轮转** |
| 车体几何 | 轮半径 19.9 mm（轮宽 26 mm）、轮距 **158.6 mm**、轴距 170 mm |
| 本车极性标定 | `INV_DIR_VEC = 4'b0000`、`ENC_INV_DIR_VEC = 4'b1010`（左侧不动、右侧反相） |

> **编码器线数判定**：实测手转车轮一圈 ≈ **61657** 计数。512 线理论值 61440（偏差 0.35%）、
> 500 线理论值 60000（偏差 2.76%）→ 判定 **512 线**。因此速度换算系数是 **10.24 计数/rpm·10ms**，
> 而不是 `car_fpga` 早期使用的 10（那里隐含 60000 计数/转的假设，**2.4% 系统偏差**）。

> 引脚权威依据是 `class\car_fpga\ACG720_引脚接线手册_RevI.pdf`（J1~J9 外设接口）。
> 抄约束前必读；换接线方式必须同步重写约束。

---

## 七、分步计划（与功能设计文档的 L0–L6 对齐）

> 阶段划分采用 `doc\使用FPGA实现的差速小车功能设计_ai优化版.md` §9 的 **L0–L6**；
> 完整验收判据、依赖与待确认项见 **`doc\设计落实待办.md`**。

| 阶段 | 目标与判据 | 本工程状态 |
| --- | --- | --- |
| **L0 底盘闭环** | 4×`speed_pid` + `motor_tb6612` 整定；目标 300 rpm 稳态误差 ≤5%，堵转与急停有效 | **代码已就位**（`chassis_ctrl` + `rtl\motor\speed_pid.v`），仿真通过；**待上板整定** |
| L1 估计层 | `odom` + 编码器互补 + 侧滑标定；静止 60 s 漂移 <1°、直行 5 m 横向偏差 <10 cm | 未开始；几何参数已到位（轮距/轮径/减速比/线数） |
| L2 显示与交互 | `hdmi_1024x600` + UI；四页可切换、无撕裂、数据与串口一致 | **基本完成**：中文四页 + 状态栏 + 数据页 5 行实时数值（航向角 / 车轮转速 ×4 / 角速度 / 中心速度 / 航位）；页 0/1/3 为占位；**导航页（§33）整体未实现** |
| L3 感知 | ToF 急停 + 雷达建图；30 cm 内 100 ms 刹停、雷达图 ≥10 Hz | 雷达模块已内嵌、**未接入**（RX=C14）；ToF 模块库里没有，需新建；急停通路已在顶层预留 |
| L4 遥控接管 | `rc_input`（遥控输入 + 模式判定，无独立 rc_rx）；接管/让出无扰切换、丢信号 100 ms 内停车 | **已实现**：**配置层 `ht10a_input`**（HT-10A 槽位/标定）+ 通用适配层 `rc_input` + **解锁守卫 `rc_drive_guard`**；6 路 `rc_pwm_capture` 实例化（实际用 5 路）+ 两档模式 + 丢信号保护 + **信号有效性掩码**（bit5/J6.6 闲置不拖死整车使能）（`rc_input`） |
| L5 导航 | 路径点跟踪 + 局部绕障；3~5 个点误差 <30 cm | 未开始；底盘侧接口已预留（`i_auto_valid/center/diff`） |
| L6 预留 | RK3588 通路（USB2.0 / 以太网） | **引脚未分配**：J1 的 11 个 GPIO 已被遥控(6)+ToF(4)+雷达(1)占满 → 需走 USB2.0/以太网（待 Q9 选型） |

例化 `imu_top` 时**必须显式写 `.SCK_HALF_DIV(25)`**（源码默认值 3 = 8.33 MHz，实测会导致初始化失败）。

---

## 八、已知坑速查

| 坑 | 一句话结论 |
| --- | --- |
| 单元测试过、集成不通 | 跨模块的握手/时序契约必须用**集成测试**覆盖（曾有 `o_data_valid` vs `o_acc_valid` 接错，单元测试完全没抓到） |
| ModelSim `-c` 模式 | tb 里的 `$finish` 会**终结整个 vsim 进程** → 每个测试开独立进程 |
| 只给 `.cst` 不给 `.sdc` | 工具按默认 100 MHz 估算 → 假时序违规，两个约束必须同时给 |
| 引脚"能用"≠"正确" | 旧约束与飞线接法自洽，换成排线直连后 21 根信号**整排错位** |
| 堵转判据 | 不能用"转速为 0"或单窗口阈值 → 用**净位移**判据 |
| 中文文件读取 | PS 5.1 的 `Get-Content` 默认按 GBK 解码无 BOM UTF-8，**静默**给出错误结果 |
| `.ps1` 的 BOM | 改完必查前 3 字节 `EF BB BF`（BOM 归 BOM、行尾归行尾，两者独立） |
| 行尾策略 | 必须随仓库走（`.gitattributes`），不能依赖各机器 `core.autocrlf` |
| `git archive` 而非拖拽 | 拖拽会把 `sim\work\`、`transcript`、`*.qdb` 一起搬进仓库 |
| 仿真通过 ≠ 能综合 | 位宽越界 ModelSim 容忍、Gowin 报 `EX3784` 并把模块降级成黑盒 |
| 约束里引用 PLL 派生时钟 | `set_false_path -to [get_clocks {pixel_clk}]` 报 `TA2004` 并**让综合失败**（该时钟在约束解析阶段还不存在） |
| PowerShell 数组拼换行 | `@( @('a','b'), @('c', 'x' + "``n" + 'y') )` 里第二行会**丢** → 用 here-string |
| `git commit -m` 带引号的多行文本 | 含 `"` 的多行 here-string 放进 `-m` 会被 PowerShell **拆成多个参数** → git 报一串 `pathspec … did not match any file(s)`，看着像路径问题、其实是参数被拆了。**改用 `git commit -F <文件>`**（消息写 UTF-8 无 BOM 文件）。同一个坑在 `git commit -m`、`-File` 传参、`"a,b"` 上共踩过 4 次 |
| 批量替换改坏 `.ps1` 的 BOM | 写回时用了无 BOM 的 `UTF8Encoding($false)` → PowerShell 按 GBK 解中文注释，报一串 `Unexpected token` / `Missing closing '}'`，**看起来像脚本被改坏了，其实只少了 3 个字节**。改 `.ps1` 要点：`ReadAllBytes` 判原 BOM，写回保持一致 |
| 仿真清单漏依赖闭包 | `vlog` 逐文件独立编译、**不做模块解析** → 少列依赖时编译全过，`vsim` 加载才报 `Module 'xxx' is not defined` / `Error loading design`。补依赖要**补到叶子** |
| `i_force` 是电平不是脉冲 | `speed_report` 的 `i_force` 在 `S_IDLE` 里是**或**条件：保持高电平会**连续上报**。按"单周期"的旧注释给长脉冲 → 每次多留一行残留，检查到的永远是上一行（行长 39/42/42/42）。**注释与实现不符要立刻改注释** |
| 文档里的检查项数量 | 曾出现 `num_text` 标 22 项、主 README 标 26 项、实测 28 项 —— **数字必须从命令输出统计**，不能凭印象写 |

完整版（按 6 类归，含现象与处理）见 `doc\工作记录.md` 第十节。

---

## 九、实现进度与未完成项

### 已完成（2026-10-15）

- **工程骨架**：`vendor\` 模块库（10 包 / 144 文件）+ `deps\modules.list` + 工具脚本 + Tcl 侧源文件派生
  （原先的同步机制 —— `VENDOR.lock` / `vendor_manifest.sha256` / `sync_vendor.ps1` / `check_vendor.ps1` /
  `模块库升级流程.md` —— 已于 **2026-10-15 删除**，改为 vendor 直接维护，见第四节）
- **应用层 RTL**：`falcons_top`（顶层）、`chassis_ctrl`（差速分解 + 4×增量式 PID）、`rc_input`（遥控适配层）、
  `ht10a_input`（HT-10A 配置层）、`rc_drive_guard`（手动解锁守卫）、
  `app_key`（按键去抖）、`ui_render` + `num_text` + `dyn_text_ram`（中文界面与实时数值）；
  从 `car_fpga` 搬入 `speed_pid` / `reset_sync` / `uart_cmd_rx` / `speed_report`
- **遥控控制**（配置层 `rtl\rc\ht10a_input.v` + 适配层 `rtl\rc\rc_input.v`）：库内**单路** `rc_pwm_capture` 实例化 **×6**（**实际用 5 路**，**J6.6 / bit5 闲置**）；
  转向/油门归一化到 ±1000（中位 1500 µs、死区 ±25 µs）；**方向取反参数** `STEER_REVERSE` / `THROTTLE_REVERSE`（顶层 `u_rc` 显式给 0）；
  模式拨杆**两档**（手动接管 / 屏幕光标）；**2 个按钮**（确认 / select）边沿脉冲；
  丢信号 **100 ms** 检测 → `run_en` 归零 + 状态栏提示；摇杆拨动切菜单 + **确认才切页**（两段式）；
  ★ **事件有效性与档位门控的单一真源 = `rc_input` 的 `cursor_ready`**（信号有效 + 模式通道连续 2 帧已确认 + 光标档），**顶层不再重复门控**；
  ★ **`SIG_VALID_MASK = 6'b011111`**（2026-10-16 新增）：信号有效性只由**参与判定的 bit0~bit4** 决定 ——
  否则 bit5（J6.6）闲置悬空时 `timed_out` 恒 1，会把整车使能拖死（车永远不动，且只报"无信号"）。
  **已验证**：仿真 **83 + 32 项** PASS、综合通过。**待实测**的是**行程 / 中位 / 极性**（槽位角色已由需求方确认，真源 `doc\HT10A接线与验收.md` §2）；
  **导航页（准心/辅助菜单）未实现**
- **启动门禁 / 解锁守卫**（2026-10-15 加、2026-10-16 改为模式驱动、当晚搬进 `rtl\rc\rc_drive_guard.v`）：`ARM_DEADZONE = 100`，
  **切到"手动接管"档**后**油门回到中位即解锁、解锁一次后保持**（切档 / 丢信号 / 急停 / S1 停机都会撤销解锁；不再需要先按 S1）；
  **S1 变为"停机锁存"**；状态栏提示 `请回中位`
- **IMU 链路**：`imu660ra` → `gyro_tilt_comp` → `yaw_integrate` → 数据页航向角（三个包已进 `modules.list`）
- **中文点阵字库**：**227 字 / 76 条文案**，由 `tools\gen_font_rom.py` 生成并**内嵌**（不用 `$readmemh`）。
  2026-10-16 新增 `ST_CURSOR:光标档`、把 `HINT_MOVE_MAP:摇杆移图` 改名为 `HINT_MOVE_CURSOR:摇杆移准心`
  （旧名与"地图不动、准心动"的新需求**语义相反**）→ 新增字形"光/档/准"，字符数 224 → 227
- **板级约束**：`constr\falcons.cst`（**60 个 `IO_LOC`**，逐针抄手册；雷达 RX=C14）+ `falcons.sdc`
- **仿真**：6 个自检测试台 / **203 项检查全 PASS**（`ht10a_input` 83 / `rc_input` 32 / `chassis_ctrl` 23 / `num_text` 39 / `ui_render` 22 / `speed_report` 4）
- **综合与布局布线**：ERROR 0、比特流 **34.2 MB**（**2026-10-16 20:51:52**，SHA256 `4FA40D37…3493`，含 PID 初值 0/64/0、两档模式、两段式切页、数据页 9 字段、配置层 `ht10a_input` + 解锁守卫 `rc_drive_guard`）；
  **SETUP 路径表 0 条负 slack**（最差 **+0.032 ns**）、REMOVAL 0 条（+0.269 ns）；负 slack 全在跨时钟域同步器与上电复位路径上
  （HOLD 13 条 −1.026 ns / RECOVERY 8 条 −5.983 ns，详见 `doc\工作记录.md` §21.4）。
  ★ 报告头部 `Numbers of Setup Violated Endpoints` 与路径表**口径不一致，本轮未查清、待全量 `report_timing`** —— **不要写成"0 违规端点"**
- **位流与 RTL 的时间关系（上板前必看）**：现有位流 `impl\pnr\falcons.fs` 综合于 **2026-10-16 20:51:52**，
  **晚于本轮全部 RTL 改动**（含下午的需求原文整改与晚间的 HT-10A 分支合并）→ **可以直接烧**；
  时间关系由 `tools\program_flash.ps1` 打印，上板前顺手核一眼。
  **历史位流备查**（`impl\` 不入库，仅作对照）：`impl\pnr\falcons_before_fixreview_20261016_2013.fs`（审查修复前，20:13 综合）、
  `impl\pnr\falcons_before_ht10a_merge_20261016_1836.fs`（合并前，18:36 综合）、
  `impl\pnr\falcons_before_cdcfix_20261016_0221.fs`（复位跨域修正前，02:21 综合）—— 这三份**不含**最新改动，别误烧
- **上板前补全 A/B 六项**（2026-10-15）：启动门禁+油门死区、CSV 12 列、堵转启动宽限 50、`program_flash.ps1` 迁入、
  遥控方向取反、切页限定光标档；并**消除了一条本轮引入的 CDC 深组合路径**（−2.473 → **−0.998 ns**；
  其中 **−0.998 ns 是"跨域源打拍那一轮"的中间值**，**当前** HOLD 最差为 **−1.026 ns**，见 20:51:52 报告）
- **文档**：功能设计（权威）、工作记录、上板指南、参数总表与待定清单、模块库接口速查、设计落实待办

### 尚未完成

1. ~~CSV 上报未接线~~ **已完成**：`speed_report` 已在顶层例化（`u_report`），串口按 **50 Hz** 输出 **12 列** CSV：
   `tgt,act,duty,kp,ki,kd,flags,err,steer,throttle,mode,sig_ok`（它自带 FIFO + UART TX，故不必再例化 `uart_byte_tx`；
   后 4 列为 2026-10-15 新增，用于核对"杆位 → 归一化值"）
2. **串口命令只接了一部分**：`P/I/D`、`X`、`Z` 已生效；`T/1-4`（目标速度）与 `E/U`（开环/闭环）未接 ——
   本轮速度给定只来自遥控
3. ~~引脚逐针复验未做~~ **已完成**：`tools\check_pin_constraints.py` 已迁入并**扩展**（覆盖手册 J2/J4/J5/J6/J7 五张表），**35 根信号逐针一致**；并借此发现、修掉了顶层 ToF 引脚语义错误（详见 `doc\工作记录.md` 第十四节）
4. ~~`tools\` 还缺 1 个脚本~~ **已完成**（2026-10-15）：`program_flash.ps1` 已迁入（默认位流 `impl\pnr\falcons.fs`，用法见 `doc\上板指南_v1.md` 第一节）。`tools\` 仍缺 `plot_yaw.py`（串口采集+分析+绘图），见 `tools\README.md`
5. **速度环 PID 未整定**：初值由用户 2026-10-16 直接指定为 **`P=0 / I=64 / D=0`**（Q8 口径 → Kp=0、Ki=0.25、Kd=0，**纯积分控制**），
   需上板后按现象整定（Ki 比原值大 21 倍，注意积分饱和；串口 `P/I/D` 可在线改）
6. **尚未上板**：所有结论来自编译、仿真、综合与时序报告；`.fs` 生成但没烧过
7. **快照升级路径**：将来若要改用 git submodule，把 `vendor\verilog-reusable-modules` 这个挂载点
   就地换成 submodule 即可，应用层与脚本几乎不用改

### Git 状态

| 项 | 值 |
| --- | --- |
| 远程 | `https://github.com/HITSZ-FPGA-FALCONS/FALCONS-embodied-robot.git`（分支 `main`，已建立跟踪） |
| 首次提交 | `bb9ea07`（199 文件 / 72107 行：vendor 模块 144 + 工程文件 55） |
| 提交历史 | **不在这里维护完整列表**（每提交一次都会过期，是必然漂移的信息）—— 用 `git log --oneline` 看。关键节点：`bb9ea07` 首次提交 → `c2cddf4` vendor 转为本项目直接维护 → `7dba09e` 串口 CSV 上报 + 引脚逐针复验 → `57ff8ba` 上板前补全 A/B 六项 → `22969a8` 写入 PID 初值 0/64/0 → `7767b4f` 按需求原文与需求方裁定整改（含复位跨域修正）→ **`beb7e23` 合并队友的 HT-10A 分支并重新综合 → `43be57a` 全项目审查修复 + 重新综合 = 当前 `main`（ahead 1、未推送）** |
| 本地是否领先 | 用 `git status -sb` 看：行首出现 `[ahead N]` 就是有未推送的提交 |
| 未入库 | `impl\`（含 34.2 MB 比特流）、`logs\`、`work*\`、`transcript` —— 由 `.gitignore` 排除 |
| 推送注意 | 本机受限环境下 `git push` 会因 schannel `SEC_E_NO_CREDENTIALS` 失败，需放行网络/TLS 访问后重试（本次即如此） |

---

## 十、文档同步纪律（★ 每完成一项任务后必做）

**规则**：每完成一项任务（新功能 / 修缺陷 / 改参数 / 上板验证 / 升级快照）后，**必须同步更新下列文档**，
不允许"代码改了、文档还停在旧状态"。收尾时按下表逐项过一遍：

| 文档 | 记什么 | 什么时候必须动 |
| --- | --- | --- |
| `README.md` | 工程总览、**〇 当前状态一览**、进度与未完成项、Git 状态 | **每次都动**（状态一览与"尚未完成"必须与代码一致） |
| `doc\工作记录.md` | 做过什么、怎么验证的、踩了什么坑、为什么这么决策 | **每次都动**（新增阶段小节，或补进已有阶段） |
| `doc\参数总表与待定清单.md` | 每个参数的当前值、在哪个文件哪一行改、哪些待实测 | **参数一改就动**；发现"文档值与代码值不一致"必须立即修正 |
| `doc\设计落实待办.md` | L0–L6 阶段状态（□/◐/■）、待确认项、下一步可执行 | 状态变化时 |
| `doc\上板指南_v1.md` | 烧录、接线、按键/LED/遥控映射、现象判据、已知限制 | 顶层端口 / 引脚 / 按键语义变化时 |
| `doc\模块库接口速查.md` | 模块端口与参数 | 升级快照后，或重新通读时 |
| `doc\README.md` | 文档索引 | 新增 / 删除文档时 |
| `README.md`（各目录）／`sim\`、`tools\`、`rtl\`、`constr\` | 该目录的文件清单与用法 | 目录内容变化时 |

### 三条自查（防止"文档说了代码没做"或反之）

1. 文档里的**数字**（资源占用、时序 slack、仿真项数、包数量、计数换算）必须能在工具输出里找到出处 ——
   不允许凭印象写"大概"。
2. 文档里的**路径 / 文件名 / 参数名**必须真实存在；改过名的要全局搜一遍（`grep` 旧名应无结果）。
3. **同一事实只在一处定义**，其它地方引用它：参数值以 `doc\参数总表与待定清单.md` + 代码里的实际值为准，
   其它文档用链接而不是复制粘贴（复制出来的副本一定会漂移）。

### 收尾动作

> ⚠ **本地提交 ≠ 可以推送** —— `git push` 属外部副作用，需要**单独指令**才做。
> 同理：删除文件 / 目录、回退历史（`reset --hard`、`clean -fd`）、覆盖工作区外路径，
> **一律先列出"删/改什么、影响什么、怎么恢复"等确认**（见铁律 4）。

```powershell
# 1) 自查过时信息（示例：换成这次改动的关键词）
Select-String -Path README.md,doc\*.md -Pattern '旧数字|旧文件名|旧状态' -Encoding UTF8
# 2) 提交（提交信息写明改了哪些文档、为什么）
git add -A; git commit -m "docs: ..."
```

> 这条纪律是本工程与用户的明确约定：**任务完成的定义包含"文档已同步"**，不是只把代码跑通。