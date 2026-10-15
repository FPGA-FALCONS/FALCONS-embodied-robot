# FALCONS-embodied-robot

FPGA 四驱差速小车（具身机器人）的**主工程**。板卡：高云 **GW5AT-138B**（ACG720 开发板）。

> 远程仓库：**`https://github.com/HITSZ-FPGA-FALCONS/FALCONS-embodied-robot.git`**（分支 `main`）
> 与 `class\car_fpga` 的分工：`car_fpga` 是**试验场**（每条链路先在那里上板验证），
> 本工程是**正式项目** —— 采用"**模块库内嵌 vendor 快照**"，依赖被钉死在一份可复现的 commit 上。
> 方案依据：`class\模块库内嵌方案_反馈.md`；工作区切换背景：`class\工作区切换_交接单.md`。

> ★ **功能设计文档**：`doc\使用FPGA实现的差速小车功能设计_ai优化版.md`（652 行，2026-10-15 从
> `class\` 纳入）—— 需求与实现口径的**权威依据**：赛题与范围、J1~J9 外设接口总表、供电预算、
> 时钟域与复位策略、模块清单、开发路线与验收判据、待确认信息清单。
> 工程根的 `使用FPGA实现的差速小车功能设计.md`（37 行）是最初的功能清单 + UI 交互初稿，仅作历史留档。

> ★ **工作记录**：`doc\工作记录.md` —— 本轮做了什么、怎么验证的、踩了哪些坑、为什么这么决策，
> 连"哪些是实测数据、哪些是推断"也一并标明，便于回溯与交接。

---

## 〇、当前状态一览（2026-10-15）

| 项 | 状态 |
| --- | --- |
| 进度 | 骨架 + 中文四页界面（含实时数值）+ 遥控 6 路 + 四路速度闭环 + **IMU 偏航角链路**；ToF/雷达/导航为预留 |
| 仿真 | 4 个自检 tb / **70 项检查全 PASS**（`sim\run_sim.ps1` 一条命令） |
| 综合 | **ERROR 0**；比特流 `impl\pnr\falcons.fs`（34.1 MB，未入库） |
| 资源 | Logic 8068/138240（**6%**）、Register 3137（3%）、BSRAM **9**/340（3%）、DSP 41/298（14%） |
| 时序 | SETUP 1 条（−2.003 ns）+ HOLD 9 条，**全部为跨时钟域结构性现象**；真实逻辑路径最差 +0.105 ns |
| 内嵌快照 | 源库 `c9b8025` / 144 文件 / tree `9f40e862…`，体检逐文件一致 |
| Git | 首次提交 `bb9ea07`（199 文件 / 72107 行），当前 `main` 已推送到远程并建立跟踪 |
| 上板 | **尚未做过**（本轮所有结论来自编译、仿真、综合与时序报告） |

---

## 一、三条铁律

| # | 规则 | 为什么 |
| --- | --- | --- |
| 1 | **模块的修改必须先在源库 `verilog-reusable-modules` 提交，再同步进本工程。** 本工程内不直接改 `vendor\` 里的模块 | 在 vendor 里直接改 = 制造第二份真源：下次 `sync_vendor` 会静默覆盖，而且没人会发现 |
| 2 | **未经用户明确命令，不动源库**（读可以，写/提交/推送不行） | 源库是队友共用的；`sync_vendor.ps1` 只用 `git archive` / `git log`，本身是只读的 |
| 3 | **`.ps1` 必须是 UTF-8 with BOM**；读中文文件必须显式指定 UTF-8；改顶层后必须跑一次综合；引脚约束逐针抄硬件手册 | 分别是：PS 5.1 按 GBK 读会乱码+语法错、`Get-Content` 会静默数错行、仿真过 ≠ 能综合、引脚错位看起来像"模块坏了" |

---

## 二、目录结构

```text
FALCONS-embodied-robot\
├─ vendor\verilog-reusable-modules\   ★ 内嵌快照（10 个包 / 144 个文件 / 2.8 MB，无 .git）
│    └─ modules\<包>\{rtl, sim, constraints, examples, README.md, ...}
├─ deps\
│    ├─ modules.list                  ★ 本工程【实际引用】的包 = 依赖闭包（唯一定义处）
│    └─ vendor_manifest.sha256        逐文件 SHA256（144 行，体检时逐条比对）
├─ VENDOR.lock                        ★ 锁定的源 URL + commit + tree SHA256 + 包清单
├─ tools\
│    ├─ lib_paths.ps1                 所有路径的唯一定义处（被其它脚本 dot-source）
│    ├─ sync_vendor.ps1               升级快照（只读源库，带差异报告）
│    ├─ check_vendor.ps1              快照体检（构建前跑）
│    ├─ run_all_modules.ps1           跑库自带测试（库自带测试当门槛）
│    ├─ build.ps1                     先体检再综合的一键构建（调 gw_sh）
│    ├─ gen_font_rom.py               中文字库生成器（字表 → 点阵 ROM，数据内嵌）
│    └─ font_chars.txt                UI 文案字表（73 条）
├─ scripts\vendor_sources.tcl         Tcl 侧源文件清单（从 modules.list 派生）
├─ rtl\
│    ├─ top\falcons_top.v             整车顶层（只做例化与连线）
│    ├─ rc\rc_input.v                 遥控 6 路输入 + 模式仲裁
│    ├─ motor\chassis_ctrl.v          差速分解 + 4×增量式 PID；speed_pid.v（搬自 car_fpga）
│    ├─ ui\ui_render.v                中文界面渲染；num_text.v 数值格式化；dyn_text_ram.v 跨域字符缓冲
│    ├─ debug\uart_cmd_rx.v           串口命令解析（搬自 car_fpga）；speed_report.v（待接线）
│    └─ common\                       按键去抖、复位同步、字库与字符串 ROM
├─ constr\falcons.cst / .sdc          板级与电气约束（成对给出；44 个 IO_LOC）
├─ sim\                               4 个自检测试台 + run_sim.ps1 批量入口
├─ doc\                               功能设计、工作记录、上板指南、参数总表、接口速查、升级流程、待办
├─ build.tcl                          主构建脚本（产物 impl\pnr\falcons.fs）
└─ logs\                              脚本日志（.gitignore 已忽略）
```

---

## 三、快速开始

```powershell
cd "D:\Programs\FPGA Projects\FALCONS-embodied-robot"

# 0) 环境体检：快照是否被本地改动 / 落后源库多少提交 / 工程引用的包有没有变化
powershell -File tools\check_vendor.ps1

# 1) 跑内嵌库各包自带的测试（同步完之后强烈建议）
powershell -File tools\run_all_modules.ps1 -ReferencedOnly

# 2) 跑本工程的仿真（4 个自检 tb，每个开独立 vsim 进程）
powershell -File sim\run_sim.ps1

# 3) 升级模块库快照（只在需要时；只读源库，会打印"变了哪些文件"的差异报告）
powershell -File tools\sync_vendor.ps1
#    ★ 完整六步升级流程：doc\模块库升级流程.md

# 4) 构建（先体检，再调 Gowin 综合 + 布局布线）
powershell -File tools\build.ps1
#    等价于手写：
#    & "D:\Apps\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build.tcl

# 5) 改了界面文案后重新生成字库
python tools\gen_font_rom.py
```

✅ **最近一次构建**（含 IMU 链路）：**ERROR 0**、比特流 `impl\pnr\falcons.fs`（34.1 MB）、
Logic 6% / BSRAM 9 块 / DSP 41。上板步骤、按键/LED/遥控映射、IMU 现象判据、已知限制见 **`doc\上板指南_v1.md`**。

---

## 四、依赖锁定机制（三个文件的关系）

| 文件 | 谁写 | 作用 |
| --- | --- | --- |
| `VENDOR.lock` | `sync_vendor.ps1` 自动生成 | 记录**源 URL + commit + tree SHA256 + 包清单**，是"这份快照到底是什么"的唯一权威 |
| `deps\vendor_manifest.sha256` | `sync_vendor.ps1` 自动生成 | 144 行逐文件哈希，`check_vendor.ps1` 用它判断"vendor 是否被本地改过" |
| `deps\modules.list` | **人工维护** | 本工程实际引用哪些包（依赖闭包）。构建源文件、体检高亮范围都从它派生 |

`check_vendor.ps1` 报告三件事：

1. 内嵌目录是否被本地改动过（逐文件 SHA256 比对，**只比对被跟踪的源文件**；
   跑仿真产生的 `work\`、`transcript`、`build\` 属正常产物，会被识别为"manifest 之外的文件"而不误报）；
2. 相对源库落后多少个提交（源库不可达时跳过，不算失败），并单独列出源库里标了 `⚠ BREAKING` 的提交；
3. **只高亮工程引用的包**是否有变化（不刷屏）—— 变化落在未引用的包里，不影响当前构建。

> 首次提交之后，"vendor 被本地改动"的保护才真正生效（未跟踪时那段逻辑会跳过）。

### 源库更新后，怎么把项目库一并更新？

完整六步操作手册（含判据表、异常处理、回退办法、实测记录）见 **`doc\模块库升级流程.md`**。命令速查：

```powershell
git -C <源库> fetch origin             # ① 让源库拿到新提交（写源库，需你明确授权）
powershell -File tools\check_vendor.ps1                     # ② 升级前体检：落后多少 / 有无 BREAKING
powershell -File tools\sync_vendor.ps1                      # ③ 同步 + ★ 差异报告（变了哪些文件）
powershell -File tools\check_vendor.ps1                     # ④ 复检：应"已是最新"且逐文件一致
powershell -File tools\run_all_modules.ps1 -ReferencedOnly  # ⑤ 库自带测试当门槛
powershell -File tools\build.ps1                            # ⑥ 适配应用层后构建
git add vendor deps VENDOR.lock rtl; git commit -m "..."     # 快照与记账【一起】提交
```

差异报告里带 `★` 的文件 = 本工程 `deps\modules.list` 引用的包 —— 这些才需要判断是否改 `rtl\`。
回退一步到位：`git checkout -- vendor VENDOR.lock deps/vendor_manifest.sha256`，或
`tools\sync_vendor.ps1 -Ref <旧commit>`（升级 = 换一份快照 + 换一份记账，不存在"半升级"状态）。

---

## 五、当前快照状态

| 项 | 值 |
| --- | --- |
| 源库 | `git@github.com:HITSZ-FPGA-FALCONS/verilog-reusable-modules.git` |
| 锁定 commit | **`c9b8025`**（`c9b8025589dcde7c3d5f261ec28d7358d08d9953`，2026-10-14，main） |
| 内容 | 10 个包 / **144 个文件** / 2.8 MB |
| manifest tree SHA256 | `9f40e862f7c8437ddd43c7752933131a1f547913a76680d771c04a612467f38a` |
| 导出方式 | `git archive --format=tar HEAD` → 解压到 `vendor\`（**不含 `.git`**） |

已内嵌的 10 个包：`motor_tb6612`、`imu660ra`、`gyro_tilt_comp`、`yaw_integrate`、`uart`、`sync_fifo`、
`rc_pwm_receiver`、`hdmi_1024x600`、`m10_radar`、`ov5640_capture`。

**本工程已引用 8 个**（见 `deps\modules.list`）：`motor_tb6612`、`imu660ra`、`gyro_tilt_comp`、
`yaw_integrate`、`uart`、`sync_fifo`、`hdmi_1024x600`、`rc_pwm_receiver`；
其余 2 个（`m10_radar`、`ov5640_capture`）是"已内嵌、按阶段接入"。

### 快照自检记录（2026-10-15 实测）

| 检查项 | 命令 | 结果 |
| --- | --- | --- |
| 快照完整性 | `tools\check_vendor.ps1` | 144 项逐文件比对与 `VENDOR.lock` **一致**；tree SHA256 相符 |
| 库自带测试 | `tools\run_all_modules.ps1` | **PASS 11 / COMPILE-ONLY 1 / FAIL 0 / SKIP 0**（12 个入口，每个独立进程） |
| 仿真产物不污染快照 | `git check-ignore` + 再体检 | 跑测试后 vendor 内新增 263 个产物文件，均被忽略，体检**不误报** |
| 行尾策略 | `git check-attr -a` | `.ps1 → crlf`，`.v/.md/.tcl → lf`（与源库同一套规则） |
| 脚本编码 | 逐文件查前 3 字节 | 6 个 `.ps1` 全部 `EF BB BF`；`.gitattributes` 无 BOM |

库自带测试明细：`gyro_tilt_comp`（单元 9 用例 + 与 imu660ra 的集成测试）/ `imu660ra` /
`motor_tb6612`（5 个测试平台）/ `yaw_integrate`（11 用例）/ `uart` / `sync_fifo` /
`hdmi_1024x600` / `ov5640_capture` / `m10_radar` 均 PASS；`m10_radar/run_tests.do` 为 COMPILE-ONLY；
`rc_pwm_receiver` PASS（包自带 `run_test.ps1` 需 iverilog，本机未装，脚本**自动回退**到 ModelSim `.do`）。

> 日志在 `logs\`（已忽略）。ModelSim 在管道下输出的中文会变成 `?`，判定靠英文 `PASS`/`FAIL`。

---

## 六、硬件与工具环境

| 项 | 值 |
| --- | --- |
| 板卡 | 高云 **GW5AT-138B**（ACG720 开发板），工程器件 `GW5AT-LV138PG484AC1/I0` |
| 系统时钟 | 50 MHz（管脚 **Y18**） |
| Gowin EDA | `D:\Apps\Gowin\Gowin_V1.9.12.03_x64\`（命令行入口 `IDE\bin\gw_sh.exe`） |
| ModelSim | `D:\Apps\modeltech64_10.6d\win64`（`vlib` / `vlog` / `vsim`） |
| 源库（只读） | `D:\Programs\FPGA Projects\verilog-reusable-modules`（可用 `-LibRoot` 或 `FALCONS_LIB_ROOT` 覆盖） |
| 串口 | 板载 USB 转串口：TX=**M15**，RX=**J21**，115200 8N1 |
| 板载按键 | S0=**F15**（复位）、S1=**A20**（启动/停止）、S2=**B20**（计数+航向角清零）、S3=**A21**（切页）、S4=**B21**（急停锁存） |
| IMU 接线 | **J2** 排针：SCL→**A14**、SDA→**A13**、SA0/SDO→**A16**、CS→**A15** |
| 遥控 | **J6** 整排：**D19 E19 E18 F18 F20 F19**（6 路单路捕获实例化） |
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
| L2 显示与交互 | `hdmi_1024x600` + UI；四页可切换、无撕裂、数据与串口一致 | **基本完成**：中文四页 + 状态栏 + 数据页实时数值（航向角/目标轮速/四路实际轮速）；页 0/1/3 为占位 |
| L3 感知 | ToF 急停 + 雷达建图；30 cm 内 100 ms 刹停、雷达图 ≥10 Hz | 雷达模块已内嵌、**未接入**（RX=C14）；ToF 模块库里没有，需新建；急停通路已在顶层预留 |
| L4 遥控接管 | `rc_rx` + 模式仲裁；接管/让出无扰切换、丢信号 200 ms 内停车 | **已实现**：6 路 `rc_pwm_capture` 实例化 + 三档模式 + 丢信号保护（`rc_input`） |
| L5 导航 | 路径点跟踪 + 局部绕障；3~5 个点误差 <30 cm | 未开始；底盘侧接口已预留（`i_auto_valid/center/diff`） |
| L6 预留 | RK3588 通路（串口） | 引脚下已分配（J7），RTL 为空接口 |

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

完整版（按 6 类归，含现象与处理）见 `doc\工作记录.md` 第十节。

---

## 九、实现进度与未完成项

### 已完成（2026-10-15）

- **工程骨架与依赖锁定**：内嵌快照 + `VENDOR.lock` + `deps\` 记账 + 5 个工具脚本 + Tcl 侧源文件派生
- **应用层 RTL**：`falcons_top`（顶层）、`chassis_ctrl`（差速分解 + 4×增量式 PID）、`rc_input`（6 路遥控）、
  `app_key`（按键去抖）、`ui_render` + `num_text` + `dyn_text_ram`（中文界面与实时数值）；
  从 `car_fpga` 搬入 `speed_pid` / `reset_sync` / `uart_cmd_rx` / `speed_report`
- **IMU 链路**：`imu660ra` → `gyro_tilt_comp` → `yaw_integrate` → 数据页航向角（三个包已进 `modules.list`）
- **中文点阵字库**：220 字 / 73 条文案，由 `tools\gen_font_rom.py` 生成并**内嵌**（不用 `$readmemh`）
- **板级约束**：`constr\falcons.cst`（44 个 `IO_LOC`，逐针抄手册；雷达 RX=C14）+ `falcons.sdc`
- **仿真**：4 个自检测试台 / **70 项检查全 PASS**（`rc_input` 12 / `chassis_ctrl` 23 / `num_text` 26 / `ui_render` 9）
- **综合与布局布线**：ERROR 0、比特流 34.1 MB、时序仅剩 CDC 结构性违规
- **文档**：功能设计（权威）、工作记录、上板指南、参数总表与待定清单、模块库接口速查、升级流程、设计落实待办

### 尚未完成

1. **CSV 上报未接线**：`speed_report` 已搬入 `rtl\debug\`，但 `o_uart_tx` 仍是恒定高电平 —— 串口目前**只收不发**
2. **串口命令只接了一部分**：`P/I/D`、`X`、`Z` 已生效；`T/1-4`（目标速度）与 `E/U`（开环/闭环）未接 ——
   本轮速度给定只来自遥控
3. **引脚逐针复验未做**：`tools\check_pin_constraints.py` 尚未从 `car_fpga` 迁入；当前 `.cst` 是
   "抄自已复核过的旧值 + 手册表格"，还没用脚本对手册 PDF 原文逐针比对过
4. **`tools\` 还缺 2 个脚本**：`check_pin_constraints.py`、`program_flash.ps1`（见 `tools\README.md`）
5. **速度环 PID 未整定**：用的是 `car_fpga` 的 Q8 初值（26/3/0），需上板后按现象整定
6. **尚未上板**：所有结论来自编译、仿真、综合与时序报告；`.fs` 生成但没烧过
7. **快照升级路径**：将来若要改用 git submodule，把 `vendor\verilog-reusable-modules` 这个挂载点
   就地换成 submodule 即可，应用层与脚本几乎不用改

### Git 状态

| 项 | 值 |
| --- | --- |
| 远程 | `https://github.com/HITSZ-FPGA-FALCONS/FALCONS-embodied-robot.git`（分支 `main`，已建立跟踪） |
| 首次提交 | `bb9ea07`（199 文件 / 72107 行：内嵌快照 144 + 工程文件 55） |
| 未入库 | `impl\`（含 34.1 MB 比特流）、`logs\`、`work*\`、`transcript` —— 由 `.gitignore` 排除 |
