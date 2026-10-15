# FALCONS-embodied-robot

FPGA 四驱差速小车（具身机器人）的**主工程**。板卡：高云 **GW5AT-138B**（ACG720 开发板）。

> 与 `class\car_fpga` 的分工：`car_fpga` 是**试验场**（每条链路先在那里上板验证），
> 本工程是**正式项目** —— 采用"**模块库内嵌 vendor 快照**"，依赖被钉死在一份可复现的 commit 上。
> 方案依据：`class\模块库内嵌方案_反馈.md`；工作区切换背景：`class\工作区切换_交接单.md`。

> ★ **功能设计文档**：`doc\使用FPGA实现的差速小车功能设计_ai优化版.md`（652 行，2026-10-15 从
> `class\` 纳入）—— 需求与实现口径的**权威依据**：赛题与范围、J1~J9 外设接口总表、供电预算、
> 时钟域与复位策略、模块清单（已有/新建）、开发路线与验收判据、待确认信息清单。
> 工程根的 `使用FPGA实现的差速小车功能设计.md`（37 行）是最初的功能清单 + UI 交互初稿，仅作历史留档；
> 早先那份 404 行的「ai优化版」已于同日删除（内容与 652 行版重叠且更简略）。
>
> ★ **本轮工作记录**：`doc\工作记录.md` —— 做了什么、怎么验证的、踩了哪些坑、为什么这么决策；
> 连"哪些是实测数据、哪些是推断"也一并标明，便于回溯。

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
├─ vendor\verilog-reusable-modules\   ★ 内嵌快照（10 个包 / 144 个文件，无 .git）
│    └─ modules\<包>\{rtl, sim, constraints, README.md, ...}
├─ deps\
│    ├─ modules.list                  ★ 本工程【实际引用】的包 = 依赖闭包（唯一定义处）
│    └─ vendor_manifest.sha256        逐文件 SHA256（144 行，体检时逐条比对）
├─ VENDOR.lock                        ★ 锁定的源 URL + commit + tree SHA256 + 包清单
├─ tools\
│    ├─ lib_paths.ps1                 所有路径的唯一定义处（被其它脚本 dot-source）
│    ├─ sync_vendor.ps1               升级快照（只读源库）
│    ├─ check_vendor.ps1              快照体检（构建前跑）
│    ├─ run_all_modules.ps1           跑库自带测试（库自带测试当门槛）
│    └─ build.ps1                     先体检再综合的一键构建（调 gw_sh）
├─ scripts\vendor_sources.tcl         Tcl 侧源文件清单（从 modules.list 派生）
├─ rtl\{common,motor,imu,debug,top}\  应用层 RTL（本工程自己写的）
├─ constr\                            板级约束 .cst + .sdc（成对给出）
├─ sim\                               跨模块集成仿真与测试平台
├─ doc\                               ★ 功能设计（权威口径）＋ 硬件标定记录、上板指南、引脚复核、模块库升级流程
├─ build.tcl                          主构建脚本（产物 impl\pnr\falcons.fs）
└─ logs\                              脚本日志（.gitignore 已忽略）
```

---

## 三、快速开始

```powershell
cd "D:\Programs\FPGA Projects\FALCONS-embodied-robot"

# 0) 环境体检：快照是否被本地改动 / 落后源库多少提交 / 工程引用的包有没有变化
powershell -File tools\check_vendor.ps1

# 1) 跑一遍内嵌库各包自带的测试（可选但推荐，尤其同步完之后）
powershell -File tools\run_all_modules.ps1 -ReferencedOnly

# 2) 升级模块库快照（只在需要时；只读源库，会打印"变了哪些文件"的差异报告）
powershell -File tools\sync_vendor.ps1
#    ★ 完整六步升级流程（判据 / 异常处理 / 回退 / 实测记录）：doc\模块库升级流程.md

# 3) 构建（先体检，再调 Gowin 综合 + 布局布线）
powershell -File tools\build.ps1
#    等价于手写：
#    & "D:\Apps\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build.tcl
```

✅ **当前已可构建**：`build.tcl` 从 `deps\modules.list` 派生 vendor 源文件、从 `rtl\` 递归收集应用层，顶层是 `rtl\top\falcons_top.v`。首次构建：**ERROR 0**、比特流 `impl\pnr\falcons.fs`、Logic 4% / BSRAM 5 块 / DSP 16。上板步骤、按键/LED/遥控映射、已知限制见 **`doc\上板指南_v1.md`**。

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

差异报告里带 `★` 的文件 = 本工程 `deps\modules.list` 引用的包 —— 这些才需要判断是否改 `rtl\`；
没有 `★` 的改动（未引用的包）可以直接忽略。回退同样一步到位：
`git checkout -- vendor VENDOR.lock deps/vendor_manifest.sha256`，或 `tools\sync_vendor.ps1 -Ref <旧commit>`
（升级 = 换一份快照 + 换一份记账，不存在"半升级"状态）。

---

## 五、当前快照状态

| 项 | 值 |
| --- | --- |
| 源库 | `git@github.com:HITSZ-FPGA-FALCONS/verilog-reusable-modules.git` |
| 锁定 commit | **`c9b8025`**（`c9b8025589dcde7c3d5f261ec28d7358d08d9953`，2026-10-14，main） |
| 内容 | 10 个包 / **144 个文件** / 2.8 MB |
| manifest tree SHA256 | `9f40e862f7c8437ddd43c7752933131a1f547913a76680d771c04a612467f38a` |
| 导出方式 | `git archive --format=tar HEAD` → 解压到 `vendor\`（**不含 `.git`**） |

已内嵌的 10 个包：`motor_tb6612`、`imu660ra`、`gyro_tilt_comp`、`yaw_integrate`、
`uart`、`sync_fifo`、`rc_pwm_receiver`、`m10_radar`、`hdmi_1024x600`、`ov5640_capture`。

本工程**已引用** 6 个（见 `deps\modules.list`）：`motor_tb6612`、`imu660ra`、
`gyro_tilt_comp`、`yaw_integrate`、`uart`、`sync_fifo`；其余 4 个是"已内嵌、待接入"。

### 骨架搭建时的实测记录（2026-10-15）

| 检查项 | 命令 | 结果 |
| --- | --- | --- |
| 快照完整性 | `tools\check_vendor.ps1` | 144 项逐文件比对与 `VENDOR.lock` **一致**；tree SHA256 相符；退出码 0 |
| 库自带测试 | `tools\run_all_modules.ps1` | **PASS 11 / COMPILE-ONLY 1 / FAIL 0 / SKIP 0** |
| 仿真产物不污染快照 | `git check-ignore` + 再跑一次体检 | 跑测试后 vendor 内新增 263 个产物文件，均被忽略，体检**不误报** |
| 行尾策略 | `git check-attr -a` | `.ps1 → crlf`，`.v/.md/.tcl → lf`（与源库同一套规则） |
| 脚本编码 | 逐文件查前 3 字节 | 5 个 `.ps1` 全部 `EF BB BF`（UTF-8 with BOM）；`.gitattributes` 无 BOM |

测试明细（12 个入口，每个都是独立进程）：

| 包 | 入口 | 结果 |
| --- | --- | --- |
| `gyro_tilt_comp` | `run_test.do`（单元 9 用例）/ `run_integration.do`（★ 与 imu660ra 集成） | PASS / PASS |
| `imu660ra` | `run_test.do` | PASS |
| `motor_tb6612` | `run_motor.ps1`（5 个测试平台） | PASS |
| `yaw_integrate` | `run_test.do`（11 用例） | PASS |
| `uart` / `sync_fifo` | `run_test.do` | PASS / PASS |
| `hdmi_1024x600` / `ov5640_capture` / `m10_radar` | `run_test.do` | PASS / PASS / PASS |
| `m10_radar` | `run_tests.do`（只编译不跑仿真） | COMPILE-ONLY |
| `rc_pwm_receiver` | `run_test.do`（17 组 / 502 断言） | PASS —— 包自带的 `run_test.ps1` 需要 iverilog，本机未装，脚本**自动回退**到 ModelSim 的 `.do` 入口 |

> 日志在 `logs\`（`.gitignore` 已忽略）。注意 ModelSim 在管道下输出的中文会变成 `?`，
> 判定靠英文 `PASS`/`FAIL` 关键字，读日志时留意。

---

## 六、硬件与工具环境

| 项 | 值 |
| --- | --- |
| 板卡 | 高云 **GW5AT-138B**（ACG720 开发板），工程器件 `GW5AT-LV138PG484AC1/I0` |
| 系统时钟 | 50 MHz（管脚 **Y18**） |
| Gowin EDA | `D:\Apps\Gowin\Gowin_V1.9.12.03_x64\`（命令行入口 `IDE\bin\gw_sh.exe`） |
| ModelSim | `D:\Apps\modeltech64_10.6d\win64`（`vlib` / `vlog` / `vsim`） |
| 源库（只读） | `D:\Programs\FPGA Projects\verilog-reusable-modules`（可用 `-LibRoot` 或环境变量 `FALCONS_LIB_ROOT` 覆盖） |
| 串口 | 板载 USB 转串口：TX=**M15**，RX=**J21**，115200 8N1 |
| 板载按键 | S0=**F15**（复位）、S1=**A20**、S2=**B20**、S3=**A21**、S4=**B21** |
| IMU 接线 | **J2** 排针：SCL→**A14**、SDA→**A13**、SA0/SDO→**A16**、CS→**A15** |
| 电机驱动 | WHEELTEC **D24A**（2×TB6612FNG，四路带稳压），经主板 **J4/J5** 排线直连 |
| 电机 | MG513P30_12V ×4（减速比 30；堵转 3.2 A；空载 366 rpm） |
| 编码器 | GMR 500 PPR，装在**电机轴**（4 倍频 → 约 60000 计数/车轮转；实测手转一圈 ≈ **61657**） |
| 本车极性标定 | `INV_DIR_VEC = 4'b0000`、`ENC_INV_DIR_VEC = 4'b1010`（左侧不动、右侧反相） |

> 引脚权威依据是 `class\car_fpga\ACG720_引脚接线手册_RevI.pdf`（J1~J9 外设接口）。
> 抄约束前必读；换接线方式必须同步重写约束。

---

## 七、分步计划（与功能设计文档的 L0–L6 对齐）

> 阶段划分采用 `doc\使用FPGA实现的差速小车功能设计_ai优化版.md` §9 的 **L0–L6**；
> 完整验收判据、依赖关系与待确认项见 **`doc\设计落实待办.md`**（含"三个硬件前提"与"下一步可执行"）。

| 阶段 | 目标 | 本工程状态 |
| --- | --- | --- |
| **L0 底盘闭环** | 4×`speed_pid` + `motor_tb6612` 速度环整定。判据：目标 300 rpm 稳态误差 ≤5%；堵转保护与急停有效 | `car_fpga` 里**仿真 + 综合已完成、待上板**（`speed_loop.fs`）；应用层**尚未搬进本工程** |
| L1 估计层 | `odom` + 编码器互补 + 侧滑系数 k 标定。判据：静止 60 s yaw 漂移 <1°；直行 5 m 横向偏差 <10 cm；原地转 360° 误差 <5° | 未开始；依赖实测（轮距 L / 轮径 / 减速比 / 编码器线数） |
| L2 显示与交互 | `hdmi_1024x600` + `ui_compositor` + 四页 UI。判据：四页可切换、无撕裂、数据与串口一致 | 未开始；菜单 UI 逻辑目前只在 `class\ov5640_menu`、`try_menu` 试验工程里，未进模块库 |
| L3 感知 | ToF 急停 + 雷达建图。判据：距障碍 30 cm 内 100 ms 内刹停；雷达图刷新 ≥10 Hz | 雷达模块已内嵌、未接入（**RX 接 C14**）；ToF 模块库里没有，需新建 |
| L4 遥控接管 | `rc_rx` + `mode_arb`。判据：接管/让出无扰切换；**丢信号 200 ms 内停车** | 待确认遥控接收机输出形式（库内 `rc_pwm_receiver` 是 **2 通道**航模 PWM 捕获，6 通道/PPM/SBUS 则不够） |
| L5 导航 | 路径点跟踪 + 局部绕障（可选全局规划/RK3588）。判据：沿 3~5 个路径点依次到达，误差 <30 cm | 未开始 |
| L6 预留 | RK3588 通路（USB2.0 / 以太网） | 未开始 |

**第一步（L0）的搬迁清单**见 `rtl\README.md`：`speed_pid` / `uart_cmd_rx` / `speed_report` /
`speed_loop_top` / `reset_sync` 属应用层要搬；`motor_*`、`uart`、`sync_fifo` **不搬**，
直接用 vendor（这正是内嵌方案替代 `car_fpga\ext\` 副本模式的地方）。
例化 `imu_top` 时**必须显式写 `.SCK_HALF_DIV(25)`**（RTL 默认值 3 = 8.33 MHz 实测会导致初始化失败）。

---

## 八、已知坑速查（详细版见 `class\工作交接总结.md`）

| 坑 | 一句话结论 |
| --- | --- |
| 单元测试过、集成不通 | 跨模块的握手/时序契约必须用**集成测试**覆盖（曾有 `o_data_valid` vs `o_acc_valid` 接错，单元测试完全没抓到） |
| ModelSim `-c` 模式 | tb 里的 `$finish` 会**终结整个 vsim 进程** → 每个测试开独立进程，别串在一个 do 里 |
| 只给 `.cst` 不给 `.sdc` | 工具按默认 100 MHz 估算 → 假时序违规，两个约束必须同时给 |
| 引脚"能用"≠"正确" | 旧约束与飞线接法自洽，换成排线直连后 21 根信号**整排错位** |
| 堵转判据 | 不能用"转速为 0"或单窗口阈值（抖动与低速转动同量级）→ 用**净位移**判据 |
| 中文文件读取 | PS 5.1 的 `Get-Content` 默认按 GBK 解码无 BOM UTF-8，**静默**给出错误结果 |
| `.ps1` 的 BOM | `edit` 类工具会丢 BOM，改完必查前 3 字节 `EF BB BF`（BOM 归 BOM、行尾归行尾，两者独立） |
| 行尾策略 | 必须随仓库走（`.gitattributes`），不能依赖各机器的 `core.autocrlf`；否则改一行显示成全文件重写 |
| `git archive` 而非拖拽 | 拖拽会把 `sim\work\`、`transcript`、`*.qdb` 等未跟踪产物一起搬进仓库 |

---

## 九、实现进度与未完成项

### 本轮已完成（2026-10-15）

- **应用层 RTL**：`falcons_top`（顶层）+ `chassis_ctrl`（差速分解 + 4×增量式 PID）+ `rc_input`（6 路遥控捕获与模式/按钮）
  + `app_key`（按键去抖）+ `ui_render`（中文界面）+ 从 `car_fpga` 搬入的 `speed_pid` / `reset_sync` / `uart_cmd_rx` / `speed_report`
- **中文点阵字库**：`tools\gen_font_rom.py` 从 `tools\font_chars.txt` 生成 220 字点阵 + 73 条文案索引
  → `rtl\common\font_rom.v`、`rtl\common\ui_text_rom.v`、`rtl\ui\ui_text_index.vh`（数据内嵌，不用 $readmemh）
- **实时数值显示**：`rtl\ui\num_text.v`（6 个字段 ×4 字符，逐位减法格式化，100 ms 刷新）
  + `rtl\ui\dyn_text_ram.v`（简单双口 BSRAM）+ `ui_render` 的 6 段译码（静态文案与动态数值混合渲染）
- **板级约束**：`constr\falcons.cst`（逐针抄手册；雷达 RX = **C14**）+ `constr\falcons.sdc`
- **综合与布局布线**：ERROR 0、比特流 33.5 MB、时序仅剩 CDC 结构性违规（详见 `doc\上板指南_v1.md` 第六节）

### 尚未完成

1. ~~没有仿真测试~~ **已完成**：4 个自检 tb（`rc_input` 12 项 / `chassis_ctrl` 23 项 / `num_text` 22 项 / `ui_render` 8 项 = **65 项全 PASS**）+ `sim\run_sim.ps1` 批量入口；仿真期间抓出并修掉 3 个真缺陷（差速限幅顺序、状态栏配色、数值格式化语法）
2. ~~数据页无数值~~ **已实现**：`num_text`（sys 域整数→定宽字符）+ `dyn_text_ram`（sys 写/pixel 读双口）+ `ui_render` 段化渲染，数据页显示左右目标轮速与四路实际轮速（亮绿数值）
3. ~~IMU 链路未接~~ **已接入**：`imu660ra` + `gyro_tilt_comp` + `yaw_integrate` 已进 `deps\modules.list` 并在 `falcons_top` 例化（`u_imu`/`u_tilt`/`u_yaw`），数据页航向角显示实时值；综合 ERROR 0
4. **串口只收不发**：`speed_report` 已搬入但未接 `uart_byte_tx`；`T/1-4`、`E/U` 命令未接
5. **`tools\` 还缺 3 个脚本**：`check_pin_constraints.py`、`program_flash.ps1`、`plot_*.py`（见 `tools\README.md`）
4. **`tools\` 还缺 3 个脚本**：`check_pin_constraints.py`、`program_flash.ps1`、`plot_*.py`（见 `tools\README.md`）。
5. ~~未做首次提交~~ **已完成**：首次提交 `bb9ea07`（199 文件 / 72107 行，含内嵌快照 144 文件），工作区干净。**注意这是本地仓库 —— 尚未配置远程、未 push**。
6. **快照升级路径**：将来若要改用 git submodule，把 `vendor\verilog-reusable-modules` 这个挂载点
   就地换成 submodule 即可，应用层与脚本几乎不用改（方案文档第二节 D→C 的平滑升级）。
