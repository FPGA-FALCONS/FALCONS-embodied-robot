# tools\ —— 工程自带的脚本

## 现有脚本

| 脚本 | 作用 | 什么时候跑 |
| --- | --- | --- |
| `lib_paths.ps1` | **所有路径的唯一定义处**（工程根、vendor 根、`deps\modules.list`、外部工具路径）+ 三个公共函数 | 被其它脚本 `dot-source`，不单独运行 |
| `build.ps1` | 一键构建：调 `gw_sh.exe` 综合 + 布局布线，最后列出 `impl\pnr\*.fs` | 日常构建 |
| `run_all_modules.ps1` | 把 `vendor\` 里各模块自带的测试跑一遍（**模块自带测试当门槛**），日志落 `logs\` | **改了 vendor 里某个模块之后**做回归 |
| `gen_font_rom.py` | 字库生成器：从 `font_chars.txt` 一次生成**三套点阵**——小字 `font_rom.v`（16×16，4096×16）、大字 `font_rom_lg.v`（24×24，6144×24）、紧凑数字 `font_num.v`（16 宽×24 高，6144×16），外加字符串 ROM 与索引；**数据内嵌**进 Verilog（不用 `$readmemh`） | 改了界面文案或字号后 |
| `font_chars.txt` | UI 文案字表（当前119条；字符容量与字段数见[参数总表](../doc/参数总表与待定清单.md#显示与存储)），`gen_font_rom.py` 的输入 | 加 / 改界面文案时 |
| `check_pin_constraints.py` | ★ **引脚约束逐针复验**：从手册 PDF 原文解析「针号→网络→J1正式Pin」，与 `constr\falcons.cst` 里实际生效的 `IO_LOC` 逐针比对（覆盖手册 J2/J4/J5/J6/J7 五张表，共 35 根信号），并查引脚唯一性与注释一致性。★ **2026-10-16 队友那轮修过一个真缺陷**：PyMuPDF 回退路径原按**逐单元格**提取文本，五张表会被解析成 **0 针**（静默"全过"）；已改为 `sort=True` **保留行布局**，并增加**各连接器针数门槛**（针数不足即报错，不再假装通过）。比对用的手册已拷入本工程：`doc\ACG720_引脚接线手册_RevI.pdf` | **改约束 / 换接线 / 拿到新版手册后必跑** |
| `preview_data_page.py` | ★ **数据页离屏预览器**（2026-10-18 新增，**只读**、不生成任何 RTL）：**直接解析 `rtl\common\font_num.v` 与 `font_rom_lg.v` 的点阵**，按 `ui_render.v` 的真实布局把"运行数据页"渲染成 1024×600 PNG。用途：改数值位宽/坐标/小数点后，**在综合之前**肉眼确认"数字有没有撞标签或撞轮速框"。它复刻了 `num_text.v` 的 5 格格式化（含前导零抑制与负号跟随最高位）和 `sm_dot` 的悬挂小数点判定 —— 两处**当场抓出过真实 RTL bug**（见 `工作记录.md` §42.7）。`--preset typical` 给典型行驶值，也可用 `--center/--pose-x/--gyro/--yaw/--w0..w3` 逐项指定 | 改了数据页布局、字段位宽、小数点后（**综合之前**） |
| `sta_timing_probe.tcl` | ★ **时序统计口径的"探针"构建**（一次性调查用）：用 `-output_base_name falcons_sta` 把整套报告落到**新文件名**（`impl\pnr\falcons_sta*`），**一个字节都不动** `impl\pnr\falcons.*`（不覆盖现有位流与报告；是否已加载以下载日志为准）。用途：对**同一份 RTL 再独立布一次线**，用来区分"结构性统计"与"随机落点"。⚠ 结论：本项目需要它时只在复核时序口径时跑；`gw_sh` 的 tcl **没有** `report_timing` 命令（详见 `doc\工作记录.md` §21.9） | 复核时序统计口径时（**非日常构建**）。★ 2026-10-16 首跑后已按需求方授权**清理产物（142.4 MB）**，只留 `impl\pnr\falcons_sta_tr_content.html` + `impl\pnr\falcons_sta.timing_paths` 两份证据；需要时重跑本脚本即可复现（约 2 分钟） |
| `program_flash.ps1` | ★ **位流下载与固化**（`programmer_cli.exe` 封装）：`-Action Cables/Scan/DetectId/Sram/Program/Verify`，`-DryRun` 只打印命令；内置位流新鲜度的**硬检查**：`rtl\**\*.v|*.vh` 的 mtime 晚于位流即 `throw`（`-Force` 跳过；mtime 只是粗筛，会因 git 检出而变） | **上板测试第一步**：先 `-Action Sram`（掉电即失）验证功能，确认无误再 `-Action Program` 固化 |

```powershell
# 常用
powershell -File tools\build.ps1                                    # 构建
powershell -File tools\build.ps1 -Tcl build_speed.tcl               # 分阶段工程
powershell -File tools\run_all_modules.ps1 -Packages motor_tb6612   # 改了某个模块后回归
powershell -File tools\run_all_modules.ps1 -ReferencedOnly          # 只跑 modules.list 里的包
python tools\gen_font_rom.py                                        # 重新生成字库
python tools\check_pin_constraints.py                               # ★ 引脚逐针复验（改约束/换接线后必跑）
python tools\preview_data_page.py                                   # ★ 数据页预览图（改布局后、综合之前）
python tools\preview_data_page.py --preset typical --out doc\img\data_page_typical.png

# 烧录（★ 上板第一步先只下 SRAM，别直接固化）
powershell -File tools\program_flash.ps1 -Action Cables             # 列已连接的下载器
powershell -File tools\program_flash.ps1 -Action DetectId           # 读板上 Flash 的 ID
powershell -File tools\program_flash.ps1 -Action Sram               # ★ 只下 FPGA（掉电即失，先验证功能）
powershell -File tools\program_flash.ps1 -Action Program            # 擦除 + 写入 + 校验（固化）
powershell -File tools\program_flash.ps1 -Action Sram -DryRun       # 只打印命令不执行
powershell -File tools\program_flash.ps1 -Action Sram -Force        # 位流比 RTL 旧时跳过新鲜度硬检查（确知该位流可用才加）
```

> ⚠ **改了 vendor 里的模块，必须跑 `sim\run_sim.ps1`**（整车仿真回归）。模块是整车的一部分，
> 改了电机驱动 / IMU 驱动 / 遥控捕获都会影响整车行为 —— 见 `vendor\ORIGIN.md`。

## 2026-10-15 删除的两个脚本（历史存档）

`sync_vendor.ps1` 与 `check_vendor.ps1` 已随"vendor 转为本项目直接维护"一并删除：
这批模块只有本工程一个消费者，同步机制的复杂度不值得。来龙去脉见 `vendor\ORIGIN.md`；
需要时可用 `git checkout <删除前的提交> -- tools/sync_vendor.ps1` 从历史取回。

## `run_all_modules.ps1` 的三个实现细节（踩出来的）

1. **`-do` 的路径必须正斜杠化并加大括号**：Windows 反斜杠在 Tcl 里是转义字符，
   `D:\Programs\FPGA` 会被吃成 `D:ProgramsFPGA`，vsim 只报 `Cannot open macro file` 却不报错退出。
2. **末尾补 `; quit -f`**：有的 `.do` 结尾没有 quit，vsim 读到 EOF 会抛
   `U/I: Unexpected EOF on RPC channel` 并以非 0 退出（`rc_pwm_receiver` 正是如此）。
3. **判定不能裸匹配 `FAIL`**：`rc_pwm_receiver` 的 tb 会打印
   `TOTAL: 17 groups PASS; 502 assertions PASS; 0 FAIL` —— "零失败"会被误判成失败。
   脚本把 `0 FAIL / FAILURES: 0 / no FAIL` 这类表述先数出来再判断。

## ★ 编码要求：`.ps1` 必须是 UTF-8 **with BOM**

PowerShell 5.1 读无 BOM 的脚本会**按 GBK 解释**，中文注释直接变成乱码 + 语法错误。
`write`/`edit` 类工具默认写无 BOM 的 UTF-8，**改完 `.ps1` 必查前 3 字节是不是 `EF BB BF`**：

```powershell
$p = 'tools\build.ps1'
$c = [System.IO.File]::ReadAllText($p, [System.Text.Encoding]::UTF8)
[System.IO.File]::WriteAllText($p, $c, (New-Object System.Text.UTF8Encoding($true)))
```

> ⚠ **用脚本批量替换 `.ps1` 时，写回必须带上原文件的 BOM**。
> 2026-10-15 改 `sim\run_sim.ps1` 时用 `[System.IO.File]::WriteAllText($p, $s, UTF8Encoding($false))`
> 批量替换，结果写成无 BOM；下一次运行 PowerShell 把中文注释按 GBK 解出乱码，
> 报的是一串 `Unexpected token` / `The string is missing the terminator` / `Missing closing '}'`
> —— 看起来像脚本被改坏了，实际只是少了 3 个字节。
> 正确做法：先 `ReadAllBytes` 判断原文件有无 BOM，写回时用 `UTF8Encoding($true/$false)` 与之保持一致。

反过来，**读中文文件要显式指定编码**：
`Get-Content` 在 PS 5.1 下默认按 ANSI（本机 GBK）解码无 BOM 的 UTF-8，可能把中文显示成乱码，导致人工核对出错。
统一用本目录的 `Read-Utf8Text`，或 `Get-Content -Encoding UTF8`。

## 引脚校验依赖

`check_pin_constraints.py`读取`doc/ACG720_引脚接线手册_RevI.pdf`。优先使用PATH中的`pdftotext`；找不到该程序时使用已安装的PyMuPDF。若找到了程序但执行报权限/访问错误，不会自动切换后端，应先检查程序位置、文件权限及PDF路径。程序执行失败不代表引脚校验通过。

## 尚未迁入（按分步计划补）

| 来源 | 用途 |
| --- | --- |
| `class\car_fpga\tools\plot_yaw.py` | 串口采集 + 分析 + 绘图。迁入时避免用 `⚠`/`✓` 直接 print（GBK 控制台下 `UnicodeEncodeError` 直接崩溃） |

> `program_flash.ps1` **已于 2026-10-15 迁入**（见上表）。迁入时只改了默认位流名
> `imu_demo.fs → falcons.fs`，脚本主体（含"必须展开 `@cliArgs` 而不是 `@args`"的修正）
> 原样保留 —— 旧写法 `& $cli @args` 里的 `$args` 是自动变量，脚本用了 `param()` 后它恒为空，
> 等于不带参数执行 `programmer_cli`。

## 独立手柄构建

`build.ps1 -Tcl build_rc_led.tcl -ArtifactDirectory impl/rc_led_test/impl/pnr`构建并报告独立手柄位流；默认不传参数仍构建整车。测试约束由规范约束筛选生成，详见`doc/手柄LED上板测试.md`。文件和日志都落在已有忽略的impl/logs目录，不重复提交生成物。

## 竞赛源码导出

`python tools/package_competition_source.py --out '<仓库外的新英文目录>'`
按生产依赖导出独立源码目录和 ZIP，保留 FPGA/RK 源码、标定、测试及项目训练工具，
排除构建产物、历史过程证据、未接入模块及独立板卡例程。脚本同时导出文件哈希与排除清单，
不删除原仓库文件，不包含模型权重及 RK 当前运行环境，详见 [竞赛源码提交说明](../doc/竞赛源码提交说明.md)。
