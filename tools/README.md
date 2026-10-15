# tools\ —— 工程自带的脚本

## 现有脚本

| 脚本 | 作用 | 什么时候跑 |
| --- | --- | --- |
| `lib_paths.ps1` | **所有路径的唯一定义处**（工程根、vendor 根、`deps\modules.list`、外部工具路径）+ 三个公共函数 | 被其它脚本 `dot-source`，不单独运行 |
| `build.ps1` | 一键构建：调 `gw_sh.exe` 综合 + 布局布线，最后列出 `impl\pnr\*.fs` | 日常构建 |
| `run_all_modules.ps1` | 把 `vendor\` 里各模块自带的测试跑一遍（**模块自带测试当门槛**），日志落 `logs\` | **改了 vendor 里某个模块之后**做回归 |
| `gen_font_rom.py` | 中文字库生成器：从 `font_chars.txt` 渲染 16×16 点阵 + 字符串索引，**数据内嵌**进 Verilog（不用 `$readmemh`） | 改了界面文案后 |
| `font_chars.txt` | UI 文案字表（74 条），`gen_font_rom.py` 的输入 | 加 / 改界面文案时 |
| `check_pin_constraints.py` | ★ **引脚约束逐针复验**：从手册 PDF 原文解析「针号→网络→J1正式Pin」，与 `constr\falcons.cst` 里实际生效的 `IO_LOC` 逐针比对（覆盖手册 J2/J4/J5/J6/J7 五张表，共 35 根信号），并查引脚唯一性与注释一致性 | **改约束 / 换接线 / 拿到新版手册后必跑** |
| `program_flash.ps1` | ★ **位流下载与固化**（`programmer_cli.exe` 封装）：`-Action Cables/Scan/DetectId/Sram/Program/Verify`，`-DryRun` 只打印命令；内置"位流文件时间是否晚于最后一次改 RTL"的提醒 | **上板测试第一步**：先 `-Action Sram`（掉电即失）验证功能，确认无误再 `-Action Program` 固化 |

```powershell
# 常用
powershell -File tools\build.ps1                                    # 构建
powershell -File tools\build.ps1 -Tcl build_speed.tcl               # 分阶段工程
powershell -File tools\run_all_modules.ps1 -Packages motor_tb6612   # 改了某个模块后回归
powershell -File tools\run_all_modules.ps1 -ReferencedOnly          # 只跑 modules.list 里的包
python tools\gen_font_rom.py                                        # 重新生成字库
python tools\check_pin_constraints.py                               # ★ 引脚逐针复验（改约束/换接线后必跑）

# 烧录（★ 上板第一步先只下 SRAM，别直接固化）
powershell -File tools\program_flash.ps1 -Action Cables             # 列已连接的下载器
powershell -File tools\program_flash.ps1 -Action DetectId           # 读板上 Flash 的 ID
powershell -File tools\program_flash.ps1 -Action Sram               # ★ 只下 FPGA（掉电即失，先验证功能）
powershell -File tools\program_flash.ps1 -Action Program            # 擦除 + 写入 + 校验（固化）
powershell -File tools\program_flash.ps1 -Action Sram -DryRun       # 只打印命令不执行
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
`Get-Content` 在 PS 5.1 下默认按 ANSI（本机 GBK）解码无 BOM 的 UTF-8，会把汉字续字节后的
`0x0A` 当 trail byte 吞掉 —— **不报错，只是安静地给出错误结果**（曾把 588 行数成 434 行）。
统一用本目录的 `Read-Utf8Text`，或 `Get-Content -Encoding UTF8`。

## 尚未迁入（按分步计划补）

| 来源 | 用途 |
| --- | --- |

| `class\car_fpga\tools\plot_yaw.py` | 串口采集 + 分析 + 绘图。迁入时避免用 `⚠`/`✓` 直接 print（GBK 控制台下 `UnicodeEncodeError` 直接崩溃） |

> `program_flash.ps1` **已于 2026-10-15 迁入**（见上表）。迁入时只改了默认位流名
> `imu_demo.fs → falcons.fs`，脚本主体（含"必须展开 `@cliArgs` 而不是 `@args`"的修正）
> 原样保留 —— 旧写法 `& $cli @args` 里的 `$args` 是自动变量，脚本用了 `param()` 后它恒为空，
> 等于不带参数执行 `programmer_cli`。
