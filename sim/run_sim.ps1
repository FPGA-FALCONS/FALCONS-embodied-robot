# run_sim.ps1 —— 本工程的仿真批量入口（每个测试开独立 vsim 进程）
#
# 为什么必须"每个测试一个进程"：
#   ModelSim 在 -c（命令行）模式下，测试平台里的 $finish 会【终结整个 vsim 进程】。
#   把多个 tb 串在同一个 do 脚本里跑，只有第一个会执行，后面静默跳过且不报错 ——
#   极易误判成"全部通过"（这是模块库 README 与本工程交接文档都强调过的坑）。
#
# 覆盖的测试（都是自检 tb，末尾打印 PASS:/FAIL:）：
#   ht10a_input_tb  HT10A 通道/标定/开关/失效/启动门禁集成
#   rc_input_tb      遥控槽位映射、归一化/死区/模式两档/按钮(select+confirm)/丢信号/取反/摇杆拨动/掩码有效性
#   chassis_ctrl_tb  差速分解、限幅、安全逻辑、闭环方向、rpm 换算      （23 项）
#   num_text_tb      实时数值格式化（含负数与前导空格）                （39 项 = 原 36 + 2026-10-16 补的 3 条）
#   ui_render_tb     中文界面渲染（菜单/高亮/状态栏/门禁提示/动态数值/占位）
#   speed_report_tb  串口 CSV 上行 12 列格式（整行逐字符比对）          （4 行）
#
# 编码：本文件必须保存为 UTF-8 with BOM（PowerShell 5.1 靠 BOM 判编码）。
#
# 输出：屏幕只回显"检查行 + 结论"；每个测试的**完整输出**落盘
#       logs\sim_<test>.log（UTF-8 无 BOM，logs\ 已被 .gitignore 忽略）。
#       ★ 判 PASS 的三条硬条件（2026-10-16 收紧，见判定段注释）：
#         ① PASS:/FAIL 都**锚行首**匹配；② 先做 zeroFail 抵消（"0 FAIL" 不算失败）；
#         ③ tb 打印的 `CHECKS: N` 必须与实测 pass 行数一致。
#       ★ 每次运行前会**清空并重建**仿真库（只清 sim\work_sim 这一个目录）——
#         防止库里残留上次编译的旧模块，把"漏列源文件"掩盖成假通过。
#
# 用法：
#   powershell -File sim\run_sim.ps1
#   powershell -File sim\run_sim.ps1 -ModelSim 'D:\Apps\modeltech64_10.6d\win64'

[CmdletBinding()]
param(
    [string]$ModelSim = 'D:\Apps\modeltech64_10.6d\win64',
    [string]$Library  = 'work_sim',
    [ValidateSet('ModelSim','Icarus')][string]$Simulator = 'ModelSim'
)

$ErrorActionPreference = 'Stop'

# 【已知限制，2026-10-16 记录】logs\sim_<test>.log 里 tb 的**中文** $display 是乱码
#   （形如 "??????"、夹杂少量怪字），而 ASCII 部分（pass / FAIL / 断言名 / 数值）完好。
#   根因在 ModelSim 侧：vsim 把 UTF-8 无 BOM 的 tb 源文件读进来后，$display 字符串按
#   本地代码页输出，无法表示的字符在**字节层面**就已经是 '?'（0x3F，实测 dump 过）——
#   信息在进入 PowerShell 之前就丢了，**脚本侧修不了**。
#   （试过把 [Console]::OutputEncoding 设成 UTF-8：残余字节只会从"怪汉字"变成 U+FFFD，
#     可读性没有任何改善，所以不保留这个设置 —— 免得给人"已经修好了"的错觉。）
#   影响面：仅段标题行（"-- xxx --"）可读性差；判定与断言定位不受影响，
#   因为断言名、pass/FAIL、检查数全部是 ASCII。
#   真要可读的中文日志，得从 tb 侧解决（$display 只用英文，或让 vsim 以 UTF-8 输出），
#   不在本次改动范围内。

$ProjectRoot = Split-Path -Parent $PSScriptRoot          # sim\ 的上一级 = 工程根
$VendorRoot  = Join-Path $ProjectRoot 'vendor\verilog-reusable-modules\modules'
$UiIncDir    = Join-Path $ProjectRoot 'rtl\ui'           # ui_text_index.vh 所在目录
$LogDir      = Join-Path $ProjectRoot 'logs'             # 完整测试输出落盘目录（.gitignore 已忽略 logs\）

# ------------------------------------------------------------------
# 运行一个原生命令并完整捕获 stdout+stderr，返回 { Output, ExitCode }
#   ★ 为什么必须有这个包装（2026-10-16 加）：
#     本脚本开头是 `$ErrorActionPreference = 'Stop'`，而 `2>&1` 会把子进程的
#     stderr 行包装成 **ErrorRecord** —— 在 Stop 语义下第一行 stderr 就抛终止性
#     异常，脚本直接崩掉：既看不到测试输出、也拿不到非零退出码。
#     所以这些原生调用期间临时把偏好设为 'Continue'，调用结束再恢复；
#     非零退出码由调用方判定成 FAIL（而不是让脚本崩）。
# ------------------------------------------------------------------
function Invoke-Capture {
    param([scriptblock]$Command)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $output = & $Command 2>&1
        $code   = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prev
    }
    return [pscustomobject]@{ Output = @($output); ExitCode = $code }
}

if ($Simulator -eq 'ModelSim') {
    $vlib = Join-Path $ModelSim 'vlib.exe'
    $vlog = Join-Path $ModelSim 'vlog.exe'
    $vsim = Join-Path $ModelSim 'vsim.exe'
    # vdel 是"每次运行前清库"用的（见下面的建库段）。
    #   ★ 它是**可选**的：不存在就回退成直接删库目录，所以不参与下面的存在性检查。
    $vdel = Join-Path $ModelSim 'vdel.exe'
    foreach ($exe in @($vlib, $vlog, $vsim)) {
        if (-not (Test-Path $exe)) { throw "Missing $exe; specify -ModelSim or use -Simulator Icarus." }
    }
} else {
    $iverilog = (Get-Command iverilog -ErrorAction Stop).Source
    $vvp = Join-Path (Split-Path -Parent $iverilog) 'vvp.exe'
    if (-not (Test-Path $vvp)) { throw "Missing $vvp" }
}

Set-Location $PSScriptRoot

# ---------- 源文件（顺序无关，vlog 会各自独立编译）----------
$sources = @(
    # 内嵌模块库（只编译本工程用到的）
    "$VendorRoot\rc_pwm_receiver\rtl\rc_pwm_capture.v",
    "$VendorRoot\motor_tb6612\rtl\motor_pwm.v",
    "$VendorRoot\motor_tb6612\rtl\quad_decoder.v",
    "$VendorRoot\motor_tb6612\rtl\speed_measure.v",
    "$VendorRoot\motor_tb6612\rtl\stall_detect.v",
    "$VendorRoot\motor_tb6612\rtl\motor_unit.v",
    "$VendorRoot\motor_tb6612\rtl\motor_top.v",
    # 串口包（speed_report 内部例化 uart_tx_queue；falcons_top 用 uart_byte_rx）
    #   ★ 漏这几个文件时【编译仍然全过】—— vlog 是逐文件独立编译的，不做模块解析；
    #     直到 vsim 加载 design 才报 "Module 'uart_tx_queue' is not defined"，
    #     在 run_sim.ps1 的输出里表现为 speed_report 测试 "Error loading design"。
    "$VendorRoot\uart\rtl\uart_byte_tx.v",
    "$VendorRoot\uart\rtl\uart_tx_queue.v",
    "$VendorRoot\uart\rtl\uart_byte_rx.v",
    "$VendorRoot\uart\rtl\uart_rx_queue.v",
    # FIFO 包：uart_tx_queue / uart_rx_queue 内部都例化 sync_fifo
    #   ★ 依赖是【两层】的：speed_report → uart_tx_queue → sync_fifo。
    #     补依赖时只补一层仍会在 vsim 加载时报下一个没定义的模块，
    #     所以这里一次性把闭包补全（sync_fifo.v 是叶子，无下级例化）。
    "$VendorRoot\sync_fifo\rtl\sync_fifo.v",
    # 应用层
    "$ProjectRoot\rtl\common\font_rom.v",
    "$ProjectRoot\rtl\common\ui_text_rom.v",
    "$ProjectRoot\rtl\common\app_key.v",
    "$ProjectRoot\rtl\common\reset_sync.v",
    "$ProjectRoot\rtl\rc\rc_input.v",
    "$ProjectRoot\rtl\rc\ht10a_input.v",
    "$ProjectRoot\rtl\rc\rc_drive_guard.v",
    "$ProjectRoot\rtl\motor\speed_pid.v",
    "$ProjectRoot\rtl\motor\chassis_ctrl.v",
    "$ProjectRoot\rtl\ui\dyn_text_ram.v",
    "$ProjectRoot\rtl\debug\speed_report.v",
    "$ProjectRoot\rtl\ui\num_text.v",
    "$ProjectRoot\rtl\ui\ui_render.v",
    # 测试平台
    "$PSScriptRoot\rc_input_tb.v",
    "$PSScriptRoot\ht10a_input_tb.v",
    "$PSScriptRoot\chassis_ctrl_tb.v",
    "$PSScriptRoot\num_text_tb.v",
    "$PSScriptRoot\ui_render_tb.v",
    "$PSScriptRoot\speed_report_tb.v"
)

$tests = @(
    @{ Name = 'ht10a_input'; Top = 'ht10a_input_tb'; Desc = 'HT10A mapping/calibration/switches/loss/manual drive integration' },
    @{ Name = 'rc_input';     Top = 'rc_input_tb';     Desc = '遥控槽位映射：归一化/死区/模式两档/按钮 select+confirm/丢信号/取反/摇杆拨动/掩码有效性' },
    @{ Name = 'chassis_ctrl'; Top = 'chassis_ctrl_tb'; Desc = '★ 差速分解/限幅/安全逻辑/闭环方向/rpm 换算' },
    @{ Name = 'num_text';     Top = 'num_text_tb';     Desc = '实时数值格式化（9 字段、负数、前导空格、饱和）' },
    @{ Name = 'ui_render';    Top = 'ui_render_tb';    Desc = '★ 中文界面：菜单/高亮随 menu_sel/状态栏/光标档文案/动态数值/占位页' },
    @{ Name = 'speed_report'; Top = 'speed_report_tb'; Desc = '串口 CSV 上行：12 列格式/负数/边界/大数' }
)

# ---------- 建库并编译 ----------
Write-Host ''
Write-Host '================ 编译（FALCONS 仿真）================' -ForegroundColor Cyan

$libraryPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot $Library))
# ★ 库的**逻辑名**：vsim 的 design 名必须写成 `<库逻辑名>.<top>`，
#   不能拿 $Library 变量或整条路径去拼（路径形式在 -work 里能用，但 design 名不行）。
$libName     = Split-Path -Leaf $libraryPath
$simPrefix = [System.IO.Path]::GetFullPath($PSScriptRoot).TrimEnd('\') + '\'
if (-not $libraryPath.StartsWith($simPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'Library must stay within sim/.'
}
if ($Simulator -eq 'ModelSim') {
    # ★ 2026-10-16：每次运行前**先清空、再重建**这个库（原来是"库存在就复用"）。
    #   为什么必须清：复用会让库里留着**上一次运行编译出的旧模块**。实测
    #   sim\work_sim\_info 里存在模块 `vfalcons_top`，而 $sources 里根本没有这个源文件
    #   —— 历史遗留（顶层曾改名过一次）。危害是：以后把某个源文件从 $sources 里
    #   **漏掉**时，vlog 照样成功、vsim 也照样能加载（库里还留着上次编出来的那份），
    #   于是"漏列依赖"不再暴露 = **假通过**。
    #   实现：优先用 vdel -lib <该库> -all 清空；vdel 不存在或没清干净时，
    #   再直接删这一个库目录，然后 vlib 重建。
    #   ★ 只清这一个仿真库目录（sim\work_sim），不动 work_synt / work_check 等其它产物。
    if (Test-Path $vdel) {
        # 同样用 Invoke-Capture：Stop 语义下原生命令的 stderr 会变成终止性异常
        $null = Invoke-Capture { & $vdel -lib $libraryPath -all }
    }
    if (Test-Path $libraryPath) { Remove-Item -Recurse -Force $libraryPath }
    & $vlib $libraryPath | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
    foreach ($f in $sources) {
        if (-not (Test-Path $f)) { throw "Missing source: $f" }
        $r = Invoke-Capture { & $vlog -work $libraryPath "+incdir+$UiIncDir" $f 2>&1 }
        if ($r.ExitCode -ne 0) { $r.Output | ForEach-Object { Write-Host $_ }; throw "Compile failed: $f" }
    }
} else {
    New-Item -ItemType Directory -Path $libraryPath -Force | Out-Null
}
Write-Host "Source list: $($sources.Count) files; simulator: $Simulator" -ForegroundColor Green

# ---------- 逐个测试（每个独立进程）----------
$results = @()

foreach ($t in $tests) {
    Write-Host ''
    Write-Host "================ 测试：$($t.Name) ================" -ForegroundColor Cyan
    Write-Host "  $($t.Desc)"

    if ($Simulator -eq 'ModelSim') {
        # design 名用**库逻辑名** $libName（原来用 $Library 变量拼，一旦 -Library
        # 传的是"库名\子目录"这类形式就会拼错；现在统一从解析后的路径取名字）
        $r = Invoke-Capture { & $vsim -c -work $libraryPath -do "run -all; quit -f" "$libName.$($t.Top)" 2>&1 }
    } else {
        $image = Join-Path $libraryPath "$($t.Top).out"
        $c = Invoke-Capture { & $iverilog -g2012 -I $UiIncDir -s $t.Top -o $image @sources 2>&1 }
        if ($c.ExitCode -ne 0) { $c.Output | ForEach-Object { Write-Host $_ }; throw "Compile failed: $($t.Top)" }
        $r = Invoke-Capture { & $vvp $image 2>&1 }
    }
    $testExit = $r.ExitCode
    $out      = @($r.Output)
    $text     = ($out | ForEach-Object { $_.ToString() }) -join "`n"

    # ---------- 完整输出落盘 logs\sim_<test>.log（UTF-8 无 BOM）----------
    #   ★ 原来只把匹配 'pass |FAIL|PASS:|FAIL:|-- ' 的行回显到屏幕，其余**全丢**：
    #     "Error loading design"、"Module 'uart_tx_queue' is not defined"、
    #     编译告警等**最关键的失败信息恰好都不匹配那几个模式** → 屏幕上什么都没有，
    #     只剩一句 NO-RESULT，根本无从定位。
    #     现在：完整输出一律落盘（logs\ 已被 .gitignore 忽略），屏幕仍只回显检查行。
    if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory -Path $LogDir -Force | Out-Null }
    $logFile = Join-Path $LogDir "sim_$($t.Name).log"
    $logText = "# 测试：$($t.Name)  top=$($t.Top)  模拟器=$Simulator  退出码=$testExit`n" +
               "# 下面是完整输出（屏幕只回显了检查行与结论）`n" + $text + "`n"
    [System.IO.File]::WriteAllText($logFile, $logText, (New-Object System.Text.UTF8Encoding($false)))

    # 只回显检查行与结论（tb 输出很多；完整输出见上面的日志文件）
    $out | ForEach-Object {
        $s = $_.ToString().TrimStart('#',' ')
        if ($s -match '^\s*(pass |FAIL|PASS:|FAIL:|-- )') { Write-Host "  $s" }
    }
    Write-Host "  （完整输出：$logFile）" -ForegroundColor DarkGray

    # ---------- PASS / FAIL 判定（2026-10-16 收紧）----------
    #   ① 匹配**必须锚行首**：原写法 `(?im)^\s*(?:#\s*)?(?:FAIL|FATAL|\*\* Error)`
    #      能匹配任何"行首就是这些词"的内容（含 tb 里打印的说明句），
    #      而 PASS 用的是**不锚行首**的 'PASS:' —— 只要输出里出现过 "PASS:" 就算过。
    #      现在两边都锚行首：`(?m)^\s*#?\s*FAIL` 与 `(?m)^\s*#?\s*PASS:`。
    #   ② 先做 zeroFail 抵消：形如 "0 FAIL" / "FAIL: 0 ..." 是**通过**的汇总句，
    #      不能当失败，先在文本里剔掉再判。
    #   ③ 解析 tb 自己打印的 `CHECKS: N`，与实测 pass 行数比对：不一致即 FAIL ——
    #      防"断言被删/被跳过、但结论照样打印 PASS"。tb 不打印 CHECKS 时跳过这一项。
    $textNZF = $text -replace '(?m)^\s*#?\s*(?:0\s+FAIL\b.*|FAIL:\s*0\b.*)$', ''
    $passCount = ([regex]::Matches($textNZF, '(?m)^\s*#?\s*pass\b')).Count
    $checksM   = [regex]::Match($textNZF, '(?m)^\s*#?\s*CHECKS:\s*(\d+)\s*$')
    $checkTotal = if ($checksM.Success) { [int]$checksM.Groups[1].Value } else { -1 }

    if ($testExit -ne 0) {
        $status = 'FAIL'; $why = "退出码 $testExit"
    } elseif ($textNZF -match '(?m)^\s*#?\s*(?:FAIL|FATAL|\*\* Error)') {
        $status = 'FAIL'; $why = '输出里有 FAIL/FATAL/Error'
    } elseif ($textNZF -notmatch '(?m)^\s*#?\s*PASS:') {
        $status = 'NO-RESULT'; $why = '没有行首 PASS: 结论行'
    } elseif ($checkTotal -ge 0 -and $checkTotal -ne $passCount) {
        $status = 'FAIL'; $why = "CHECKS=$checkTotal 与实测 pass=$passCount 不一致"
    } else {
        $status = 'PASS'
        $why = if ($checkTotal -ge 0) { "pass=$passCount / CHECKS=$checkTotal" } else { "pass=$passCount" }
    }

    $results += [pscustomobject]@{ 测试 = $t.Name; 结果 = $status; 检查数 = $passCount; 说明 = $why }
}

# ---------- 汇总 ----------
Write-Host ''
Write-Host '================ 汇总 ================' -ForegroundColor Cyan
$results | Format-Table 测试, 结果, 检查数, 说明 -AutoSize

$bad = @($results | Where-Object { $_.结果 -ne 'PASS' })
if ($bad.Count -eq 0) {
    Write-Host "全部 $($tests.Count) 个测试通过" -ForegroundColor Green
    exit 0
} else {
    Write-Host "$($bad.Count) 个测试未通过：$(($bad.测试) -join ', ')" -ForegroundColor Red
    exit 1
}
