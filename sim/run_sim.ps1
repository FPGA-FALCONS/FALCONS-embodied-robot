# run_sim.ps1 —— 本工程的仿真批量入口（每个测试开独立 vsim 进程）
#
# 为什么必须"每个测试一个进程"：
#   ModelSim 在 -c（命令行）模式下，测试平台里的 $finish 会【终结整个 vsim 进程】。
#   把多个 tb 串在同一个 do 脚本里跑，只有第一个会执行，后面静默跳过且不报错 ——
#   极易误判成"全部通过"（这是模块库 README 与本工程交接文档都强调过的坑）。
#
# 覆盖的测试（都是自检 tb，末尾打印 PASS:/FAIL:）：
#   ht10a_input_tb  HT10A 通道/标定/开关/失效/启动门禁集成
#   rc_input_tb      6 路遥控捕获、归一化/死区/模式两档/按钮/丢信号  （26 项）
#   chassis_ctrl_tb  差速分解、限幅、安全逻辑、闭环方向、rpm 换算      （23 项）
#   num_text_tb      实时数值格式化（含负数与前导空格）                （28 项）
#   ui_render_tb     中文界面渲染（菜单/高亮/状态栏/门禁提示/动态数值/占位）
#   speed_report_tb  串口 CSV 上行 12 列格式（整行逐字符比对）          （4 行）
#
# 编码：本文件必须保存为 UTF-8 with BOM（PowerShell 5.1 靠 BOM 判编码）。
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

$ProjectRoot = Split-Path -Parent $PSScriptRoot          # sim\ 的上一级 = 工程根
$VendorRoot  = Join-Path $ProjectRoot 'vendor\verilog-reusable-modules\modules'
$UiIncDir    = Join-Path $ProjectRoot 'rtl\ui'           # ui_text_index.vh 所在目录

if ($Simulator -eq 'ModelSim') {
    $vlib = Join-Path $ModelSim 'vlib.exe'
    $vlog = Join-Path $ModelSim 'vlog.exe'
    $vsim = Join-Path $ModelSim 'vsim.exe'
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
    @{ Name = 'rc_input';     Top = 'rc_input_tb';     Desc = '遥控 6 路：归一化/死区/模式两档/按钮/丢信号' },
    @{ Name = 'chassis_ctrl'; Top = 'chassis_ctrl_tb'; Desc = '★ 差速分解/限幅/安全逻辑/闭环方向/rpm 换算' },
    @{ Name = 'num_text';     Top = 'num_text_tb';     Desc = '实时数值格式化（负数、前导空格、字段边界）' },
    @{ Name = 'ui_render';    Top = 'ui_render_tb';    Desc = '★ 中文界面：菜单/高亮/状态栏/动态数值/占位页' },
    @{ Name = 'speed_report'; Top = 'speed_report_tb'; Desc = '串口 CSV 上行：12 列格式/负数/边界/大数' }
)

# ---------- 建库并编译 ----------
Write-Host ''
Write-Host '================ 编译（FALCONS 仿真）================' -ForegroundColor Cyan

$libraryPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot $Library))
$simPrefix = [System.IO.Path]::GetFullPath($PSScriptRoot).TrimEnd('\') + '\'
if (-not $libraryPath.StartsWith($simPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'Library must stay within sim/.'
}
if ($Simulator -eq 'ModelSim') {
    # Reuse the library; recompile all listed sources, avoiding recursive deletion.
    if (-not (Test-Path $libraryPath)) {
        & $vlib $libraryPath | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
    }
    foreach ($f in $sources) {
        if (-not (Test-Path $f)) { throw "Missing source: $f" }
        $out = & $vlog -work $libraryPath "+incdir+$UiIncDir" $f 2>&1
        if ($LASTEXITCODE -ne 0) { $out | ForEach-Object { Write-Host $_ }; throw "Compile failed: $f" }
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
        $out = & $vsim -c -work $libraryPath -do "run -all; quit -f" "$Library.$($t.Top)" 2>&1
        $testExit = $LASTEXITCODE
    } else {
        $image = Join-Path $libraryPath "$($t.Top).out"
        $compileOut = & $iverilog -g2012 -I $UiIncDir -s $t.Top -o $image @sources 2>&1
        if ($LASTEXITCODE -ne 0) { $compileOut | ForEach-Object { Write-Host $_ }; throw "Compile failed: $($t.Top)" }
        $out = & $vvp $image 2>&1
        $testExit = $LASTEXITCODE
    }
    $text = ($out | ForEach-Object { $_.ToString() }) -join "`n"

    # 只回显检查行与结论（tb 输出很多）
    $out | ForEach-Object {
        $s = $_.ToString().TrimStart('#',' ')
        if ($s -match '^\s*(pass |FAIL|PASS:|FAIL:|-- )') { Write-Host "  $s" }
    }

    if ($testExit -ne 0 -or $text -match '(?im)^\s*(?:#\s*)?(?:FAIL|FATAL|\*\* Error)') { $status = 'FAIL' }
    elseif ($text -match 'PASS:') { $status = 'PASS' }
    else                          { $status = 'NO-RESULT' }

    $results += [pscustomobject]@{ 测试 = $t.Name; 结果 = $status }
}

# ---------- 汇总 ----------
Write-Host ''
Write-Host '================ 汇总 ================' -ForegroundColor Cyan
$results | Format-Table -AutoSize

$bad = @($results | Where-Object { $_.结果 -ne 'PASS' })
if ($bad.Count -eq 0) {
    Write-Host "全部 $($tests.Count) 个测试通过" -ForegroundColor Green
    exit 0
} else {
    Write-Host "$($bad.Count) 个测试未通过：$(($bad.测试) -join ', ')" -ForegroundColor Red
    exit 1
}
