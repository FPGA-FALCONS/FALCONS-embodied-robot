# run_sim.ps1 —— 本工程的仿真批量入口（每个测试开独立 vsim 进程）
#
# 为什么必须"每个测试一个进程"：
#   ModelSim 在 -c（命令行）模式下，测试平台里的 $finish 会【终结整个 vsim 进程】。
#   把多个 tb 串在同一个 do 脚本里跑，只有第一个会执行，后面静默跳过且不报错 ——
#   极易误判成"全部通过"（这是模块库 README 与本工程交接文档都强调过的坑）。
#
# 覆盖的测试（都是自检 tb，末尾打印 PASS:/FAIL:；**项数以 tb 实际打印为准，此处不再写死数字**）：
#   rc_input_tb      遥控捕获、归一化/死区/模式两档/按钮(select+confirm)/丢信号/取反/摇杆拨动/★掩码有效性
#   chassis_ctrl_tb  差速分解、限幅、安全逻辑、闭环方向、rpm 换算
#   num_text_tb      实时数值格式化（9 字段、负数、前导空格、字段边界、>999 饱和）
#   ui_render_tb     中文界面渲染（菜单/★高亮随 menu_sel/状态栏/★光标档文案/门禁提示/动态数值/占位）
#   speed_report_tb  串口 CSV 上行 12 列格式（整行逐字符比对）
#
# 编码：本文件必须保存为 UTF-8 with BOM（PowerShell 5.1 靠 BOM 判编码）。
#
# 用法：
#   powershell -File sim\run_sim.ps1
#   powershell -File sim\run_sim.ps1 -ModelSim 'D:\Apps\modeltech64_10.6d\win64'

[CmdletBinding()]
param(
    [string]$ModelSim = 'D:\Apps\modeltech64_10.6d\win64',
    [string]$Library  = 'work_sim'
)

$ErrorActionPreference = 'Stop'

$ProjectRoot = Split-Path -Parent $PSScriptRoot          # sim\ 的上一级 = 工程根
$VendorRoot  = Join-Path $ProjectRoot 'vendor\verilog-reusable-modules\modules'
$UiIncDir    = Join-Path $ProjectRoot 'rtl\ui'           # ui_text_index.vh 所在目录

$vlib = Join-Path $ModelSim 'vlib.exe'
$vlog = Join-Path $ModelSim 'vlog.exe'
$vsim = Join-Path $ModelSim 'vsim.exe'

foreach ($exe in @($vlib, $vlog, $vsim)) {
    if (-not (Test-Path $exe)) { throw "找不到 $exe`n用 -ModelSim 指定 ModelSim 的 win64 目录。" }
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
    "$ProjectRoot\rtl\motor\speed_pid.v",
    "$ProjectRoot\rtl\motor\chassis_ctrl.v",
    "$ProjectRoot\rtl\ui\dyn_text_ram.v",
    "$ProjectRoot\rtl\debug\speed_report.v",
    "$ProjectRoot\rtl\ui\num_text.v",
    "$ProjectRoot\rtl\ui\ui_render.v",
    # 测试平台
    "$PSScriptRoot\rc_input_tb.v",
    "$PSScriptRoot\chassis_ctrl_tb.v",
    "$PSScriptRoot\num_text_tb.v",
    "$PSScriptRoot\ui_render_tb.v",
    "$PSScriptRoot\speed_report_tb.v"
)

$tests = @(
    @{ Name = 'rc_input';     Top = 'rc_input_tb';     Desc = '遥控：归一化/死区/模式两档/按钮 select+confirm/丢信号/取反/摇杆拨动/掩码有效性' },
    @{ Name = 'chassis_ctrl'; Top = 'chassis_ctrl_tb'; Desc = '★ 差速分解/限幅/安全逻辑/闭环方向/rpm 换算' },
    @{ Name = 'num_text';     Top = 'num_text_tb';     Desc = '实时数值格式化（9 字段、负数、前导空格、饱和）' },
    @{ Name = 'ui_render';    Top = 'ui_render_tb';    Desc = '★ 中文界面：菜单/高亮随 menu_sel/状态栏/光标档文案/动态数值/占位页' },
    @{ Name = 'speed_report'; Top = 'speed_report_tb'; Desc = '串口 CSV 上行：12 列格式/负数/边界/大数' }
)

# ---------- 建库并编译 ----------
Write-Host ''
Write-Host '================ 编译（FALCONS 仿真）================' -ForegroundColor Cyan

if (Test-Path $Library) { Remove-Item -Recurse -Force $Library }
& $vlib $Library | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'vlib 建库失败' }

foreach ($f in $sources) {
    if (-not (Test-Path $f)) { throw "源文件缺失：$f" }
    $out = & $vlog -work $Library "+incdir+$UiIncDir" $f 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "编译失败：$f" -ForegroundColor Red
        $out | ForEach-Object { Write-Host "  $_" }
        exit 1
    }
}
Write-Host "编译通过（$($sources.Count) 个文件，无错误）" -ForegroundColor Green

# ---------- 逐个测试（每个独立进程）----------
$results = @()

foreach ($t in $tests) {
    Write-Host ''
    Write-Host "================ 测试：$($t.Name) ================" -ForegroundColor Cyan
    Write-Host "  $($t.Desc)"

    $out  = & $vsim -c -work $Library -do "run -all; quit -f" "$Library.$($t.Top)" 2>&1
    $text = ($out | ForEach-Object { $_.ToString() }) -join "`n"

    # 只回显检查行与结论（tb 输出很多）
    $out | ForEach-Object {
        $s = $_.ToString().TrimStart('#',' ')
        if ($s -match '^\s*(pass |FAIL|PASS:|FAIL:|-- )') { Write-Host "  $s" }
    }

    # ★ 除文本匹配外还检查 vsim 退出码：tb 打完 "PASS:" 之后若异常退出，也必须算失败
    #   （只匹配文本会在这种情况下误记 PASS）
    if (($null -ne $LASTEXITCODE) -and ($LASTEXITCODE -ne 0)) {
        $status = "FAIL (vsim exit $LASTEXITCODE)"
    } elseif ($text -match 'PASS:') { $status = 'PASS' }
    elseif ($text -match 'FAIL:')   { $status = 'FAIL' }
    else                            { $status = 'NO-RESULT' }

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
