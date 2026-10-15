# run_motor.ps1 —— motor_tb6612 模块包的命令行批量仿真脚本
#
# 为什么需要这个脚本（而不是直接 do run_test.do）：
#   ModelSim 在 -c（命令行）模式下，测试平台里的 $finish 会【直接结束整个
#   vsim 进程】。因此不能在同一个 do 脚本里串行跑多个 tb —— 只有第一个会
#   执行，后面的静默不跑（且不报错，很容易误以为"全部通过了"）。
#   本脚本改为给每个测试启动一个独立的 vsim 进程，并输出 PASS/FAIL 汇总表。
#
#   sim/run_test.do    用于 ModelSim GUI 的 Transcript（那里 $finish 只结束
#                      当前仿真，ModelSim 不退出，串行跑 5 个没问题）
#   sim/run_motor.ps1  用于命令行（本文件）
#
# 编码：本文件必须保存为 UTF-8 with BOM，否则 Windows PowerShell 5.1 会按
#       GBK 读取而导致中文乱码 + 语法错误。
#
# 用法：
#   powershell -File run_motor.ps1
#   pwsh -File run_motor.ps1
#   指定 ModelSim 目录：-ModelSim 'D:\Apps\modeltech64_10.6d\win64'

[CmdletBinding()]
param(
    [string]$ModelSim = 'D:\Apps\modeltech64_10.6d\win64',
    [string]$Library  = 'work'
)

$ErrorActionPreference = 'Stop'

$vlib = Join-Path $ModelSim 'vlib.exe'
$vlog = Join-Path $ModelSim 'vlog.exe'
$vsim = Join-Path $ModelSim 'vsim.exe'

foreach ($exe in @($vlib, $vlog, $vsim)) {
    if (-not (Test-Path $exe)) {
        throw "找不到 $exe`n用 -ModelSim 参数指定 ModelSim 的 win64 目录。"
    }
}

Set-Location $PSScriptRoot

# ---------- 源文件 ----------
$sources = @(
    # 被测 RTL（4 个原子模块 + 单路封装 + 多路集成）
    '..\rtl\motor_pwm.v',
    '..\rtl\quad_decoder.v',
    '..\rtl\speed_measure.v',
    '..\rtl\stall_detect.v',
    '..\rtl\motor_unit.v',
    '..\rtl\motor_top.v',
    # 验证环境
    'motor_pwm_tb.v',
    'quad_decoder_tb.v',
    'speed_measure_tb.v',
    'stall_detect_tb.v',
    'motor_top_tb.v'
)

$tests = @(
    @{ Name = 'motor_pwm';     Top = 'motor_pwm_tb';     Desc = '单路 PWM：占空比/方向/刹车/使能/安全态' },
    @{ Name = 'quad_decoder';  Top = 'quad_decoder_tb';  Desc = '正交解码：4 倍频/方向/清零/去毛刺/非法跳变' },
    @{ Name = 'speed_measure'; Top = 'speed_measure_tb'; Desc = '定窗口测速：窗口长度/正负增量/清零基准复位' },
    @{ Name = 'stall_detect';  Top = 'stall_detect_tb';  Desc = '★ 堵转保护：净位移判据（振动判堵转 / 低速不误报）' },
    @{ Name = 'motor_top';     Top = 'motor_top_tb';     Desc = '★ 四路集成：STBY 门控 + 复位安全态 + 四路独立 + 堵转' }
)

# ---------- 建库并编译 ----------
Write-Host ''
Write-Host '================ 编译（motor_tb6612）================' -ForegroundColor Cyan

if (Test-Path $Library) { Remove-Item -Recurse -Force $Library }
& $vlib $Library | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'vlib 建库失败' }

foreach ($f in $sources) {
    $out = & $vlog -work $Library $f 2>&1
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
    $out | ForEach-Object { Write-Host $_ }

    $text = $out -join "`n"
    if     ($text -match 'PASS') { $status = 'PASS' }
    elseif ($text -match 'FAIL') { $status = 'FAIL' }
    else                         { $status = 'NO-RESULT' }

    $results += [pscustomobject]@{ 测试 = $t.Name; 结果 = $status }
}

# ---------- 汇总 ----------
Write-Host ''
Write-Host '================ 汇总（motor_tb6612）================' -ForegroundColor Cyan
$results | Format-Table -AutoSize

$bad = @($results | Where-Object { $_.结果 -ne 'PASS' })
if ($bad.Count -eq 0) {
    Write-Host "全部 $($tests.Count) 个测试通过" -ForegroundColor Green
    exit 0
} else {
    Write-Host "$($bad.Count) 个测试未通过：$(($bad.测试) -join ', ')" -ForegroundColor Red
    exit 1
}
