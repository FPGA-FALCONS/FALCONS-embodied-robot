# program_flash.ps1 —— ACG720 (GW5AT-138B) 位流固化辅助脚本
#
# 用途：把 impl/pnr/falcons.fs 烧进板上 SPI Flash，实现上电脱机运行。
# 环境：Gowin V1.9.12.03 x64（Programmer\bin\programmer_cli.exe）
#
# 用法（PowerShell 7 / pwsh，在本文件所在目录执行）：
#   .\program_flash.ps1 -Action Cables     # 列出已连接的下载器
#   .\program_flash.ps1 -Action Scan       # 扫描链上的 FPGA
#   .\program_flash.ps1 -Action DetectId   # 读外部 Flash 的 ID（先跑这个！）
#   .\program_flash.ps1 -Action Sram       # 只下到 FPGA（掉电即失，先验证功能）
#   .\program_flash.ps1 -Action Program    # ★ 擦除 + 写入 + 校验（固化）
#   .\program_flash.ps1 -Action Verify     # 只校验 Flash 里的内容
#   任何动作加 -DryRun 只打印命令不执行
#
# 编码：本文件必须保存为 UTF-8 with BOM。Windows PowerShell 5.1 会把「无 BOM 的
#       UTF-8」按 GBK 读取，中文注释变乱码并直接导致语法错误、脚本无法运行。
#
# 注意：Program 之前必须先用 Sram 验证过功能正常；固化后断电、拔下载器再上电。
#
# 来源：2026-10-15 自 class\car_fpga\tools\program_flash.ps1 迁入本工程。
#       迁入时只改了一处默认值（imu_demo.fs → falcons.fs，本工程的位流名），
#       以及下面这段说明；脚本主体（含"展开 @cliArgs 而不是 @args"的修正）原样保留。
#       ⚠ 本文件必须保持 UTF-8 with BOM，改完请复核前 3 字节是 EF BB BF。

[CmdletBinding()]
param(
    [ValidateSet('Cables', 'Scan', 'DetectId', 'Sram', 'Program', 'Verify')]
    [string]$Action = 'Scan',

    [string]$Fs,

    [string]$Device = 'GW5AT-138B',
    [string]$Cable  = 'Gowin USB Cable(FT2CH)',

    # 手动指定 programmer_cli.exe 的完整路径（自动搜索失败时用）
    [string]$Programmer,

    # 指定用哪块下载器（多块时），留空则用默认
    [int]$CableIndex = -1,

    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'

# ---------- 0. 默认位流路径 ----------
# 注意：不能写在 param 块的默认值里。Windows PowerShell 5.1 在求值 param 默认值
# 时 $PSScriptRoot 还是空的（PowerShell 7 才行），会报 "Cannot bind argument to
# parameter 'Path' because it is an empty string"。这里在脚本主体里再算。
if (-not $Fs) {
    $Fs = Join-Path (Split-Path -Parent $PSScriptRoot) 'impl\pnr\falcons.fs'
}

# ---------- 1. 定位 programmer_cli.exe ----------
function Find-Programmer {
    $roots = @('D:\Apps\Gowin', 'C:\Gowin', 'D:\Gowin', 'D:\Programs\Gowin')
    foreach ($r in $roots) {
        if (-not (Test-Path $r)) { continue }
        $hit = Get-ChildItem $r -Directory -ErrorAction SilentlyContinue |
               Sort-Object Name -Descending |
               ForEach-Object { Join-Path $_.FullName 'Programmer\bin\programmer_cli.exe' } |
               Where-Object { Test-Path $_ } |
               Select-Object -First 1
        if ($hit) { return $hit }
    }
    throw '没找到 programmer_cli.exe，请用 -Programmer 参数手动指定，或直接从 Gowin GUI 里固化。'
}

if ($Programmer) {
    if (-not (Test-Path $Programmer)) { throw "指定的 programmer_cli.exe 不存在：$Programmer" }
    $cli = (Resolve-Path $Programmer).Path
} else {
    $cli = Find-Programmer
}
$env:GOWINHOME = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $cli))

# ---------- 2. 组装参数 ----------
$opMap = @{
    'Cables'   = @('--scan-cables', 'F')
    'Scan'     = @('--scan')
    'DetectId' = @('-d', $Device, '-r', '51')                       # 51: exFlash Detect ID
    'Sram'     = @('-d', $Device, '-r', '2',  '-f', $Fs)            # 2 : SRAM Program
    'Program'  = @('-d', $Device, '-r', '54', '-f', $Fs)            # 54: exFlash Erase,Program,Verify Arora V
    'Verify'   = @('-d', $Device, '-r', '66')                       # 66: exFlash Verify Arora V
}

$cliArgs = @($opMap[$Action])

if ($Action -in @('Sram', 'Program')) {
    if (-not (Test-Path $Fs)) { throw "位流文件不存在：$Fs（先在 Gowin EDA 里跑完 Place & Route 生成 .fs）" }
    $info = Get-Item $Fs
    Write-Host ("位流：{0}" -f $info.FullName)
    Write-Host ("      生成于 {0}，大小 {1:N1} MB" -f $info.LastWriteTime, ($info.Length / 1MB))
    Write-Host '      确认这个时间晚于你最后一次改 RTL —— 否则烧进去的是旧逻辑。' -ForegroundColor Yellow
}

if ($Action -ne 'Cables' -and $Action -ne 'Scan') {
    $cliArgs += @('--cable', $Cable)
    if ($CableIndex -ge 0) { $cliArgs += @('--cable-index', $CableIndex) }
}

Write-Host ("`n执行：{0} {1}`n" -f $cli, ($cliArgs -join ' ')) -ForegroundColor Cyan

if ($DryRun) { Write-Host '（-DryRun：未真正执行）' -ForegroundColor Yellow; return }

if ($Action -eq 'Program') {
    Write-Host '即将擦除并写入板上配置 Flash。' -ForegroundColor Yellow
    $ans = Read-Host '确认？(yes/N)'
    if ($ans -ne 'yes') { Write-Host '已取消。'; return }
}

# ---------- 3. 执行 ----------
# 注意：这里必须展开 $cliArgs。写成 @args 会展开 PowerShell 的自动变量 $args
#（本脚本用了 param()，$args 恒为空），导致 programmer_cli 不带任何参数执行。
& $cli @cliArgs
$code = $LASTEXITCODE

Write-Host ''
if ($code -eq 0) {
    switch ($Action) {
        'DetectId' { Write-Host '读到了 Flash ID —— 说明板上确有配置 Flash，且能透过 FPGA 访问。' -ForegroundColor Green }
        'Program'  { Write-Host '写入完成。下一步：断电 → 拔掉下载器 → 重新上电 → 看 LED / 串口是否自动工作。' -ForegroundColor Green }
        default    { Write-Host '完成。' -ForegroundColor Green }
    }
} else {
    Write-Host "退出码 $code —— 失败。排查顺序：下载器是否被 Gowin GUI 占用（GUI 开着会锁线）→ 板子是否上电 → Flash 型号是否被支持 → 改用 GUI 的 External Flash Mode 看具体报错。" -ForegroundColor Red
}
exit $code
