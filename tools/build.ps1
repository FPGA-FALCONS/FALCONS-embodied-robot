# build.ps1 —— 一键构建：调 Gowin 综合 + 布局布线
#
# 用法：
#   powershell -File tools\build.ps1                          # 全量：综合 + 布局布线（约 8.6 分钟）
#   powershell -File tools\build.ps1 -Tcl build_syn.tcl       # 只综合（约 2 分钟，看资源与告警）
#   powershell -File tools\build.ps1 -Tcl build_pnr.tcl       # 只布局布线（约 6.6 分钟，改约束不改 RTL 时用）
#   powershell -File tools\build.ps1 -Tcl build_rc_led.tcl    # 遥控器单独验证位流
#
# 三个生产入口的分工、以及"Gowin 1.9.12 没有增量综合 / 增量 PnR / 并行开关"的实测结论，
# 见 doc\工作记录.md §67。不要为了"省时间"去改 run 的阶段名 —— 只有 all / syn / pnr 三个合法。
#
# 注：vendor 里的模块自 2026-10-15 起由本工程直接维护（见 vendor\ORIGIN.md），
#     所以不再有"构建前体检快照"这一步 —— 原来的 check_vendor.ps1 已删除。
#     ★ 改了 vendor 里的模块后，请先跑 sim\run_sim.ps1 做回归，再来构建。
#
# 编码：本文件必须保存为 UTF-8 with BOM。

[CmdletBinding()]
param(
    [string]$Tcl = 'build.tcl',
    [string]$GowinSh,
    [string]$ArtifactDirectory = 'impl\pnr'
)

$ErrorActionPreference = 'Stop'
$requestedGowin = $GowinSh
. (Join-Path $PSScriptRoot 'lib_paths.ps1')
if ($requestedGowin) { $GowinSh = $requestedGowin } else { $GowinSh = $GowinShDefault }

# ---------- 构建 ----------
$tclPath = Join-Path $ProjectRoot $Tcl
if (-not (Test-Path $GowinSh)) { throw "找不到 gw_sh.exe：$GowinSh`n用 -GowinSh 指定，或改 tools\lib_paths.ps1。" }
if (-not (Test-Path $tclPath)) { throw "工程根下找不到 $Tcl（当前工程根：$ProjectRoot）。" }

# 本轮是否应该产出新位流（只有 run all 的两支会更新 .fs）
$producesBitstream = @('build.tcl', 'build_rc_led.tcl') -contains $Tcl
$bitstreamBefore = @{}
$pnr = Join-Path $ProjectRoot $ArtifactDirectory
if (Test-Path $pnr) {
    Get-ChildItem $pnr -Filter '*.fs' -ErrorAction SilentlyContinue | ForEach-Object { $bitstreamBefore[$_.FullName] = $_.LastWriteTime }
}

Write-Host ''
Write-Host "===== 构建：$GowinSh $Tcl =====" -ForegroundColor Cyan
$sw = [Diagnostics.Stopwatch]::StartNew()
Push-Location $ProjectRoot
try {
    & $GowinSh $Tcl
    $buildExit = $LASTEXITCODE
} finally {
    Pop-Location
    $sw.Stop()
}
Write-Host ''
Write-Host ("耗时 {0:mm\:ss}（{1:N1} 秒）" -f $sw.Elapsed, $sw.Elapsed.TotalSeconds) -ForegroundColor Cyan

# ---------- 报告产物 ----------
Write-Host ''
if (Test-Path $pnr) {
    $fs = @(Get-ChildItem $pnr -Filter '*.fs' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
    if ($fs.Count -gt 0) {
        if ($producesBitstream) {
            Write-Host '比特流产物：' -ForegroundColor Green
        } else {
            Write-Host '比特流产物（本轮没跑 run all，下面列的是历史位流，看时间戳）：' -ForegroundColor Yellow
        }
        $fs | Select-Object -First 5 | ForEach-Object {
            $unchanged = (-not $producesBitstream) -and $bitstreamBefore.ContainsKey($_.FullName) -and ($bitstreamBefore[$_.FullName] -eq $_.LastWriteTime)
            $mark = if ($unchanged) { '  (未变)' } else { '' }
            Write-Host ("  {0}  ({1:N1} MB, {2}){3}" -f $_.FullName, ($_.Length/1MB), $_.LastWriteTime, $mark)
        }
    }
}
$synDir = Join-Path $ProjectRoot 'impl\gwsynthesis'
if (Test-Path $synDir) {
    $rsc = @(Get-ChildItem $synDir -Filter 'falcons_syn_resource.html' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
    if ($rsc.Count -gt 0) { Write-Host ("综合资源：{0}" -f $rsc[0].FullName) -ForegroundColor Green }
}
$rpt = @(Get-ChildItem $pnr -Filter '*_syn.rpt.html' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
if ($rpt.Count -gt 0) { Write-Host ("综合报告：{0}" -f $rpt[0].FullName) -ForegroundColor Green }

if ($buildExit -ne 0) { Write-Host "构建失败（gw_sh 退出码 $buildExit）。" -ForegroundColor Red; exit $buildExit }
Write-Host '构建完成。' -ForegroundColor Green
exit 0