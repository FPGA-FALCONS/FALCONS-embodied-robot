# build.ps1 —— 一键构建：调 Gowin 综合 + 布局布线
#
# 用法：
#   powershell -File tools\build.ps1
#   powershell -File tools\build.ps1 -Tcl build_speed.tcl      # 分阶段工程
#
# 注：vendor 里的模块自 2026-10-15 起由本工程直接维护（见 vendor\ORIGIN.md），
#     所以不再有"构建前体检快照"这一步 —— 原来的 check_vendor.ps1 已删除。
#     ★ 改了 vendor 里的模块后，请先跑 sim\run_sim.ps1 做回归，再来构建。
#
# 编码：本文件必须保存为 UTF-8 with BOM。

[CmdletBinding()]
param(
    [string]$Tcl = 'build.tcl',
    [string]$GowinSh
)

$ErrorActionPreference = 'Stop'
$requestedGowin = $GowinSh
. (Join-Path $PSScriptRoot 'lib_paths.ps1')
if ($requestedGowin) { $GowinSh = $requestedGowin } else { $GowinSh = $GowinShDefault }

# ---------- 构建 ----------
$tclPath = Join-Path $ProjectRoot $Tcl
if (-not (Test-Path $GowinSh)) { throw "找不到 gw_sh.exe：$GowinSh`n用 -GowinSh 指定，或改 tools\lib_paths.ps1。" }
if (-not (Test-Path $tclPath)) { throw "工程根下找不到 $Tcl（当前工程根：$ProjectRoot）。" }

Write-Host ''
Write-Host "===== 构建：$GowinSh $Tcl =====" -ForegroundColor Cyan
Push-Location $ProjectRoot
try {
    & $GowinSh $Tcl
    $buildExit = $LASTEXITCODE
} finally {
    Pop-Location
}

# ---------- 报告产物 ----------
Write-Host ''
$pnr = Join-Path $ProjectRoot 'impl\pnr'
if (Test-Path $pnr) {
    $fs = @(Get-ChildItem $pnr -Filter '*.fs' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
    if ($fs.Count -gt 0) {
        Write-Host '比特流产物：' -ForegroundColor Green
        $fs | Select-Object -First 5 | ForEach-Object {
            Write-Host ("  {0}  ({1:N1} MB, {2})" -f $_.FullName, ($_.Length/1MB), $_.LastWriteTime)
        }
    }
    $rpt = @(Get-ChildItem $pnr -Filter '*_syn.rpt.html' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
    if ($rpt.Count -gt 0) { Write-Host ("综合报告：{0}" -f $rpt[0].FullName) -ForegroundColor Green }
}

if ($buildExit -ne 0) { Write-Host "构建失败（gw_sh 退出码 $buildExit）。" -ForegroundColor Red; exit $buildExit }
Write-Host '构建完成。' -ForegroundColor Green
exit 0