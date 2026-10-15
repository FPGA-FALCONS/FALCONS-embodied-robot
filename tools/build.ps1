# build.ps1 —— 一键构建：先体检 vendor 快照，再调 Gowin 综合 + 布局布线
#
# 为什么要把体检放在构建之前自动执行：
#   若 vendor 里的模块被谁"就地改过"，构建会成功、结果却不可复现，
#   后面排查问题时最贵的就是这一步。所以让它挡在前面。
#
# 编码：本文件必须保存为 UTF-8 with BOM。
#
# 用法：
#   powershell -File tools\build.ps1
#   powershell -File tools\build.ps1 -Tcl build_speed.tcl      # 分阶段工程
#   powershell -File tools\build.ps1 -SkipCheck                # 确认快照有意的改动时跳过体检

[CmdletBinding()]
param(
    [string]$Tcl = 'build.tcl',
    [string]$GowinSh,
    [switch]$SkipCheck
)

$ErrorActionPreference = 'Stop'
$requestedGowin = $GowinSh
. (Join-Path $PSScriptRoot 'lib_paths.ps1')
if ($requestedGowin) { $GowinSh = $requestedGowin } else { $GowinSh = $GowinShDefault }

# ---------- 1. 体检 ----------
if (-not $SkipCheck) {
    Write-Host ''
    Write-Host '===== 构建前体检：tools\check_vendor.ps1 =====' -ForegroundColor Cyan
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'check_vendor.ps1')
    if ($LASTEXITCODE -ne 0) {
        throw "快照体检未通过（退出码 $LASTEXITCODE）—— 先解决上面的 [BAD] 项；确认是有意改动再加 -SkipCheck。"
    }
} else {
    Write-Host '（已按 -SkipCheck 跳过体检）' -ForegroundColor Yellow
}

# ---------- 2. 构建 ----------
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

# ---------- 3. 报告产物 ----------
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
