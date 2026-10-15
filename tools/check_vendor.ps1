# check_vendor.ps1 —— 内嵌快照体检（放在构建之前自动执行）
#
# 报告三件事（只报告，不改任何东西）：
#   1. 内嵌目录是否被本地改动过（逐文件 SHA256 与 deps/vendor_manifest.sha256 比对）；
#   2. 相对源库落后多少个提交（源库不可达时跳过，不算失败）；
#   3. ★ 实际被工程引用的那几个包（deps/modules.list）是否有变化 —— 只高亮相关的，不刷屏。
#
# 退出码：0 = 快照与 VENDOR.lock 一致；1 = 有改动 / 有文件缺失 / 有未被忽略的多余文件。
#         "落后源库"只警告，不改退出码（除非加 -FailOnOutdated）。
#
# 编码：本文件必须保存为 UTF-8 with BOM。
#
# 用法：
#   powershell -File tools\check_vendor.ps1
#   powershell -File tools\check_vendor.ps1 -FailOnOutdated

[CmdletBinding()]
param(
    [string]$LibRoot,
    [switch]$FailOnOutdated,
    [int]$MaxListed = 20
)

$ErrorActionPreference = 'Stop'
$requestedLibRoot = $LibRoot
. (Join-Path $PSScriptRoot 'lib_paths.ps1')
if ($requestedLibRoot) { $LibRoot = $requestedLibRoot }

$fail = 0

function Write-Head { param([string]$T) Write-Host ''; Write-Host "===== $T =====" -ForegroundColor Cyan }
function Write-Ok   { param([string]$T) Write-Host "  [OK]   $T" -ForegroundColor Green }
function Write-Bad  { param([string]$T) Write-Host "  [BAD]  $T" -ForegroundColor Red }
function Write-Warn2{ param([string]$T) Write-Host "  [WARN] $T" -ForegroundColor Yellow }

# ---------- 前置 ----------
if (-not (Test-Path $VendorLock)) { throw "找不到 VENDOR.lock —— 先跑 tools\sync_vendor.ps1 生成快照。" }
if (-not (Test-Path $Manifest))   { throw "找不到 deps\vendor_manifest.sha256 —— 先跑 tools\sync_vendor.ps1。" }
if (-not (Test-Path $VendorRoot)) { throw "找不到内嵌目录 $VendorRoot —— 先跑 tools\sync_vendor.ps1。" }

$lock = (Read-Utf8Text $VendorLock) | ConvertFrom-Json
$refPkgs = @(Get-ReferencedPackages)

Write-Host ''
Write-Host 'FALCONS-embodied-robot 内嵌快照体检' -ForegroundColor White
Write-Host ("  锁定源    : {0}" -f $lock.source.url)
Write-Host ("  锁定 commit: {0}  ({1}, {2})" -f $lock.source.commit_short, $lock.source.commit_date, $lock.source.branch)
Write-Host ("  快照       : {0} 个文件 / tree {1}" -f $lock.snapshot.file_count, $lock.snapshot.tree_sha256.Substring(0,16))
Write-Host ("  工程引用包 : {0}" -f ($refPkgs -join ', '))

# ---------- 1. 逐文件比对 ----------
Write-Head '1. 内嵌目录是否被本地改动过（SHA256 逐文件比对）'

$expected = @{}
foreach ($line in ((Read-Utf8Text $Manifest) -split "`r?`n")) {
    if ($line.Trim() -eq '') { continue }
    $m = [regex]::Match($line, '^([0-9a-fA-F]{64})\s+(.+)$')
    if ($m.Success) { $expected[$m.Groups[2].Value] = $m.Groups[1].Value.ToLower() }
}
if ($expected.Count -eq 0) { throw "manifest 解析出 0 条记录，文件可能损坏。" }

$modified = @()
$missing  = @()
foreach ($rel in $expected.Keys) {
    $full = Join-Path $VendorRoot ($rel -replace '/', '\')
    if (-not (Test-Path $full)) { $missing += $rel; continue }
    $h = (Get-FileHash $full -Algorithm SHA256).Hash.ToLower()
    if ($h -ne $expected[$rel]) { $modified += $rel }
}
$modified = @($modified | Sort-Object)
$missing  = @($missing  | Sort-Object)

# 多余文件：在 vendor 里但不在 manifest 里（应为仿真产物）
$onDisk = @(Get-ChildItem -Recurse -File $VendorRoot | ForEach-Object {
    $_.FullName.Substring((Resolve-Path $VendorRoot).Path.Length + 1).Replace('\','/')
})
$extra = @($onDisk | Where-Object { -not $expected.ContainsKey($_) } | Sort-Object)

# 多余文件里"未被 .gitignore 忽略"的才是真问题（会被 git 收进仓库、污染快照）
$extraNotIgnored = @()
if ($extra.Count -gt 0) {
    $extraSet = @{}
    foreach ($p in $extra) { $extraSet[$p] = $true }
    $untracked = @(git -C $ProjectRoot status --porcelain --untracked-files=all -- 'vendor' 2>$null)
    foreach ($line in $untracked) {
        if ($line -match '^\?\?\s+(.+)$') {
            $p = $matches[1].Trim() -replace '\\','/'
            # 只关心"不在 manifest 里、又没被忽略"的文件；
            # vendor 尚未被 git 跟踪时 status 会把已内嵌文件也列成未跟踪，必须排除掉。
            if ($extraSet.ContainsKey($p)) { $extraNotIgnored += $p }
        }
    }
}

if ($modified.Count -eq 0 -and $missing.Count -eq 0) {
    Write-Ok "$($expected.Count) 项逐一比对：全部与 VENDOR.lock 记录一致（未被本地改动）"
} else {
    if ($modified.Count) {
        Write-Bad "$($modified.Count) 个文件内容与锁定快照不一致（疑似在 vendor 里直接改了模块）："
        $modified | Select-Object -First $MaxListed | ForEach-Object { Write-Host "           $_" }
        if ($modified.Count -gt $MaxListed) { Write-Host "           …（还有 $($modified.Count - $MaxListed) 个）" }
    }
    if ($missing.Count) {
        Write-Bad "$($missing.Count) 个文件缺失："
        $missing | Select-Object -First $MaxListed | ForEach-Object { Write-Host "           $_" }
    }
    Write-Host '   恢复办法：重跑 tools\sync_vendor.ps1（会整体替换 vendor/）；' -ForegroundColor Yellow
    Write-Host '   若改动本来就有价值 —— 按铁律应先在【源库】提交，再同步进来。' -ForegroundColor Yellow
    $fail = 1
}
if ($extra.Count -gt 0) {
    Write-Ok "另有 $($extra.Count) 个 manifest 之外的文件（仿真/构建产物，正常；若其中有该进仓库的文件请查 .gitignore）"
    if ($extraNotIgnored.Count -gt 0) {
        Write-Bad "其中 $($extraNotIgnored.Count) 个【未被 .gitignore 忽略】—— 会被 git 当成快照的一部分："
        $extraNotIgnored | Select-Object -First $MaxListed | ForEach-Object { Write-Host "           $_" }
        $fail = 1
    }
}

# ---------- 2. 落后源库多少提交 ----------
Write-Head '2. 相对源库的进度（只读源库）'
if (-not (Test-Path $LibRoot) -or -not (Test-Path (Join-Path $LibRoot '.git'))) {
    Write-Warn2 "源库不可达（$LibRoot）—— 跳过落后检查（不影响快照可用性）。"
} else {
    $lockedCommit = $lock.source.commit
    git -C $LibRoot cat-file -e "$lockedCommit^{commit}" 2>$null
    $hasCommit = ($LASTEXITCODE -eq 0)
    if (-not $hasCommit) {
        Write-Warn2 "源库里找不到锁定的 commit $($lock.source.commit_short)（可能被 rebase / 强推过）。"
        $fail = 1
    } else {
        $behind = [int](git -C $LibRoot rev-list --count "$lockedCommit..HEAD" -- modules README.md THIRD_PARTY_NOTICES.md)
        $headShort = (git -C $LibRoot rev-parse --short HEAD)
        if ($behind -eq 0) {
            Write-Ok "已是最新：锁定 commit = 源库 HEAD（$headShort）"
        } else {
            Write-Warn2 "落后源库 $behind 个提交（本地 $($lock.source.commit_short) → 源库 $headShort）："
            git -C $LibRoot log --oneline --no-decorate "$lockedCommit..HEAD" -- modules README.md THIRD_PARTY_NOTICES.md |
                Select-Object -First 10 | ForEach-Object { Write-Host "           $_" }
            Write-Host '           升级办法：tools\sync_vendor.ps1（升级后请重跑 tools\run_all_modules.ps1）' -ForegroundColor Yellow
            if ($FailOnOutdated) { $fail = 1 }
        }
        $breaking = @(git -C $LibRoot log --oneline --no-decorate "$lockedCommit..HEAD" -- modules | Where-Object { $_ -match 'BREAKING' })
        if ($breaking.Count -gt 0) {
            Write-Warn2 "源库有 $($breaking.Count) 个标了 ⚠ BREAKING 的提交 —— 升级后应用层必须跟着改（提示，不算体检失败）："
            $breaking | ForEach-Object { Write-Host "           $_" }
        }
    }
    $libDirty = @(git -C $LibRoot status --porcelain)
    if ($libDirty.Count -gt 0) {
        Write-Warn2 "源库工作区有 $($libDirty.Count) 处未提交改动（只提示，本脚本不写入源库）。"
    }
}

# ---------- 3. 只高亮"工程引用包"的变化 ----------
Write-Head '3. 工程引用的包是否有变化（deps/modules.list）'
$changedAll = @($modified + $missing)
$hitRef = @()
$hitOther = @()
foreach ($rel in $changedAll) {
    $isRef = $false
    foreach ($pkg in $refPkgs) {
        if ($rel -like "modules/$pkg/*") { $isRef = $true; break }
    }
    if ($isRef) { $hitRef += $rel } else { $hitOther += $rel }
}

if ($changedAll.Count -eq 0) {
    Write-Ok "工程引用的包（$($refPkgs.Count) 个）在内嵌目录里与锁定快照完全一致"
} elseif ($hitRef.Count -gt 0) {
    Write-Bad "★ 工程正在引用的包里有 $($hitRef.Count) 个文件与快照不一致 —— 构建结果不再可复现："
    $hitRef | ForEach-Object { Write-Host "           $_" }
} else {
    Write-Warn2 "变化都落在本工程【未引用】的包里，不影响当前构建："
    $hitOther | Select-Object -First $MaxListed | ForEach-Object { Write-Host "           $_" }
}

# ---------- 结论 ----------
Write-Host ''
if ($fail -eq 0) {
    Write-Host '结论：快照一致，可以构建。' -ForegroundColor Green
    exit 0
} else {
    Write-Host '结论：快照有问题（见上面 [BAD]）—— 先解决再构建。' -ForegroundColor Red
    exit 1
}
