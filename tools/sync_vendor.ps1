# sync_vendor.ps1 —— 从源库取最新快照，刷新 vendor/ 与 VENDOR.lock
#
# 做什么（顺序）：
#   1. 只读地检查源库（LIB_ROOT）：存在、是 git 仓库、当前分支与 HEAD；
#   2. 用 `git archive` 导出【被 git 跟踪的内容】到临时目录（绝不拖拽，
#      否则会把 sim/work、transcript、*.qdb 这类未跟踪仿真产物一起搬进来）；
#   3. 校验临时目录（必须有 modules/ 与 README.md，且不含 .git）；
#   4. 原子替换 vendor/verilog-reusable-modules/（先建 staging，成功后才换）；
#   5. 重算逐文件 SHA256 清单 deps/vendor_manifest.sha256 与整体 tree_sha256；
#   6. 重写 VENDOR.lock（源 URL + commit + SHA256 + 包清单）。
#
# ★ 铁律：本脚本【只读】源库，绝不写入、绝不提交源库。
#   模块的修改必须先在源库提交，再用本脚本同步进工程；
#   在 vendor 里直接改模块 = 制造第二份真源，下次同步会被静默覆盖。
#
# 编码：本文件必须保存为 UTF-8 with BOM（PowerShell 5.1 靠 BOM 判编码）。
#
# 用法：
#   pwsh -File tools\sync_vendor.ps1                 # 同步源库 HEAD
#   pwsh -File tools\sync_vendor.ps1 -Ref v1.2.0     # 同步某个 tag / commit
#   pwsh -File tools\sync_vendor.ps1 -Force          # 跳过"vendor 有本地改动"的确认

[CmdletBinding()]
param(
    [string]$Ref = 'HEAD',
    [string]$LibRoot,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'

# 先存下命令行传入的 -LibRoot：下面 dot-source lib_paths.ps1 时它会定义同名变量，
# 必须在这之后把用户显式指定的值放回去（优先级：-LibRoot > 环境变量 > lib_paths 默认值）。
$requestedLibRoot = $LibRoot
. (Join-Path $PSScriptRoot 'lib_paths.ps1')
if ($requestedLibRoot) { $LibRoot = $requestedLibRoot }

function Write-Step { param([string]$Text) Write-Host "==> $Text" -ForegroundColor Cyan }
function Write-Ok   { param([string]$Text) Write-Host "    $Text" -ForegroundColor Green }
function Write-Warn { param([string]$Text) Write-Host "    $Text" -ForegroundColor Yellow }

# ---------- 0. 读上一份快照的记账（用于稍后报告"这次升级变了哪些文件"）----------
$prevCommit = '(首次同步，无旧快照)'
$prevManifest = @{}
if (Test-Path $VendorLock) {
    try { $prevCommit = ((Read-Utf8Text $VendorLock) | ConvertFrom-Json).source.commit_short }
    catch { $prevCommit = '(VENDOR.lock 解析失败)' }
}
if (Test-Path $Manifest) {
    foreach ($l in ((Read-Utf8Text $Manifest) -split "`r?`n")) {
        if ($l.Trim() -eq '') { continue }
        $m0 = [regex]::Match($l, '^([0-9a-fA-F]{64})\s+(.+)$')
        if ($m0.Success) { $prevManifest[$m0.Groups[2].Value] = $m0.Groups[1].Value.ToLower() }
    }
}

# ---------- 1. 只读检查源库 ----------
Write-Step "检查源库（只读，不会写入）"
if (-not (Test-Path $LibRoot))          { throw "源库不存在：$LibRoot`n用 -LibRoot 指定，或设置环境变量 FALCONS_LIB_ROOT。" }
if (-not (Test-Path (Join-Path $LibRoot '.git'))) { throw "源库不是 git 仓库：$LibRoot`n本脚本依赖 git archive，源库必须是 git 仓库。" }
if (-not (Get-Command tar -ErrorAction SilentlyContinue)) { throw "找不到 tar.exe（Windows 10+ 自带 C:\Windows\System32\tar.exe）。" }

$dirty = @(git -C $LibRoot status --porcelain)
if ($dirty.Count -gt 0) {
    Write-Warn "源库工作区有 $($dirty.Count) 处未提交改动 —— git archive 导出的是【已提交内容】，这些改动不会进快照。"
}

$info = @(git -C $LibRoot log -1 --date=short --format="%H`n%h`n%ad`n%s" $Ref)
if ($LASTEXITCODE -ne 0 -or $info.Count -lt 4) { throw "git log 取 $Ref 失败（ref 是否存在？）" }
$commitFull = $info[0].Trim()
$commitShort = $info[1].Trim()
$commitDate = $info[2].Trim()
$subject = $info[3].Trim()
$branchNow = (git -C $LibRoot symbolic-ref --short -q HEAD)
if (-not $branchNow) { $branchNow = '(detached)' }
Write-Ok "源库 $LibRoot"
Write-Ok "分支 $branchNow / ref $Ref / commit $commitShort ($commitDate)"

# ---------- 2. 安全确认：vendor 里是否有本地改动 / 未跟踪文件 ----------
if (Test-Path $VendorRoot) {
    # 只有当 vendor 已经被 git 跟踪时，本地改动才值得拦一下；
    # 尚未跟踪（首次同步）时下面必然是 "?? vendor/"，那不是问题。
    $trackedInVendor = @(git -C $ProjectRoot ls-files -- 'vendor')
    if ($trackedInVendor.Count -eq 0) {
        Write-Ok "vendor/ 尚未被 git 跟踪 —— 跳过本地改动检查。"
    } else {
        $vendorChanges = @(git -C $ProjectRoot status --porcelain -- 'vendor')
        if ($vendorChanges.Count -gt 0) {
            Write-Warn "vendor/ 下有 $($vendorChanges.Count) 处改动或未跟踪文件："
            $vendorChanges | Select-Object -First 10 | ForEach-Object { Write-Warn "      $_" }
            Write-Warn "同步会整体替换 vendor/，上述内容将丢失。"
            if (-not $Force) {
                throw "已中止。确认无误后加 -Force 重跑（未跟踪的仿真产物可以丢；被跟踪文件的改动应当先回提到源库）。"
            }
            Write-Warn "已按 -Force 继续。"
        }
    }
}

# ---------- 3. 导出到 staging ----------
Write-Step "git archive 导出快照"
$tmpTar = Join-Path $env:TEMP ("falcons_vendor_" + [guid]::NewGuid().ToString('N') + ".tar")
$staging = Join-Path (Split-Path -Parent $VendorRoot) ('.staging-' + [guid]::NewGuid().ToString('N'))

try {
    git -C $LibRoot archive --format=tar -o $tmpTar $Ref
    if ($LASTEXITCODE -ne 0) { throw "git archive 失败（exit $LASTEXITCODE）" }

    New-Item -ItemType Directory -Path $staging -Force | Out-Null
    tar -xf $tmpTar -C $staging
    if ($LASTEXITCODE -ne 0) { throw "tar 解压失败（exit $LASTEXITCODE）" }

    # ---------- 4. 校验 staging ----------
    foreach ($must in @('modules', 'README.md', '.gitattributes', '.gitignore')) {
        if (-not (Test-Path (Join-Path $staging $must))) { throw "快照校验失败：缺少 $must —— 导出的不像模块库内容。" }
    }
    if (Test-Path (Join-Path $staging '.git')) { throw "快照里含 .git —— 会让工程仓库把它当嵌套仓库（gitlink）处理，必须剔除。" }
    $stagingFiles = @(Get-ChildItem -Recurse -File $staging)
    if ($stagingFiles.Count -lt 50) { throw "快照只有 $($stagingFiles.Count) 个文件，明显偏少，中止替换。" }

    $newPkgs = @(Get-ChildItem (Join-Path $staging 'modules') -Directory | Sort-Object Name | Select-Object -ExpandProperty Name)
    $oldPkgs = @()
    if (Test-Path (Join-Path $VendorRoot 'modules')) {
        $oldPkgs = @(Get-ChildItem (Join-Path $VendorRoot 'modules') -Directory | Sort-Object Name | Select-Object -ExpandProperty Name)
    }

    # ---------- 5. 原子替换 ----------
    Write-Step "替换 vendor 快照（$($stagingFiles.Count) 个文件）"
    if (Test-Path $VendorRoot) { Remove-Item -Recurse -Force $VendorRoot }
    Move-Item $staging $VendorRoot
    Write-Ok "已更新 $VendorRoot"

    $added   = @($newPkgs | Where-Object { $oldPkgs -notcontains $_ })
    $removed = @($oldPkgs | Where-Object { $newPkgs -notcontains $_ })
    if ($added.Count)   { Write-Ok ("新增包：" + ($added -join ', ')) }
    if ($removed.Count) { Write-Warn ("移除包：" + ($removed -join ', ') + " —— 若工程正在用，构建会立刻失败。") }
    if (-not $added.Count -and -not $removed.Count) { Write-Ok "包清单未变化（$($newPkgs.Count) 个包）" }
} finally {
    if (Test-Path $tmpTar)  { Remove-Item $tmpTar -Force -ErrorAction SilentlyContinue }
    if (Test-Path $staging) { Remove-Item -Recurse -Force $staging -ErrorAction SilentlyContinue }
}

# ---------- 6. 重算 manifest + tree hash ----------
Write-Step "重算 SHA256 清单"
$rootFull = (Resolve-Path $VendorRoot).Path
$files = @(Get-ChildItem -Recurse -File $VendorRoot | Sort-Object FullName)
$lines = foreach ($f in $files) {
    $rel = $f.FullName.Substring($rootFull.Length + 1).Replace('\', '/')
    "$((Get-FileHash $f.FullName -Algorithm SHA256).Hash.ToLower())  $rel"
}
$manifestText = ($lines -join "`n") + "`n"
Write-Utf8NoBom -Path $Manifest -Text $manifestText

$sha = [System.Security.Cryptography.SHA256]::Create()
$treeHash = (($sha.ComputeHash([System.Text.Encoding]::UTF8.GetBytes($manifestText))) | ForEach-Object { $_.ToString('x2') }) -join ''
Write-Ok "deps/vendor_manifest.sha256（$($files.Count) 个文件）"
Write-Ok "tree_sha256 = $treeHash"

# ---------- 6b. 与上一份快照对比：升级时最该看到的就是"到底变了哪些文件" ----------
if ($prevManifest.Count -gt 0) {
    Write-Step "与上一份快照对比（$prevCommit → $commitShort）"

    $newMap = @{}
    foreach ($l in ($manifestText -split "`r?`n")) {
        if ($l.Trim() -eq '') { continue }
        $m1 = [regex]::Match($l, '^([0-9a-fA-F]{64})\s+(.+)$')
        if ($m1.Success) { $newMap[$m1.Groups[2].Value] = $m1.Groups[1].Value.ToLower() }
    }
    $addedF    = @($newMap.Keys        | Where-Object { -not $prevManifest.ContainsKey($_) } | Sort-Object)
    $removedF  = @($prevManifest.Keys  | Where-Object { -not $newMap.ContainsKey($_) }        | Sort-Object)
    $modifiedF = @($newMap.Keys        | Where-Object { $prevManifest.ContainsKey($_) -and $prevManifest[$_] -ne $newMap[$_] } | Sort-Object)

    $refPkgs = @(Get-ReferencedPackages)
    $isRef = {
        param([string]$Rel)
        foreach ($p in $refPkgs) { if ($Rel -like "modules/$p/*") { return $true } }
        return $false
    }

    if (($addedF.Count + $removedF.Count + $modifiedF.Count) -eq 0) {
        Write-Ok "快照内容无变化（只有 commit 变了）"
    } else {
        $groups = @(
            @{ Name = '内容变化'; Set = $modifiedF },
            @{ Name = '新增    '; Set = $addedF },
            @{ Name = '删除    '; Set = $removedF }
        )
        foreach ($g in $groups) {
            if ($g.Set.Count -eq 0) { continue }
            $refHit = @($g.Set | Where-Object { & $isRef $_ })
            $head = "    $($g.Name) $($g.Set.Count) 个文件"
            if ($refHit.Count -gt 0) { $head += "   ← 其中 $($refHit.Count) 个属【工程引用包】★" }
            Write-Host $head -ForegroundColor $(if ($refHit.Count -gt 0) { 'Yellow' } else { 'Gray' })
            $g.Set | Select-Object -First 12 | ForEach-Object {
                $mark = if (& $isRef $_) { '★' } else { ' ' }
                Write-Host "        $mark $_"
            }
            if ($g.Set.Count -gt 12) { Write-Host "          …（还有 $($g.Set.Count - 12) 个）" }
        }
        $refChanged = @(@($addedF + $removedF + $modifiedF) | Where-Object { & $isRef $_ })
        if ($refChanged.Count -gt 0) {
            Write-Warn "★ 工程引用的包有 $($refChanged.Count) 个文件发生变化 —— 应用层（rtl\）可能需要跟着改；"
            Write-Warn "  若涉及端口名/参数名/默认值，先看该包 README 顶部的说明，改完务必跑 tools\build.ps1。"
        }
    }
}

# ---------- 7. 重写 VENDOR.lock ----------
Write-Step "重写 VENDOR.lock"
function ConvertTo-JsonString { param([string]$s) '"' + ($s -replace '\\','\\' -replace '"','\"') + '"' }
$pkgJson = ($newPkgs | ForEach-Object { '      ' + (ConvertTo-JsonString $_) }) -join ",`n"
$stamp = (Get-Date).ToString('yyyy-MM-ddTHH:mm:sszzz')

$lock = @"
{
  "_comment": "本文件由 tools/sync_vendor.ps1 自动生成，用于把工程钉死在一份可复现的模块库快照上。手工改它没有意义（下次同步会覆盖）；要升级就重跑 sync_vendor.ps1。",
  "lock_version": 1,
  "generated_at": "$stamp",
  "generated_by": "tools/sync_vendor.ps1",
  "source": {
    "url": "$LibUrl",
    "lib_root_local": "$($LibRoot.Replace('\','/'))",
    "branch": "$branchNow",
    "ref": "$Ref",
    "commit": "$commitFull",
    "commit_short": "$commitShort",
    "commit_date": "$commitDate",
    "subject": $(ConvertTo-JsonString $subject)
  },
  "snapshot": {
    "path": "vendor/verilog-reusable-modules",
    "export_method": "git archive --format=tar $Ref  (解压到 vendor/，不含 .git)",
    "file_count": $($files.Count),
    "tree_sha256": "$treeHash",
    "manifest": "deps/vendor_manifest.sha256",
    "packages": [
$pkgJson
    ]
  },
  "referenced_by_project": "deps/modules.list（工程实际引用的包与依赖闭包）"
}
"@
Write-Utf8NoBom -Path $VendorLock -Text $lock
Write-Ok "VENDOR.lock：commit $commitShort / tree $($treeHash.Substring(0,12))…"

# ---------- 8. 收尾提示 ----------
Write-Host ''
Write-Host '同步完成。建议接着跑：' -ForegroundColor Cyan
Write-Host '    pwsh -File tools\check_vendor.ps1        # 校验快照完整性、是否偏离源库'
Write-Host '    pwsh -File tools\run_all_modules.ps1     # 库里自带测试当门槛，逐包跑一遍'
Write-Host ''
Write-Warn "若源库有 ⚠ BREAKING 提交，应用层（rtl/）需要跟着改 —— 见各包 README 顶部的说明。"
