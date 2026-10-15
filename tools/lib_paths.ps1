# lib_paths.ps1 —— 本项目【所有路径的唯一定义处】
#
# 用法（在其它脚本里 dot-source，不要直接运行）：
#     . (Join-Path $PSScriptRoot 'lib_paths.ps1')
#
# 为什么要有这个文件：
#   本项目吃过"三处各写一份路径、改一处漏两处"的亏。Gowin 的 build*.tcl、
#   ModelSim 的 run_*.ps1、check_vendor / sync_vendor 全部从这里取路径，
#   改目录结构时只改本文件。
#
# 编码：本文件必须保存为 UTF-8 with BOM，否则 Windows PowerShell 5.1
#       会按 GBK 读取而导致中文乱码 + 语法错误。

# ---- 本工程 ----
$ProjectRoot = Split-Path -Parent $PSScriptRoot          # tools\ 的上一级 = 工程根
$VendorRoot  = Join-Path $ProjectRoot 'vendor\verilog-reusable-modules'
$DepsRoot    = Join-Path $ProjectRoot 'deps'
$LogRoot     = Join-Path $ProjectRoot 'logs'
$RtlRoot     = Join-Path $ProjectRoot 'rtl'
$ConstrRoot  = Join-Path $ProjectRoot 'constr'
$SimRoot     = Join-Path $ProjectRoot 'sim'

# ---- 内嵌快照的记账文件 ----
$VendorLock  = Join-Path $ProjectRoot 'VENDOR.lock'
$Manifest    = Join-Path $DepsRoot 'vendor_manifest.sha256'
$ModulesList = Join-Path $DepsRoot 'modules.list'

# ---- 源库（LIB_ROOT）：同步的来源，只在 sync_vendor.ps1 里被写到的才是它 ----
# 优先取环境变量 FALCONS_LIB_ROOT，其次取下面的默认值。
# 注意：本工程【只读】源库（只用 git archive / git log），绝不写入。
$LibRootDefault = 'D:\Programs\FPGA Projects\verilog-reusable-modules'
$LibRoot = if ($env:FALCONS_LIB_ROOT) { $env:FALCONS_LIB_ROOT } else { $LibRootDefault }
$LibUrl  = 'git@github.com:HITSZ-FPGA-FALCONS/verilog-reusable-modules.git'

# ---- 外部工具 ----
$ModelSimDefault = 'D:\Apps\modeltech64_10.6d\win64'
$GowinShDefault  = 'D:\Apps\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe'

# ---- 工具函数 ----
# 读文本文件一律显式指定 UTF-8：
#   PS 5.1 的 Get-Content 默认按系统 ANSI（本机 GBK）解码无 BOM 的 UTF-8，
#   会把汉字续字节后的 0x0A 当 trail byte 吞掉 —— 不报错，只是安静地给出错误结果。
function Read-Utf8Text {
    param([Parameter(Mandatory)][string]$Path)
    return [System.IO.File]::ReadAllText($Path, [System.Text.Encoding]::UTF8)
}

# 写无 BOM 的 UTF-8（.ps1 之外的文本文件用这个；.ps1 必须带 BOM，见 tools/ 说明）
function Write-Utf8NoBom {
    param([Parameter(Mandatory)][string]$Path, [Parameter(Mandatory)][string]$Text)
    $dir = Split-Path -Parent $Path
    if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    [System.IO.File]::WriteAllText($Path, $Text, (New-Object System.Text.UTF8Encoding($false)))
}

# 解析 deps/modules.list，返回被工程引用的包名数组（忽略注释与空行）
function Get-ReferencedPackages {
    param([string]$ListPath = $ModulesList)
    if (-not (Test-Path $ListPath)) { return @() }
    $names = @()
    foreach ($line in (Read-Utf8Text $ListPath) -split "`r?`n") {
        $t = $line.Trim()
        if ($t -eq '' -or $t.StartsWith('#')) { continue }
        $tok = ($t -split '\s+')[0]
        # 只接受合法包名（字母/数字/下划线）—— 防止 modules.list 里的中文续行说明被当成包名
        if ($tok -notmatch '^[A-Za-z0-9_]+$') { continue }
        $names += $tok
    }
    return $names
}
