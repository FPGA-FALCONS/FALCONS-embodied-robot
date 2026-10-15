# run_all_modules.ps1 —— 把 vendor 里各模块自带的测试全跑一遍（"模块自带测试当门槛"）
#
# 为什么需要它：
#   vendor 里的模块都自带 sim\ 与测试入口，价值一半就在这里。改了某个模块、或改动它的
#   使用方式之后，先跑一遍库自带测试，能立刻区分"是我的应用层写错了"还是
#   "模块本身有问题"。
#
# 怎么跑（两个关键点）：
#   1. ModelSim 在 -c（命令行）模式下，tb 里的 $finish 会【终结整个 vsim 进程】。
#      因此不能把多个 tb 串在同一个 do 脚本里跑 —— 那样只有第一个执行，后面静默跳过。
#      本脚本的做法是：包里有跑在命令行的 run_*.ps1 就直接调用它（它自己已按
#      每测试一个进程实现）；没有的话，每个 .do 入口开一个独立的 vsim 进程。
#   2. 每个入口的完整输出都落盘到 logs\ 下（.gitignore 已忽略），便于事后查证。
#
# 产物说明：跑测试会在 vendor 的包 sim 目录里生成 work\ / transcript / build\ 等。
#   这些已被 .gitignore 忽略，不会污染 vendor 目录。
#
# 编码：本文件必须保存为 UTF-8 with BOM。
#
# 用法：
#   powershell -File tools\run_all_modules.ps1
#   powershell -File tools\run_all_modules.ps1 -Packages motor_tb6612,imu660ra
#   powershell -File tools\run_all_modules.ps1 -ReferencedOnly      # 只跑 deps\modules.list 里的包

[CmdletBinding()]
param(
    [string[]]$Packages,
    [switch]$ReferencedOnly,
    [string]$ModelSim,
    [int]$TimeoutSec = 1800
)

$ErrorActionPreference = 'Stop'
$requestedModelSim = $ModelSim
. (Join-Path $PSScriptRoot 'lib_paths.ps1')
if ($requestedModelSim) { $ModelSim = $requestedModelSim } else { $ModelSim = $ModelSimDefault }

if (-not (Test-Path $VendorRoot))   { throw "找不到 vendor 目录：$VendorRoot —— 检查 vendor\verilog-reusable-modules 是否完整（见 vendor\ORIGIN.md）。" }
if (-not (Test-Path $LogRoot))      { New-Item -ItemType Directory -Path $LogRoot -Force | Out-Null }

# 归一化 -Packages：
#   用 `powershell -File` 调用时 "a,b" 会整体当成一个参数值（-File 不解析逗号），
#   这里自己按逗号/分号拆一次，两种调用方式（-File 与 -Command）都能用。
if ($Packages) {
    $flat = @()
    foreach ($p in $Packages) { $flat += ($p -split '[,;]') }
    $Packages = @($flat | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne '' })
}

$vsim = Join-Path $ModelSim 'vsim.exe'
$hasModelSim = Test-Path $vsim

# ---------- 选定要跑的包 ----------
$allPkgs = @(Get-ChildItem (Join-Path $VendorRoot 'modules') -Directory | Sort-Object Name | Select-Object -ExpandProperty Name)
$target = $allPkgs
if ($ReferencedOnly) {
    $ref = @(Get-ReferencedPackages)
    $target = @($allPkgs | Where-Object { $ref -contains $_ })
}
if ($Packages) { $target = @($allPkgs | Where-Object { $Packages -contains $_ }) }
if ($target.Count -eq 0) { throw "没有选到任何包（用 -Packages 指定，或检查 deps\modules.list）。" }

# ---------- 组装入口清单 ----------
# 每个入口 = 一个独立进程。
#   1) 包里有能跑在命令行的 run_*.ps1 → 直接调用它（它自己已按"每测试一个进程"实现）；
#   2) 没有可用的 ps1（例如只有 Icarus 入口而本机没装 iverilog）→ 退回用它的 .do 入口，
#      每个 .do 开一个独立的 vsim 进程；
#   3) 两者都没有 → 记 SKIP，让人知道这个包没法自动跑。
$entries = @()
foreach ($pkg in $target) {
    $simDir = Join-Path $VendorRoot "modules\$pkg\sim"
    if (-not (Test-Path $simDir)) {
        $entries += [pscustomobject]@{ Pkg = $pkg; Kind = 'none'; File = ''; SimDir = $simDir; Note = '包里没有 sim\ 目录' }
        continue
    }

    $ps1s = @(Get-ChildItem $simDir -File -Filter 'run_*.ps1' | Sort-Object Name)
    $usablePs1 = @()
    $skippedPs1 = @()
    foreach ($s in $ps1s) {
        $body = Read-Utf8Text $s.FullName
        if ($body -match 'iverilog' -and -not (Get-Command iverilog -ErrorAction SilentlyContinue)) {
            $skippedPs1 += [pscustomobject]@{ Pkg = $pkg; Kind = 'ps1'; File = $s.FullName; SimDir = $simDir; Note = '需要 iverilog（未安装，跳过）' }
        } else {
            $usablePs1 += [pscustomobject]@{ Pkg = $pkg; Kind = 'ps1'; File = $s.FullName; SimDir = $simDir; Note = '' }
        }
    }
    if ($usablePs1.Count -gt 0) { $entries += $usablePs1; continue }

    $dos = @(Get-ChildItem $simDir -File -Filter 'run*.do' | Sort-Object Name)
    if ($dos.Count -gt 0) {
        foreach ($d in $dos) {
            $n = if ($skippedPs1.Count -gt 0) { '（ps1 入口因缺少 iverilog 跳过，改用本 .do 入口）' } else { '' }
            $entries += [pscustomobject]@{ Pkg = $pkg; Kind = 'do'; File = $d.FullName; SimDir = $simDir; Note = $n }
        }
        continue
    }

    if ($skippedPs1.Count -gt 0) { $entries += $skippedPs1; continue }
    $entries += [pscustomobject]@{ Pkg = $pkg; Kind = 'none'; File = ''; SimDir = $simDir; Note = '没有命令行入口（sim\ 下无 run_*.ps1 / run*.do）' }
}

Write-Host ''
Write-Host '================ 跑一遍内嵌模块库的自带测试 ================' -ForegroundColor Cyan
Write-Host ("  vendor   : {0}" -f $VendorRoot)
Write-Host ("  ModelSim : {0}  {1}" -f $ModelSim, $(if ($hasModelSim) { '(已找到)' } else { '(未找到！)' }))
Write-Host ("  待跑入口 : {0} 个（{1} 个包）" -f $entries.Count, $target.Count)

if (-not $hasModelSim) { throw "找不到 $vsim —— 用 -ModelSim 指定 ModelSim 的 win64 目录。" }

# ---------- 逐个入口执行（每个独立进程）----------
$results = @()
foreach ($e in $entries) {
    $label = if ($e.File) { "$($e.Pkg)/$(Split-Path -Leaf $e.File)" } else { "$($e.Pkg)/(无入口)" }
    Write-Host ''
    Write-Host "---- $label ----" -ForegroundColor Cyan

    if ($e.Kind -eq 'none') {
        Write-Host "     跳过：$($e.Note)" -ForegroundColor Yellow
        $results += [pscustomobject]@{ 包 = $e.Pkg; 入口 = '(无)'; 结果 = 'SKIP'; 日志 = $e.Note }
        continue
    }

    if ($e.Kind -eq 'ps1' -and $e.Note -ne '') {
        Write-Host "     跳过：$($e.Note)" -ForegroundColor Yellow
        $results += [pscustomobject]@{ 包 = $e.Pkg; 入口 = (Split-Path -Leaf $e.File); 结果 = 'SKIP'; 日志 = $e.Note }
        continue
    }

    $logFile = Join-Path $LogRoot ("{0}__{1}.log" -f $e.Pkg, ((Split-Path -Leaf $e.File) -replace '\.(ps1|do)$',''))
    $exit = 0
    $out = @()
    Push-Location $e.SimDir
    try {
        if ($e.Kind -eq 'ps1') {
            $out = & powershell -NoProfile -ExecutionPolicy Bypass -File $e.File 2>&1
        } else {
            # 每个 .do 入口一个独立 vsim 进程；脚本内部自己 vlib/vlog/vsim。
            # 两个细节都是踩出来的：
            #   1) 路径必须正斜杠化并用 Tcl 大括号包住 —— Windows 反斜杠在 Tcl 里是
            #      转义字符，"D:\Programs\FPGA" 会被吃成 "D:ProgramsFPGA"，vsim 报
            #      "Cannot open macro file" 却不报错退出；大括号同时解决路径空格。
            #   2) 末尾必须补 "; quit -f" —— 有的 .do 结尾没有 quit，vsim 读到 EOF 会
            #      抛 "U/I: Unexpected EOF on RPC channel" 并以非 0 退出（rc_pwm_receiver
            #      就是这样）。do 内部若已自行退出，这条不会再执行，无害。
            $doPath = ($e.File -replace '\\','/')
            $out = & $vsim -c -do "do {$doPath}; quit -f" 2>&1
        }
        $exit = $LASTEXITCODE
    } catch {
        $out = @("$($_.Exception.Message)")
        $exit = 1
    } finally {
        Pop-Location
    }

    $text = ($out | ForEach-Object { $_.ToString() }) -join "`n"
    Write-Utf8NoBom -Path $logFile -Text $text
    $out | ForEach-Object { Write-Host "     $_" }

    # 判定顺序（为什么不能简单地 -match 'FAIL'）：
    #   rc_pwm_receiver 的 tb 结尾会打印 "TOTAL: 17 groups PASS; 502 assertions PASS; 0 FAIL"
    #   —— 裸匹配 FAIL 会把"零失败"当成失败。所以先把"0 FAIL / FAILURES: 0 / no FAIL"
    #   这类"零失败"表述数出来，用 failHits - zeroFail 判断是否真有失败。
    #   另外把"硬错误"独立出来（vsim 自身报错、宏文件打不开、RPC EOF），它们一定算失败。
    #   顺序上：硬错误 → 真 FAIL → PASS → 仅编译 → 退出码 → 待人工确认。
    $hardError = ($text -match '(?m)\*\*\s*Error') -or ($text -match 'Error \(suppress') -or
                 ($text -match '(?m)^\s*U/I:') -or ($text -match 'Cannot open macro file')
    $failHits  = ([regex]::Matches($text, '(?i)\bFAIL(?:ED|URE|URES)?\b')).Count
    $zeroFail  = ([regex]::Matches($text, '(?i)\b0\s+FAIL(?:ED|URE|URES)?\b|\bFAIL(?:ED|URE|URES)?\s*[:=]?\s*0\b|\bno\s+FAIL')).Count
    $passHit   = ($text -match '(?m)PASS') -or ($text -match '测试通过')

    # 特例：有的 .do 只做编译检查、不加载设计（内容里没有 vsim，例如 m10_radar 的
    #   run_tests.do，末尾是 "quit -code 0"）。这类入口不会有 PASS/FAIL 关键字，
    #   按"退出码 0 = 编译通过"判定，标成 COMPILE-ONLY，不计入需要人工看的 NO-RESULT。
    $compileOnly = $false
    if ($e.Kind -eq 'do') {
        $doBody = Read-Utf8Text $e.File
        if ($doBody -notmatch '(?m)^\s*vsim\b' -and $doBody -notmatch '(?m)^\s*vsim\s') { $compileOnly = $true }
    }

    $status = 'NO-RESULT'
    if ($hardError) {
        $status = 'FAIL'
    } elseif (($failHits - $zeroFail) -gt 0) {
        $status = 'FAIL'
    } elseif ($passHit) {
        $status = 'PASS'
    } elseif ($compileOnly) {
        $status = if ($exit -eq 0) { 'COMPILE-ONLY' } else { 'FAIL' }
    } elseif ($exit -ne 0) {
        $status = 'FAIL'
    }

    switch ($status) {
        'PASS'         { Write-Host "     => PASS" -ForegroundColor Green }
        'COMPILE-ONLY' { Write-Host "     => COMPILE-ONLY（该入口只编译不跑仿真，退出码 $exit）" -ForegroundColor Green }
        'FAIL'         { Write-Host "     => FAIL（退出码 $exit，详见 $logFile）" -ForegroundColor Red }
        default        { Write-Host "     => NO-RESULT（没抓到 PASS/FAIL 关键字，退出码 $exit，详见 $logFile）" -ForegroundColor Yellow }
    }

    $results += [pscustomobject]@{ 包 = $e.Pkg; 入口 = (Split-Path -Leaf $e.File); 结果 = $status; 日志 = (Split-Path -Leaf $logFile) }
}

# ---------- 汇总 ----------
Write-Host ''
Write-Host '================ 汇总 ================' -ForegroundColor Cyan
$results | Format-Table -AutoSize

$pass = @($results | Where-Object { $_.结果 -eq 'PASS' })
$comp = @($results | Where-Object { $_.结果 -eq 'COMPILE-ONLY' })
$fail = @($results | Where-Object { $_.结果 -eq 'FAIL' })
$skip = @($results | Where-Object { $_.结果 -eq 'SKIP' })
$none = @($results | Where-Object { $_.结果 -eq 'NO-RESULT' })

Write-Host ("PASS {0} / COMPILE-ONLY {1} / FAIL {2} / SKIP {3} / NO-RESULT {4}" -f $pass.Count, $comp.Count, $fail.Count, $skip.Count, $none.Count)
if ($fail.Count) { Write-Host ("失败入口：" + (($fail | ForEach-Object { "$($_.包)/$($_.入口)" }) -join ', ')) -ForegroundColor Red }
if ($none.Count) { Write-Host ("需要人工看日志的入口：" + (($none | ForEach-Object { "$($_.包)/$($_.入口)" }) -join ', ')) -ForegroundColor Yellow }
Write-Host ("完整日志：{0}" -f $LogRoot)

if ($fail.Count -gt 0 -or $none.Count -gt 0) { exit 1 } else { exit 0 }
