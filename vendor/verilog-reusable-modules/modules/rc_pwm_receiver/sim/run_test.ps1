param([string]$Iverilog = 'iverilog', [string]$Vvp = 'vvp')
$ErrorActionPreference = 'Stop'
$rcModuleRoot = Split-Path -Parent $PSScriptRoot
$rcBuild = Join-Path $rcModuleRoot 'build'
New-Item -ItemType Directory -Path $rcBuild -Force | Out-Null

function Invoke-RcTool {
    param([string]$Executable, [string[]]$Arguments, [string]$LogPath)
    # Resolve first so a missing executable cannot reuse a previous exit code.
    Get-Command $Executable -ErrorAction Stop | Out-Null
    $rcSavedPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & $Executable @Arguments 2>&1 |
            ForEach-Object { $_.ToString() } | Tee-Object -FilePath $LogPath
        $rcExitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $rcSavedPreference
    }
    if ($rcExitCode -ne 0) {
        throw "Tool failed (exit $rcExitCode): $Executable; log: $LogPath"
    }
}

$rcSources = @(
    (Join-Path $rcModuleRoot 'rtl/rc_axis_normalize.v'),
    (Join-Path $rcModuleRoot 'rtl/rc_pwm_capture.v'),
    (Join-Path $rcModuleRoot 'rtl/rc_receiver.v')
)
$rcBenches = @(
    @{ Name = 'rc_receiver_tb'; Marker = 'TOTAL: 17 groups PASS; 502 assertions PASS; 0 FAIL' },
    @{ Name = 'rc_axis_normalize_tb'; Marker = 'PASS rc_axis_normalize:' },
    @{ Name = 'rc_receiver_pipeline_tb'; Marker = 'PASS rc_receiver_pipeline:' }
)
Push-Location $rcBuild
try {
    Invoke-RcTool $Iverilog (@('-g2001', '-Wall', '-s', 'rc_receiver', '-t', 'null') + $rcSources) (Join-Path $rcBuild 'rtl_compile.log')
    foreach ($rcBench in $rcBenches) {
        $rcName = $rcBench.Name
        $rcTb = Join-Path $PSScriptRoot ($rcName + '.v')
        $rcExecutable = Join-Path $rcBuild ($rcName + '.vvp')
        $rcCompileLog = Join-Path $rcBuild ($rcName + '_compile.log')
        $rcRunLog = Join-Path $rcBuild ($rcName + '_simulation.log')
        Write-Host ('Running ' + $rcName + ' ...')
        Invoke-RcTool $Iverilog (@('-g2012', '-Wall', '-s', $rcName, '-o', $rcExecutable) + $rcSources + @($rcTb)) $rcCompileLog
        # Separate VVP processes: one bench's $finish cannot terminate the rest.
        Invoke-RcTool $Vvp @('-i', $rcExecutable) $rcRunLog
        $rcOutput = Get-Content -Raw -LiteralPath $rcRunLog
        if ($rcOutput -notmatch [regex]::Escape($rcBench.Marker)) {
            throw ('Missing completion marker for ' + $rcName + '; log: ' + $rcRunLog)
        }
    }
    Write-Host 'RC module simulation PASS: all 3 independent testbenches completed.'
} finally {
    Pop-Location
}
