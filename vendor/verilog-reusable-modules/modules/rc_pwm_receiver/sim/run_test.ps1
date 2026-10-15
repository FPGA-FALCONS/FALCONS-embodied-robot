param([string]$Iverilog = 'iverilog', [string]$Vvp = 'vvp')
$ErrorActionPreference = 'Stop'
$rcModuleRoot = Split-Path -Parent $PSScriptRoot
$rcBuild = Join-Path $rcModuleRoot 'build'
New-Item -ItemType Directory -Path $rcBuild -Force | Out-Null

function Invoke-RcTool {
    param([string]$Executable, [string[]]$Arguments, [string]$LogPath)
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
    (Join-Path $rcModuleRoot 'rtl/rc_pwm_capture.v'),
    (Join-Path $rcModuleRoot 'rtl/rc_receiver.v')
)
$rcTb = Join-Path $PSScriptRoot 'rc_receiver_tb.v'
$rcExecutable = Join-Path $rcBuild 'rc_receiver_tb.vvp'
Push-Location $rcBuild
try {
    Invoke-RcTool $Iverilog (@('-g2001', '-Wall', '-s', 'rc_receiver', '-t', 'null') + $rcSources) (Join-Path $rcBuild 'rtl_compile.log')
    Invoke-RcTool $Iverilog (@('-g2012', '-Wall', '-s', 'rc_receiver_tb', '-o', $rcExecutable) + $rcSources + @($rcTb)) (Join-Path $rcBuild 'compile.log')
    Invoke-RcTool $Vvp @('-i', $rcExecutable) (Join-Path $rcBuild 'simulation.log')
    Write-Host 'RC receiver simulation PASS.'
} finally {
    Pop-Location
}
