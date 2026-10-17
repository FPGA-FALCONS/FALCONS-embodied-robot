param(
    [string]$Verilator = 'verilator',
    [string]$VerilatorRoot = ''
)
$ErrorActionPreference = 'Stop'
$rcModuleRoot = Split-Path -Parent $PSScriptRoot
$rcBuild = Join-Path $rcModuleRoot 'build'
New-Item -ItemType Directory -Path $rcBuild -Force | Out-Null
$rcLog = Join-Path $rcBuild 'lint.log'
$rcSources = @(
    (Join-Path $rcModuleRoot 'rtl/rc_axis_normalize.v'),
    (Join-Path $rcModuleRoot 'rtl/rc_pwm_capture.v'),
    (Join-Path $rcModuleRoot 'rtl/rc_receiver.v')
)
Get-Command $Verilator -ErrorAction Stop | Out-Null
$rcSavedRoot = $env:VERILATOR_ROOT
$rcSavedPreference = $ErrorActionPreference
try {
    if ($VerilatorRoot) {
        if (-not (Test-Path -LiteralPath $VerilatorRoot -PathType Container)) {
            throw ('VerilatorRoot is not a directory: ' + $VerilatorRoot)
        }
        $env:VERILATOR_ROOT = $VerilatorRoot
    }
    # PS 5.1 wraps native stderr as error records; preserve its text and check
    # the executable's real status instead of treating any stderr as failure.
    $ErrorActionPreference = 'Continue'
    & $Verilator --lint-only --language 1364-2001 -Wall --top-module rc_receiver @rcSources 2>&1 |
        ForEach-Object { $_.ToString() } | Tee-Object -FilePath $rcLog
    $rcLintExit = $LASTEXITCODE
    $ErrorActionPreference = $rcSavedPreference
    if ($rcLintExit -ne 0) {
        throw ('Verilator failed (exit ' + $rcLintExit + '); log: ' + $rcLog)
    }
    Write-Host 'RC module Verilator lint PASS (all three RTL dependencies).'
} finally {
    $ErrorActionPreference = $rcSavedPreference
    if ($null -eq $rcSavedRoot) {
        Remove-Item Env:VERILATOR_ROOT -ErrorAction SilentlyContinue
    } else {
        $env:VERILATOR_ROOT = $rcSavedRoot
    }
}
