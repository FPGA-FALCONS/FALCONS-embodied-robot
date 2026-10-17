param([string]$Verilator=$env:VERILATOR,[string]$VerilatorRoot='')
$ErrorActionPreference='Stop'
if ($VerilatorRoot) { $env:VERILATOR_ROOT=$VerilatorRoot }
if (!$Verilator) {
    $tofTool=Get-Command verilator -ErrorAction SilentlyContinue
    if (!$tofTool) { throw 'Add Verilator to PATH or pass -Verilator; optionally pass -VerilatorRoot.' }
    $Verilator=$tofTool.Source
}
$tofModule=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$tofOutput=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $tofOutput | Out-Null
$tofSources=@((Join-Path $tofModule 'rtl/tof_i2c_reg_master.v'),(Join-Path $tofModule 'rtl/tof_vl53l0x.v'),
    (Join-Path $PSScriptRoot 'tof_display_formatter.v'),(Join-Path $PSScriptRoot 'tof_hc595_display.v'),(Join-Path $PSScriptRoot 'tof_debug_top.v'))
$tofConfig=Join-Path $PSScriptRoot 'verilator_reset.vlt'
if (!(Test-Path -LiteralPath $tofConfig)) { throw "Missing documented narrow reset waiver: $tofConfig" }
$tofPreference=$ErrorActionPreference
try {
    $ErrorActionPreference='Continue'
    & $Verilator --lint-only -Wall --top-module tof_debug_top $tofConfig @tofSources 2>&1 | ForEach-Object { $_.ToString() } | Tee-Object -FilePath (Join-Path $tofOutput 'lint.log')
    $tofExit=$LASTEXITCODE
} finally { $ErrorActionPreference=$tofPreference }
if ($tofExit -ne 0) { throw "Verilator failed ($tofExit); see $tofOutput/lint.log" }
Write-Host 'PASS: production tof_debug_top Verilator -Wall lint.'
