# 将 nav_polar 的独立综合网表接回实际 UART、建图及显示 RTL 链路。
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Netlist,
    [string]$ModelSim = 'D:/apps/modelsim/win64',
    [string]$GowinSimLib = 'D:/Apps/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v'
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Netlist = (Resolve-Path -LiteralPath $Netlist).Path
$OutputDir = Join-Path $ProjectRoot 'logs/radar_polar_gate'
$Library = Join-Path $OutputDir 'work_gate'
$Vendor = Join-Path $ProjectRoot 'vendor/verilog-reusable-modules/modules'
New-Item -ItemType Directory -Force $OutputDir | Out-Null

function Invoke-SimTool {
    param([string]$Name, [string[]]$Arguments, [string]$Log)
    $exe = Join-Path $ModelSim ($Name + '.exe')
    $ErrorActionPreference = 'Continue'
    $output = & $exe @Arguments 2>&1
    $code = $LASTEXITCODE
    $output | Out-File -Encoding utf8 $Log
    if ($code -ne 0) { throw "$Name failed; see $Log" }
}

Push-Location $PSScriptRoot
try {
    if (Test-Path -LiteralPath $Library) {
        Invoke-SimTool 'vdel' @('-lib', $Library, '-all') (Join-Path $OutputDir 'vdel.log')
    }
    Invoke-SimTool 'vlib' @($Library) (Join-Path $OutputDir 'vlib.log')
    $sources = @($GowinSimLib, $Netlist)
    $sources += @(Get-ChildItem "$Vendor/auto_navigation/rtl/*.v" |
                  Where-Object Name -ne 'nav_polar.v' | ForEach-Object FullName)
    $sources += @(Get-ChildItem "$Vendor/radar_navigation/rtl/*.v" | ForEach-Object FullName)
    $sources += @("$Vendor/m10_radar/rtl/m10_radar.v", "$Vendor/m10_radar/rtl/m10_packet_parser.v",
                  "$Vendor/uart/rtl/uart_byte_rx.v", "$ProjectRoot/rtl/ui/nav_map_view.v",
                  "$PSScriptRoot/radar_coordinates_tb.v")
    Invoke-SimTool 'vlog' (@('-work', $Library) + $sources) (Join-Path $OutputDir 'compile.log')
    $transcript = Join-Path $OutputDir 'uart_gate.log'
    Invoke-SimTool 'vsim' @('-c', '-lib', '../logs/radar_polar_gate/work_gate', 'radar_coordinates_tb', 'GSR',
                            '-do', 'force /GSR/GSRI 1; run -all; quit -f', '-l', $transcript) `
                          (Join-Path $OutputDir 'console.log')
    $lines = Get-Content -LiteralPath $transcript
    if (($lines -match '^# FAIL:').Count -or
        -not ($lines -match '^# PASS: radar_coordinates_tb$')) {
        throw "UART gate regression failed; see $transcript"
    }
    $lines | Where-Object { $_ -match '^# (METRICS:|CHECKS:|PASS:)' } | Write-Host
} finally {
    Pop-Location
}
