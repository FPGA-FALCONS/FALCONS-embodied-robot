param([string]$Verilator='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../acg720_rc_debug/tool_helpers.ps1')
if (!$Verilator) {
    $tool=Get-Command verilator -ErrorAction SilentlyContinue
    if($tool) {$Verilator=$tool.Source}
    else {throw 'Pass -Verilator and configure VERILATOR_ROOT for your installation.'}
}
$output=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $output | Out-Null
$sources=@((Join-Path $PSScriptRoot '../../rtl/rc_pwm_capture.v'),
    (Join-Path $PSScriptRoot '../acg720_rc_debug/rc_hc595_display.v'),
    (Join-Path $PSScriptRoot 'rc_4ch_top.v'))
Invoke-DebugNative -Executable $Verilator -Arguments (@('--lint-only','-Wall','--top-module','rc_4ch_top')+$sources) -LogPath (Join-Path $output 'lint.log')
