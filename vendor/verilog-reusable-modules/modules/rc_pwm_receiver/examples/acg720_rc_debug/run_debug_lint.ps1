param([string]$Verilator='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'tool_helpers.ps1')
if (!$Verilator) {
    $debugTool=Get-Command verilator -ErrorAction SilentlyContinue
    if ($debugTool) { $Verilator=$debugTool.Source }
    else { throw 'Pass -Verilator and configure VERILATOR_ROOT if required by your installation.' }
}
$debugModule=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$debugOutput=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $debugOutput | Out-Null
$debugSources=@('rc_pwm_capture.v','rc_axis_normalize.v','rc_receiver.v') | ForEach-Object { Join-Path $debugModule "rtl/$_" }
$debugSources+=@(Join-Path $PSScriptRoot 'rc_hc595_display.v')
$debugSources+=@(Join-Path $PSScriptRoot 'rc_debug_top.v')
Invoke-DebugNative -Executable $Verilator -Arguments (@('--lint-only','-Wall','--top-module','rc_debug_top')+$debugSources) -LogPath (Join-Path $debugOutput 'lint.log')
