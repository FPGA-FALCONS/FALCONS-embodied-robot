param([string]$Iverilog='', [string]$Vvp='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'tool_helpers.ps1')
if (!$Iverilog) {
    $debugTool=Get-Command iverilog -ErrorAction SilentlyContinue
    if ($debugTool) { $Iverilog=$debugTool.Source }
    elseif (Test-Path 'C:/iverilog/bin/iverilog.exe') { $Iverilog='C:/iverilog/bin/iverilog.exe' }
    else { throw 'Pass -Iverilog.' }
}
if (!$Vvp) {
    $debugTool=Get-Command vvp -ErrorAction SilentlyContinue
    if ($debugTool) { $Vvp=$debugTool.Source }
    elseif (Test-Path 'C:/iverilog/bin/vvp.exe') { $Vvp='C:/iverilog/bin/vvp.exe' }
    else { throw 'Pass -Vvp.' }
}
$debugModule=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$debugOutput=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $debugOutput | Out-Null
$debugSources=@('rc_pwm_capture.v','rc_axis_normalize.v','rc_receiver.v') | ForEach-Object { Join-Path $debugModule "rtl/$_" }
$debugSources+=@(Join-Path $PSScriptRoot 'rc_hc595_display.v')
$debugSources+=@(Join-Path $PSScriptRoot 'rc_debug_top.v')
Invoke-DebugNative -Executable $Iverilog -Arguments (@('-g2001','-Wall','-s','rc_debug_top','-o',(Join-Path $debugOutput 'rc_debug_2001.vvp'))+$debugSources) -LogPath (Join-Path $debugOutput 'compile_2001.log')
$debugExe=Join-Path $debugOutput 'tb_rc_debug.vvp'
Invoke-DebugNative -Executable $Iverilog -Arguments (@('-g2012','-Wall','-s','tb_rc_debug','-o',$debugExe)+$debugSources+@(Join-Path $PSScriptRoot 'tb_rc_debug.v')) -LogPath (Join-Path $debugOutput 'compile_tb.log')
Invoke-DebugNative -Executable $Vvp -Arguments @($debugExe) -LogPath (Join-Path $debugOutput 'tb_rc_debug.log')
