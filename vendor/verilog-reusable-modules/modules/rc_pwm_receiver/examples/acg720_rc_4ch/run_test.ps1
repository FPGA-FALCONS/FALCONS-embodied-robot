param([string]$Iverilog='', [string]$Vvp='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../acg720_rc_debug/tool_helpers.ps1')
if (!$Iverilog) {
    $tool=Get-Command iverilog -ErrorAction SilentlyContinue
    if ($tool) {$Iverilog=$tool.Source}
    elseif(Test-Path 'C:/iverilog/bin/iverilog.exe') {$Iverilog='C:/iverilog/bin/iverilog.exe'}
    else {throw 'Pass -Iverilog.'}
}
if (!$Vvp) {
    $tool=Get-Command vvp -ErrorAction SilentlyContinue
    if ($tool) {$Vvp=$tool.Source}
    elseif(Test-Path 'C:/iverilog/bin/vvp.exe') {$Vvp='C:/iverilog/bin/vvp.exe'}
    else {throw 'Pass -Vvp.'}
}
$output=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $output | Out-Null
$sources=@((Join-Path $PSScriptRoot '../../rtl/rc_pwm_capture.v'),
    (Join-Path $PSScriptRoot '../acg720_rc_debug/rc_hc595_display.v'),
    (Join-Path $PSScriptRoot 'rc_4ch_top.v'))
Invoke-DebugNative -Executable $Iverilog -Arguments (@('-g2001','-Wall','-s','rc_4ch_top','-o',(Join-Path $output 'production.vvp'))+$sources) -LogPath (Join-Path $output 'compile_2001.log')
$exe=Join-Path $output 'tb_rc_4ch.vvp'
Invoke-DebugNative -Executable $Iverilog -Arguments (@('-g2012','-Wall','-s','tb_rc_4ch','-o',$exe)+$sources+@(Join-Path $PSScriptRoot 'tb_rc_4ch.sv')) -LogPath (Join-Path $output 'compile_tb.log')
Invoke-DebugNative -Executable $Vvp -Arguments @($exe) -LogPath (Join-Path $output 'tb_rc_4ch.log')
