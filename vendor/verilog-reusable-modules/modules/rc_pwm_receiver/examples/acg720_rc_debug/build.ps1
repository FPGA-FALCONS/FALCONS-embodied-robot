param([string]$GowinShell='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'tool_helpers.ps1')
if (!$GowinShell) {
    $debugTool=Get-Command gw_sh -ErrorAction SilentlyContinue
    if ($debugTool) { $GowinShell=$debugTool.Source }
    elseif ($env:GOWIN_HOME) { $GowinShell=Join-Path $env:GOWIN_HOME 'IDE/bin/gw_sh.exe' }
    else { throw 'Pass -GowinShell path/to/IDE/bin/gw_sh.exe, or set GOWIN_HOME.' }
}
$debugOutput=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $debugOutput | Out-Null
Invoke-DebugNative -Executable $GowinShell -Arguments @((Join-Path $PSScriptRoot 'build.tcl')) -LogPath (Join-Path $debugOutput 'gowin_build.log')
& (Join-Path $PSScriptRoot 'check_timing.ps1')
