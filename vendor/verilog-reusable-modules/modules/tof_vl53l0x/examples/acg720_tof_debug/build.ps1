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
$tofBuildStarted = (Get-Date).ToUniversalTime()
$tofBuildText = Invoke-DebugNative -Executable $GowinShell -Arguments @((Join-Path $PSScriptRoot 'build.tcl')) -LogPath (Join-Path $debugOutput 'gowin_build.log')
$tofBuildText | ForEach-Object { Write-Host $_ }
if (($tofBuildText -join "`n") -notmatch 'TOF_DEBUG_BUILD_FINISHED' -or
    ($tofBuildText -join "`n") -match '(?m)^ERROR\s') {
    throw 'Gowin did not finish the requested build; inspect build/gowin_build.log.'
}
$tofReport = Get-Item -LiteralPath (Join-Path $debugOutput 'impl/pnr/project_tr_content.html')
if ($tofReport.LastWriteTimeUtc -lt $tofBuildStarted) {
    throw 'Timing report predates this build. Refusing to accept stale artifacts.'
}
& (Join-Path $PSScriptRoot 'check_timing.ps1')
