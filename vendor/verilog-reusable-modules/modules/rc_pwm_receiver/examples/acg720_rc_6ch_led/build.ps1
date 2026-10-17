param([string]$GowinShell='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'tool_helpers.ps1')
if (!$GowinShell) {
    $rcTool=Get-Command gw_sh -ErrorAction SilentlyContinue
    if ($rcTool) { $GowinShell=$rcTool.Source }
    elseif ($env:GOWIN_HOME) { $GowinShell=Join-Path $env:GOWIN_HOME 'IDE/bin/gw_sh.exe' }
    else { throw 'Pass -GowinShell path/to/IDE/bin/gw_sh.exe, or set GOWIN_HOME.' }
}
$rcOutput=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $rcOutput | Out-Null
$rcBuildStarted=(Get-Date).ToUniversalTime()
$rcBuildText=Invoke-DebugNative -Executable $GowinShell -Arguments @((Join-Path $PSScriptRoot 'build.tcl')) -LogPath (Join-Path $rcOutput 'gowin_build.log')
$rcBuildText | ForEach-Object { Write-Host $_ }
if (($rcBuildText -join "`n") -notmatch 'RC_6CH_LED_BUILD_FINISHED' -or ($rcBuildText -join "`n") -match '(?m)^ERROR\s') { throw 'Gowin six-channel LED build did not finish successfully.' }
if ((Get-Item -LiteralPath (Join-Path $rcOutput 'impl/pnr/project_tr_content.html')).LastWriteTimeUtc -lt $rcBuildStarted) { throw 'Refusing stale timing report.' }
& (Join-Path $PSScriptRoot 'check_timing.ps1')
