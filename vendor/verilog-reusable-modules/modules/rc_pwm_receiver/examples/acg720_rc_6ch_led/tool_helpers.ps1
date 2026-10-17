function Invoke-DebugNative {
    param([string]$Executable,[string[]]$Arguments,[string]$LogPath)
    $debugPreference=$ErrorActionPreference
    try {
        $ErrorActionPreference='Continue'
        & $Executable @Arguments 2>&1 | ForEach-Object { $_.ToString() } | Tee-Object -FilePath $LogPath
        $debugExit=$LASTEXITCODE
    } finally { $ErrorActionPreference=$debugPreference }
    if ($debugExit -ne 0) { throw "Tool failed ($debugExit): $Executable; see $LogPath" }
}
