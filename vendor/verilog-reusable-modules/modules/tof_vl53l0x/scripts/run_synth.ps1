param([string]$GowinShell = $env:GOWIN_SH)
$ErrorActionPreference = 'Stop'
if (-not $GowinShell) {
    $found = Get-Command gw_sh -ErrorAction SilentlyContinue
    if ($found) { $GowinShell = $found.Source }
}
if (-not $GowinShell -or -not (Test-Path -LiteralPath $GowinShell)) {
    throw 'Pass -GowinShell /path/to/Gowin/IDE/bin/gw_sh.exe, or set GOWIN_SH.'
}
$tofRoot = Split-Path $PSScriptRoot -Parent
$artifacts = Join-Path $tofRoot 'sim/artifacts'
New-Item -ItemType Directory -Force -Path $artifacts | Out-Null
# ProcessStartInfo captures native stderr without PowerShell 5.1 treating normal
# vendor diagnostic messages as terminating NativeCommandError exceptions.
$startInfo = New-Object System.Diagnostics.ProcessStartInfo
$startInfo.FileName = $GowinShell
$startInfo.Arguments = '"' + (Join-Path $PSScriptRoot 'synth.tcl') + '"'
$startInfo.WorkingDirectory = $tofRoot
$startInfo.UseShellExecute = $false
$startInfo.CreateNoWindow = $true
$startInfo.RedirectStandardOutput = $true
$startInfo.RedirectStandardError = $true
$process = New-Object System.Diagnostics.Process
$process.StartInfo = $startInfo
[void]$process.Start()
$stdout = $process.StandardOutput.ReadToEndAsync()
$stderr = $process.StandardError.ReadToEndAsync()
$process.WaitForExit()
$outputText = $stdout.Result + $stderr.Result
$outputText | Out-File -LiteralPath (Join-Path $artifacts 'gowin_synthesis.log') -Encoding utf8
Write-Host $outputText
if ($process.ExitCode -ne 0 -or $outputText -notmatch 'TOF_SYNTH_SCRIPT_FINISHED' -or $outputText -match '(?m)^ERROR\s') {
    throw "Gowin synthesis failed; exit code $($process.ExitCode). See sim/artifacts/gowin_synthesis.log."
}
$generatedProject = Get-Content -LiteralPath (Join-Path $tofRoot 'build/synth/impl/gwsynthesis/project.prj') -Raw
if ($generatedProject -notmatch '<Option type="global_freq" value="50\.000"\s*/>') {
    throw 'GowinSynthesis did not apply the intended 50 MHz global_freq constraint.'
}
Write-Host 'PASS: Gowin synthesis only. This is not place-and-route or hardware verification.'
