param(
    [Parameter(Mandatory = $true)][string]$PortName,
    [ValidateRange(1, 30)][int]$Seconds = 5,
    [string]$OutputPath = ''
)
$ErrorActionPreference = 'Stop'
$rootPath = Split-Path -Parent $PSScriptRoot
if (!$OutputPath) {
    $captureDirectory = Join-Path $rootPath 'captures'
    New-Item -ItemType Directory -Path $captureDirectory -Force | Out-Null
    $OutputPath = Join-Path $captureDirectory ("m10_" + (Get-Date -Format 'yyyyMMdd_HHmmss') + '.bin')
}
$capturePath = [System.IO.Path]::GetFullPath($OutputPath)
if (Test-Path -LiteralPath $capturePath) { throw "Capture file already exists: $capturePath" }
$serial = [System.IO.Ports.SerialPort]::new($PortName, 921600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadBufferSize = 262144
$serial.ReadTimeout = 1000
$captureFile = $null
$totalBytes = 0
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $captureFile = [System.IO.File]::Open($capturePath, [System.IO.FileMode]::CreateNew, [System.IO.FileAccess]::Write, [System.IO.FileShare]::Read)
    $buffer = New-Object byte[] 65536
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
        $available = [Math]::Min($serial.BytesToRead, $buffer.Length)
        if ($available -gt 0) {
            $received = $serial.Read($buffer, 0, $available)
            $captureFile.Write($buffer, 0, $received)
            $totalBytes += $received
        } else {
            Start-Sleep -Milliseconds 5
        }
    }
} finally {
    if ($captureFile) { $captureFile.Dispose() }
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
Write-Output "Captured $totalBytes bytes from $PortName (921600 8N1): $capturePath"
if ($totalBytes -eq 0) { throw 'No bytes received. Confirm the raw-mirror bitstream is downloaded and the board USB-UART cable/COM port is correct.' }
& python (Join-Path $PSScriptRoot 'analyze_m10_dump.py') $capturePath
if ($LASTEXITCODE -ne 0) { throw 'Raw capture analysis failed.' }
