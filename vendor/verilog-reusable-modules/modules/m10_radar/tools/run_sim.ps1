param([string]$ModelSimBin = 'D:\apps\modelsim\win64')
$ErrorActionPreference = 'Stop'
$rootPath = Split-Path -Parent $PSScriptRoot
$simPath = Join-Path $rootPath 'sim'
Push-Location -LiteralPath $simPath
try {
    & (Join-Path $ModelSimBin 'vsim.exe') -c -do run_tests.do -l compile.log
    if ($LASTEXITCODE -ne 0) { throw "ModelSim failed, exit code $LASTEXITCODE" }
    $expected = @('m10_radar_tb','uart_rx_recovery_tb','m10_gps_packet_parser_tb','zoom_control_tb','polar_to_xy_tb','point_bitmap_tb','point_cloud_video_tb','m10_point_cloud_core_tb')
    Set-Content -LiteralPath regression.log -Value ''
    foreach ($test in $expected) {
        $testLog = "$test.log"
        & (Join-Path $ModelSimBin 'vsim.exe') -c -t 1ps -onfinish exit "work.$test" -do 'run -all; quit -code 0 -force' -l $testLog
        if ($LASTEXITCODE -ne 0) { throw "ModelSim failed $test, exit code $LASTEXITCODE" }
        $log = Get-Content -LiteralPath $testLog -Raw
        Add-Content -LiteralPath regression.log -Value $log
        if ($log -match '(?m)^#\s*(FAIL|FATAL|\*\* (Error|Fatal))') { throw "Simulation reported a failure: $test" }
        if ($log -notmatch "PASS $test") { throw "Missing PASS result: $test" }
    }
    Write-Output 'All radar regression tests passed.'
} finally {
    Pop-Location
}
