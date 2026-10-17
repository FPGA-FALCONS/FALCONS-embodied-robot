param([string]$Iverilog=$env:IVERILOG,[string]$Vvp=$env:VVP)
$ErrorActionPreference='Stop'
function Resolve-TofTool([string]$Requested,[string]$Name) {
    if ($Requested) { return (Get-Command $Requested -ErrorAction Stop).Source }
    $tofTool=Get-Command $Name -ErrorAction SilentlyContinue
    if ($tofTool) { return $tofTool.Source }
    throw "Cannot find $Name. Add Icarus to PATH or pass -Iverilog and -Vvp executable paths."
}
function Invoke-TofNative([string]$Executable,[string[]]$Arguments,[string]$LogPath) {
    $tofPreference=$ErrorActionPreference
    try {
        # Windows PowerShell 5.1 wraps native stderr in ErrorRecord; warnings
        # must be recorded without masking the actual native exit code.
        $ErrorActionPreference='Continue'
        & $Executable @Arguments 2>&1 | ForEach-Object { $_.ToString() } | Tee-Object -FilePath $LogPath
        $tofExit=$LASTEXITCODE
    } finally { $ErrorActionPreference=$tofPreference }
    if ($tofExit -ne 0) { throw "Tool failed ($tofExit): $Executable; see $LogPath" }
}
$Iverilog=Resolve-TofTool $Iverilog 'iverilog'
$Vvp=Resolve-TofTool $Vvp 'vvp'
$tofModule=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$tofOutput=Join-Path $PSScriptRoot 'build/sim'
New-Item -ItemType Directory -Force -Path $tofOutput | Out-Null
$tofSources=@((Join-Path $tofModule 'rtl/tof_i2c_reg_master.v'),(Join-Path $tofModule 'rtl/tof_vl53l0x.v'),
    (Join-Path $PSScriptRoot 'tof_display_formatter.v'),(Join-Path $PSScriptRoot 'tof_hc595_display.v'),(Join-Path $PSScriptRoot 'tof_debug_top.v'))
Invoke-TofNative $Iverilog (@('-g2001','-s','tof_debug_top','-o',(Join-Path $tofOutput 'production.vvp'))+$tofSources) (Join-Path $tofOutput 'production_compile.log')
foreach ($tofTop in @('tb_tof_formatter','tb_tof_debug')) {
    $tofBenchSources=$tofSources+@((Join-Path $tofModule 'tb/vl53l0x_sensor_model.sv'),(Join-Path $PSScriptRoot "$tofTop.sv"))
    $tofVvpPath=Join-Path $tofOutput "$tofTop.vvp"
    Invoke-TofNative $Iverilog (@('-g2012','-s',$tofTop,'-o',$tofVvpPath)+$tofBenchSources) (Join-Path $tofOutput "${tofTop}_compile.log")
    Invoke-TofNative $Vvp @($tofVvpPath) (Join-Path $tofOutput "$tofTop.log")
}
Write-Host 'PASS: production Verilog-2001 compilation, exhaustive formatter and physical I2C/HC595 board simulations.'
