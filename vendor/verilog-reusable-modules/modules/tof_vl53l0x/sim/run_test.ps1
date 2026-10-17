param(
    [string]$Iverilog = $env:IVERILOG,
    [string]$Vvp = $env:VVP
)
$ErrorActionPreference = 'Stop'
$moduleRoot = Split-Path $PSScriptRoot -Parent
function Resolve-Tool([string]$requested, [string]$name) {
    if ($requested) { return (Get-Command $requested -ErrorAction Stop).Source }
    $found = Get-Command $name -ErrorAction SilentlyContinue
    if ($found) { return $found.Source }
    $fallback = Join-Path 'C:/iverilog/bin' ($name + '.exe')
    if (Test-Path -LiteralPath $fallback) { return $fallback }
    throw "Cannot find $name. Add Icarus Verilog to PATH or pass its executable path."
}
$Iverilog = Resolve-Tool $Iverilog 'iverilog'
$Vvp = Resolve-Tool $Vvp 'vvp'
$artifactRoot = Join-Path $PSScriptRoot 'artifacts'
New-Item -ItemType Directory -Force -Path $artifactRoot | Out-Null
function Invoke-Checked([string]$exe, [string[]]$arguments, [string]$log) {
    $output = & $exe @arguments 2>&1
    $result = $LASTEXITCODE
    if ($log) { $output | Out-File -LiteralPath $log -Encoding utf8 }
    $output | ForEach-Object { Write-Host $_ }
    if ($result -ne 0) { throw "$exe failed with exit code $result" }
}
Push-Location $moduleRoot
try {
    $rtl = @('rtl/tof_i2c_reg_master.v', 'rtl/tof_vl53l0x.v')
    Invoke-Checked $Iverilog (@('-g2001','-s','tof_vl53l0x','-o',"$artifactRoot/rtl.vvp") + $rtl) ''
    foreach ($clock in @(4000000,50000000)) {
        $name = "master_$clock"
        Invoke-Checked $Iverilog @('-g2012','-s','tb_tof_i2c_reg_master',"-Ptb_tof_i2c_reg_master.CLK_HZ=$clock",'-o',"$artifactRoot/$name.vvp",'rtl/tof_i2c_reg_master.v','tb/tb_tof_i2c_reg_master.v') ''
        Invoke-Checked $Vvp @("$artifactRoot/$name.vvp") "$artifactRoot/$name.log"
    }
    foreach ($top in @('tb_tof_vl53l0x','tb_tof_stale')) {
        Invoke-Checked $Iverilog (@('-g2012','-s',$top,'-o',"$artifactRoot/$top.vvp") + $rtl + @('tb/vl53l0x_sensor_model.sv',"tb/$top.sv")) ''
        Invoke-Checked $Vvp @("$artifactRoot/$top.vvp") "$artifactRoot/$top.log"
    }
    Write-Host 'PASS: Verilog-2001 RTL compilation and all four simulations.'
} finally { Pop-Location }
