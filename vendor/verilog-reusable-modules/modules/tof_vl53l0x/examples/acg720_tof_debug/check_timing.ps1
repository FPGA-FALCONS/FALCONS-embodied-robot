param(
    [string]$ReportPath=(Join-Path $PSScriptRoot 'build/impl/pnr/project_tr_content.html'),
    [string]$BitstreamPath=(Join-Path $PSScriptRoot 'build/impl/pnr/project.fs'),
    [string]$ManifestPath=(Join-Path $PSScriptRoot 'build/timing_manifest.json')
)
$ErrorActionPreference='Stop'
if (!(Test-Path -LiteralPath $ReportPath) -or !(Test-Path -LiteralPath $BitstreamPath)) {
    throw 'Timing report or bitstream missing; complete Gowin Place & Route first.'
}
$debugHtml=Get-Content -LiteralPath $ReportPath -Raw
function Convert-DebugNumber([string]$Value) {
    return [double]::Parse($Value,[cultureinfo]::InvariantCulture)
}
function Get-DebugEndpointCount([string]$Label) {
    $debugMatches=[regex]::Matches($debugHtml,[regex]::Escape($Label)+'</td>\s*<td>(\d+)</td>')
    if ($debugMatches.Count -ne 1) { throw "Expected one timing endpoint count: $Label" }
    return [int]$debugMatches[0].Groups[1].Value
}
function Get-DebugPathTable([string]$AnalysisType) {
    $debugSection=[regex]::Match($debugHtml,
        '(?s)<a name="'+[regex]::Escape($AnalysisType)+'_Slack_Table">.*?</table>')
    if (!$debugSection.Success) { throw "Missing $AnalysisType timing path table; cannot treat missing data as pass." }
    if ($debugSection.Value -notmatch 'Path Slack') { throw "No slack column in $AnalysisType path table." }
    $debugRows=[regex]::Matches($debugSection.Value,
        '<tr>\s*<td>(\d+)</td>\s*<td>([-+]?[0-9]+(?:\.[0-9]+)?)</td>')
    if ($debugRows.Count -lt 1) { throw "No numeric $AnalysisType paths; timing result is inconclusive." }
    $debugMinimum=[double]::PositiveInfinity
    $debugNegative=0
    $debugPrevious=[double]::NegativeInfinity
    $debugPathIndex=0
    foreach ($debugRow in $debugRows) {
        $debugPathIndex++
        if ([int]$debugRow.Groups[1].Value -ne $debugPathIndex) {
            throw "Unexpected numbering in $AnalysisType timing table."
        }
        $debugSlack=Convert-DebugNumber $debugRow.Groups[2].Value
        if ($debugSlack -lt $debugPrevious) { throw "$AnalysisType worst-path table is not in ascending slack order." }
        $debugPrevious=$debugSlack
        if ($debugSlack -lt $debugMinimum) { $debugMinimum=$debugSlack }
        if ($debugSlack -lt 0) { $debugNegative++ }
    }
    return [ordered]@{
        reported_paths=$debugRows.Count
        reported_negative_paths=$debugNegative
        worst_reported_slack_ns=$debugMinimum
        global_violated_endpoint_count=$null
        scope='Gowin worst-path slack table; global recovery/removal endpoint counts are not published in this report.'
    }
}
$debugSetup=Get-DebugEndpointCount 'Numbers of Setup Violated Endpoints'
$debugHold=Get-DebugEndpointCount 'Numbers of Hold Violated Endpoints'
$debugRecovery=Get-DebugPathTable 'Recovery'
$debugRemoval=Get-DebugPathTable 'Removal'
$debugFmaxSection=[regex]::Match($debugHtml,'(?s)<a name="Max_Frequency_Report">.*?</table>')
if (!$debugFmaxSection.Success) { throw 'Missing Fmax summary.' }
$debugFrequencies=[regex]::Matches($debugFmaxSection.Value,'([0-9]+(?:\.[0-9]+)?)\(MHz\)')
if ($debugFrequencies.Count -ne 2 -or $debugFmaxSection.Value -notmatch '<td>clk</td>') {
    throw 'Expected exactly one clock named clk in Fmax summary.'
}
$debugConstraint=Convert-DebugNumber $debugFrequencies[0].Groups[1].Value
$debugFmax=Convert-DebugNumber $debugFrequencies[1].Groups[1].Value
if ($debugConstraint -ne 50 -or $debugFmax -lt 50 -or $debugSetup -ne 0 -or $debugHold -ne 0 -or
    $debugRecovery.reported_negative_paths -ne 0 -or $debugRemoval.reported_negative_paths -ne 0) {
    throw "Timing FAILED: constraint=${debugConstraint}MHz Fmax=${debugFmax}MHz setup=$debugSetup hold=$debugHold recovery_negative=$($debugRecovery.reported_negative_paths) removal_negative=$($debugRemoval.reported_negative_paths)."
}
$debugModuleRoot=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$debugSources=@(
    'rtl/tof_i2c_reg_master.v','rtl/tof_vl53l0x.v',
    'examples/acg720_tof_debug/tof_display_formatter.v',
    'examples/acg720_tof_debug/tof_hc595_display.v',
    'examples/acg720_tof_debug/tof_debug_top.v',
    'examples/acg720_tof_debug/tof_debug.cst',
    'examples/acg720_tof_debug/tof_debug.sdc'
)
$debugHashes=[ordered]@{}
$debugBitstream=Get-Item -LiteralPath $BitstreamPath
foreach($debugRelative in $debugSources) {
    $debugSource=Get-Item -LiteralPath (Join-Path $debugModuleRoot $debugRelative)
    if($debugSource.LastWriteTimeUtc -gt $debugBitstream.LastWriteTimeUtc) {
        throw "Source newer than bitstream: $debugRelative. Rebuild before using this bitstream."
    }
    $debugHashes[$debugRelative]=(Get-FileHash -LiteralPath $debugSource.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
}
$debugHashes['bitstream']=(Get-FileHash -LiteralPath $BitstreamPath -Algorithm SHA256).Hash.ToLowerInvariant()
$debugHashes['timing_report']=(Get-FileHash -LiteralPath $ReportPath -Algorithm SHA256).Hash.ToLowerInvariant()
$debugManifest=[ordered]@{
    status='PASS'
    checked_at_utc=(Get-Date).ToUniversalTime().ToString('o')
    top='tof_debug_top'
    clock_constraint_mhz=$debugConstraint
    actual_fmax_mhz=$debugFmax
    setup_violated_endpoints=$debugSetup
    hold_violated_endpoints=$debugHold
    recovery=$debugRecovery
    removal=$debugRemoval
    bitstream_path=$debugBitstream.FullName
    timing_report_path=(Get-Item -LiteralPath $ReportPath).FullName
    asynchronous_inputs='No input delay relationship is asserted for external I2C SCL/SDA. Internal timing pass is not an electrical or hardware functional validation.'
    sha256=$debugHashes
}
$debugManifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $ManifestPath -Encoding UTF8
Write-Host "ToF debug timing PASS: 50MHz constraint, Fmax=${debugFmax}MHz; setup=$debugSetup hold=$debugHold."
Write-Host "Recovery: $($debugRecovery.reported_paths) reported paths, worst=$($debugRecovery.worst_reported_slack_ns)ns; Removal: $($debugRemoval.reported_paths) reported paths, worst=$($debugRemoval.worst_reported_slack_ns)ns."
Write-Host "Source/bitstream SHA256 manifest: $ManifestPath"
