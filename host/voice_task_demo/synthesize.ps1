param(
    [Parameter(Mandatory=$true)][string]$Work,
    [Parameter(Mandatory=$true)][string]$Out
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Speech
$cases = Get-Content -LiteralPath (Join-Path $Work 'cases.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$protocol = Get-Content -LiteralPath (Join-Path $Work 'protocol.json') -Raw -Encoding UTF8 | ConvertFrom-Json
New-Item -ItemType Directory -Path $Out -ErrorAction Stop | Out-Null
$format = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
$records = @()
foreach ($voice in $protocol.voices) {
    $synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
    try {
        $synth.SelectVoice($voice)
        $synth.Rate = 0
        $synth.Volume = 100
        foreach ($case in $cases) {
            $id = (($voice -replace '[^a-zA-Z0-9]', '_') + '_' + $case.id)
            $path = Join-Path $Out ($id + '.wav')
            if (Test-Path -LiteralPath $path) { throw "Output already exists: $path" }
            $synth.SetOutputToWaveFile($path, $format)
            $synth.Speak($case.text)
            $synth.SetOutputToNull()
            $records += @{id=$id;case_id=$case.id;voice=$voice;text=$case.text;wav=$path;rate=0;synthetic=$true}
        }
    } finally { $synth.Dispose() }
}
ConvertTo-Json -InputObject @($records) -Depth 6 | Set-Content -LiteralPath (Join-Path $Out 'manifest.json') -Encoding UTF8
Write-Output ('SYNTHESIZED ' + $records.Count)
