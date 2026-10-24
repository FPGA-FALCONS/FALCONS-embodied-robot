param([Parameter(Mandatory=$true)][string]$Python, [Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference = 'Stop'
& $Python "$PSScriptRoot/speech_assets.py" catalog --assets $OutputDirectory
if ($LASTEXITCODE -ne 0) { throw 'Speech catalog creation failed' }
Add-Type -AssemblyName System.Speech
$speaker = New-Object System.Speech.Synthesis.SpeechSynthesizer
try {
    $voice = $speaker.GetInstalledVoices() | Where-Object { $_.Enabled -and $_.VoiceInfo.Culture.Name -eq 'zh-CN' } | Select-Object -First 1
    if ($null -eq $voice) { throw 'No installed Chinese voice; no downloads or installations performed' }
    $speaker.SelectVoice($voice.VoiceInfo.Name)
    $speaker.Rate = 0
    $format = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
    $catalog = Get-Content -LiteralPath (Join-Path $OutputDirectory 'catalog.json') -Encoding UTF8 -Raw | ConvertFrom-Json
    foreach ($entry in $catalog.PSObject.Properties) {
        $target = Join-Path $OutputDirectory ($entry.Name + '.wav')
        if (Test-Path -LiteralPath $target) { throw "Retain existing audio: $target" }
        $speaker.SetOutputToWaveFile($target, $format)
        $speaker.Speak([string]$entry.Value)
        $speaker.SetOutputToNull()
    }
    @{ engine = 'Windows System.Speech'; voice = $voice.VoiceInfo.Name; culture = 'zh-CN'; rate_hz = 16000; generated_at = (Get-Date).ToString('o') } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $OutputDirectory 'generation.json') -Encoding UTF8
} finally { $speaker.Dispose() }
& $Python "$PSScriptRoot/speech_assets.py" index --assets $OutputDirectory
if ($LASTEXITCODE -ne 0) { throw 'Speech audio verification failed' }
