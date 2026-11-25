# Builds the A240 with the MSFS 2024 SDK package tool.
#
#   powershell -ExecutionPolicy Bypass -File tools\build_sdk.ps1              # models + package
#   powershell -ExecutionPolicy Bypass -File tools\build_sdk.ps1 -SkipModel   # package only
#   powershell -ExecutionPolicy Bypass -File tools\build_sdk.ps1 -Rebuild     # ignore the build cache
#   powershell -ExecutionPolicy Bypass -File tools\build_sdk.ps1 -Previews    # also re-render previews
#
# The models are exported from blender\A240.blend with the SDK's Blender exporter.
# Set $env:BLENDER to blender.exe if Blender is not in the default 5.2 install folder.
#
# Output: sdk-project\Packages\local-aircraft-a240  (the folder that goes in Community)
param([switch]$SkipModel, [switch]$Rebuild, [switch]$Previews)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$project = Join-Path $root 'sdk-project\A240_Project.xml'

$sdk = $env:MSFS2024_SDK
if (-not $sdk) { $sdk = [Environment]::GetEnvironmentVariable('MSFS2024_SDK', 'Machine') }
if (-not $sdk) { $sdk = 'C:\MSFS 2024 SDK' }
$tool = Join-Path $sdk 'Tools\bin\fspackagetool.exe'
if (-not (Test-Path $tool)) { throw "Package tool not found at $tool - is the MSFS 2024 SDK installed?" }

if (-not $SkipModel) {
    $blender = $env:BLENDER
    if (-not $blender) { $blender = 'C:\Program Files\Blender Foundation\Blender 5.2\blender.exe' }
    if (-not (Test-Path $blender)) { throw "Blender not found at $blender - set `$env:BLENDER to blender.exe." }
    $blend = Join-Path $root 'blender\A240.blend'
    Write-Host 'Exporting models from blender\A240.blend...'
    $exportArgs = @('-b', $blend, '--python-exit-code', '1', '--python', (Join-Path $PSScriptRoot 'blender\export_a240.py'), '--')
    if ($Previews) { $exportArgs += '--previews' }
    & $blender @exportArgs
    if ($LASTEXITCODE -ne 0) { throw 'Model export failed.' }
}

$errorReport = Join-Path $root 'sdk-project\_PackageInt\_RPTErrors.xml'
if (Test-Path $errorReport) { Remove-Item $errorReport -Confirm:$false }
$started = Get-Date

Write-Host 'Running the SDK package tool (this starts the simulator in build mode)...'
$toolArgs = @($project, '-nopause')
if ($Rebuild) { $toolArgs += '-rebuild' }
# Start-Process -Wait: the tool is a console app, but wait for it explicitly all the same.
$p = Start-Process -FilePath $tool -ArgumentList ($toolArgs | ForEach-Object { '"{0}"' -f $_ }) -Wait -PassThru -NoNewWindow
if ($p.ExitCode -ne 0) { throw "Package tool exited with code $($p.ExitCode)." }

$package = Join-Path $root 'sdk-project\Packages\local-aircraft-a240'
$layout = Join-Path $package 'layout.json'
if (-not (Test-Path $layout)) { throw 'No layout.json was produced - the build failed.' }
if ((Get-Item $layout).LastWriteTime -lt $started.AddSeconds(-2) -and $Rebuild) { throw 'layout.json was not refreshed - the build did not run.' }

# The builder lists problems in _RPTErrors.xml; an empty <RPTErrors/> element means none.
$problems = @()
if (Test-Path $errorReport) {
    $xml = [xml](Get-Content $errorReport -Raw)
    $problems = @($xml.DocumentElement.ChildNodes | Where-Object { $_ })
}
if ($problems.Count -gt 0) {
    Write-Host "The builder reported $($problems.Count) problem(s):" -ForegroundColor Yellow
    $problems | ForEach-Object { Write-Host "  $($_.OuterXml)" }
    exit 1
}

$files = (Get-Content $layout -Raw | ConvertFrom-Json).content
Write-Host "Built $($files.Count) files with no problems reported:"
Write-Host "  $package"
