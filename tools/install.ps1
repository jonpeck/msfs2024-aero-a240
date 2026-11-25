# Links the SDK-built A240 package into the MSFS 2024 Community folder (no files are
# copied, so each new build shows up in the sim after the next restart).
#
#   powershell -ExecutionPolicy Bypass -File tools\install.ps1            # install
#   powershell -ExecutionPolicy Bypass -File tools\install.ps1 -Remove    # uninstall
param([switch]$Remove)

$ErrorActionPreference = 'Stop'
$package = Join-Path (Split-Path $PSScriptRoot -Parent) 'sdk-project\Packages\local-aircraft-a240'

# UserCfg.opt tells us where the sim keeps its packages (Steam and MS Store locations).
$cfgs = @(@(
    "$env:APPDATA\Microsoft Flight Simulator 2024\UserCfg.opt",
    "$env:LOCALAPPDATA\Packages\Microsoft.Limitless_8wekyb3d8bbwe\LocalCache\UserCfg.opt"
) | Where-Object { Test-Path $_ })
if (-not $cfgs) { throw 'Could not find the MSFS 2024 UserCfg.opt - is the simulator installed?' }

$line = Select-String -Path $cfgs[0] -Pattern '^\s*InstalledPackagesPath\s+"(.+)"' | Select-Object -First 1
if (-not $line) { throw "No InstalledPackagesPath in $($cfgs[0])" }
$community = Join-Path $line.Matches[0].Groups[1].Value 'Community'
if (-not (Test-Path $community)) { throw "Community folder not found: $community" }

$link = Join-Path $community 'local-aircraft-a240'
if (Test-Path $link) {
    $item = Get-Item $link -Force
    if ($item.LinkType -ne 'Junction') { throw "$link exists and is not a link made by this script - leaving it alone." }
    # Removes only the link; the package in this repo is untouched.
    [System.IO.Directory]::Delete($link)
    Write-Host "Removed link $link"
}
if (-not $Remove) {
    if (-not (Test-Path (Join-Path $package 'layout.json'))) { throw "Nothing built yet - run tools\build_sdk.ps1 first." }
    New-Item -ItemType Junction -Path $link -Target $package | Out-Null
    Write-Host "Linked $link"
    Write-Host "    -> $package"
    Write-Host 'Restart Microsoft Flight Simulator 2024, then look for "Aero A240" (manufacturer Aero).'
}
