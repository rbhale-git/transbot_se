# =============================================================================
# run_discovery.ps1 — Phase 1: run the read-only interface discovery on the
# Jetson and pull the results back into the repo.
#
# PREREQUISITE: the robot bringup must already be running on the Jetson in a
# separate terminal:
#     ssh <user>@<ip>
#     roslaunch transbot_bringup bringup.launch
#
# Usage:  .\tools\run_discovery.ps1 -JetsonIp 192.168.1.11 -User jetson
# Output: discovery\transbot_discovery.txt (then authored into FINDINGS.md)
# =============================================================================
param(
    [Parameter(Mandatory = $true)][string]$JetsonIp,
    [Parameter(Mandatory = $true)][string]$User
)

$repoRoot = Split-Path $PSScriptRoot -Parent
$target = "$User@$JetsonIp"

Write-Host "Reminder: bringup.launch must be running on the robot already."
Write-Host ""

Write-Host "1/3 Copying discovery.sh to the robot..."
scp (Join-Path $PSScriptRoot "discovery.sh") "${target}:~/discovery.sh"
if ($LASTEXITCODE -ne 0) { Write-Host "scp failed" -ForegroundColor Red; exit 1 }

Write-Host "2/3 Running discovery (read-only) on the robot..."
ssh $target "bash ~/discovery.sh"
if ($LASTEXITCODE -ne 0) { Write-Host "remote discovery failed" -ForegroundColor Red; exit 1 }

Write-Host "3/3 Fetching results..."
$outDir = Join-Path $repoRoot "discovery"
New-Item -ItemType Directory -Force $outDir | Out-Null
scp "${target}:~/transbot_discovery.txt" (Join-Path $outDir "transbot_discovery.txt")
if ($LASTEXITCODE -ne 0) { Write-Host "fetch failed" -ForegroundColor Red; exit 1 }

Write-Host ""
Write-Host "Done: discovery\transbot_discovery.txt" -ForegroundColor Green
Write-Host "Next: author FINDINGS.md from it and update the TO-VERIFY entries in dashboard/js/config.js."
