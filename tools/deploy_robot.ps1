# =============================================================================
# deploy_robot.ps1 — Phase 2: copy the robot-side launch file and start script
# to ~/transbot_dashboard/ on the Jetson.
#
# (Package installs — ros-melodic-rosbridge-suite, ros-melodic-web-video-server
# — are intentionally NOT automated; per the brief they happen only after an
# explicit go-ahead.)
#
# Usage:  .\tools\deploy_robot.ps1 -JetsonIp 192.168.1.11 -User jetson
# =============================================================================
param(
    [Parameter(Mandatory = $true)][string]$JetsonIp,
    [Parameter(Mandatory = $true)][string]$User
)

$repoRoot = Split-Path $PSScriptRoot -Parent
$target = "$User@$JetsonIp"

Write-Host "Creating ~/transbot_dashboard on the robot..."
ssh $target "mkdir -p ~/transbot_dashboard"
if ($LASTEXITCODE -ne 0) { Write-Host "ssh failed" -ForegroundColor Red; exit 1 }

Write-Host "Copying launch file and start script..."
scp (Join-Path $repoRoot "robot\transbot_dashboard.launch") `
    (Join-Path $repoRoot "robot\start_dashboard.sh") `
    "${target}:~/transbot_dashboard/"
if ($LASTEXITCODE -ne 0) { Write-Host "scp failed" -ForegroundColor Red; exit 1 }

Write-Host ""
Write-Host "Deployed. Start the stack on the robot with:" -ForegroundColor Green
Write-Host "    ssh $target"
Write-Host "    bash ~/transbot_dashboard/start_dashboard.sh"
