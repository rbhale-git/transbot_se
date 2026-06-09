# =============================================================================
# setup_ssh.ps1 — Phase 0: passwordless SSH from this laptop to the Jetson.
#
# Generates an ed25519 key if none exists, installs the PUBLIC key on the
# robot (you type the robot's password ONCE, interactively — it is never
# stored anywhere), then verifies passwordless login.
#
# Usage:  .\tools\setup_ssh.ps1 -JetsonIp 192.168.1.11 -User jetson
# =============================================================================
param(
    [Parameter(Mandatory = $true)][string]$JetsonIp,
    [Parameter(Mandatory = $true)][string]$User
)

$keyPath = Join-Path $env:USERPROFILE ".ssh\id_ed25519"

if (-not (Test-Path $keyPath)) {
    Write-Host "No SSH key found - generating ed25519 key at $keyPath"
    # '""' (not '') for the empty passphrase: PowerShell 5.1 drops empty-string
    # arguments to native executables entirely, which breaks ssh-keygen.
    & ssh-keygen -t ed25519 -f $keyPath -N '""' -C "transbot-dashboard"
} else {
    Write-Host "Using existing key: $keyPath"
}

if (-not (Test-Path "$keyPath.pub")) {
    Write-Host "Key generation failed - no public key at $keyPath.pub" -ForegroundColor Red
    exit 1
}
$pubKey = (Get-Content "$keyPath.pub" -Raw).Trim()

Write-Host ""
Write-Host "Installing public key on $User@$JetsonIp - enter the ROBOT's password when prompted."
Write-Host "NOTE: the prompt shows NOTHING while you type (no dots) - that is normal."
Write-Host "      This is the robot's LOGIN password (Yahboom default: yahboom),"
Write-Host "      NOT the Transbot Wi-Fi password."
# sort -u afterwards keeps authorized_keys free of duplicates on re-runs.
$remoteCmd = "mkdir -p ~/.ssh; chmod 700 ~/.ssh; echo '$pubKey' >> ~/.ssh/authorized_keys; sort -u ~/.ssh/authorized_keys -o ~/.ssh/authorized_keys; chmod 600 ~/.ssh/authorized_keys"
ssh "$User@$JetsonIp" $remoteCmd
if ($LASTEXITCODE -ne 0) {
    Write-Host "Key install failed (wrong password or host unreachable)." -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Verifying passwordless login..."
ssh -o BatchMode=yes "$User@$JetsonIp" "echo PASSWORDLESS-OK on \$(hostname); uname -m; lsb_release -ds 2>/dev/null"
if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "Phase 0 SSH setup complete." -ForegroundColor Green
} else {
    Write-Host "Passwordless login failed - key was not accepted." -ForegroundColor Red
    exit 1
}
