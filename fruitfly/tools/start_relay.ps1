# Start the WSL relay that lets the dev container reach the Windows flyscreen.
#
#   powershell -ExecutionPolicy Bypass -File fruitfly\tools\start_relay.ps1
#
# Container side default: http://172.17.0.1:8780

param(
    [int]$ListenPort = 8780,
    [string]$TargetHost = "127.0.0.1",
    [int]$TargetPort = 8760
)

$ErrorActionPreference = "Stop"
$script = Join-Path $PSScriptRoot "wsl_relay.py"
if (-not (Test-Path $script)) {
    throw "wsl_relay.py not found next to this script"
}

# Convert the Windows path to something WSL can open.
$wslPath = (wsl -e wslpath -a -u $script).Trim()
if (-not $wslPath) {
    throw "Could not translate $script to a WSL path"
}

Write-Host "WSL relay: 0.0.0.0:$ListenPort -> ${TargetHost}:${TargetPort}"
Write-Host "Container:  http://172.17.0.1:$ListenPort"
Write-Host "Leave this window open; Ctrl-C stops the relay."
wsl -e python3 $wslPath $ListenPort $TargetHost $TargetPort
