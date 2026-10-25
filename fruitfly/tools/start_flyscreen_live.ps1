# Start flyscreen in live mode (frames come from the ROS 2 bridge).
#
#   powershell -ExecutionPolicy Bypass -File fruitfly\tools\start_flyscreen_live.ps1
#
# Override the checkout with -Flyscreen or $env:FLYSCREEN_DIR.

param(
    [string]$Flyscreen = $env:FLYSCREEN_DIR,
    [int]$Port = 8760,
    [string[]]$ExtraArgs = @("--size", "mid", "--posture", "0.1")
)

$ErrorActionPreference = "Stop"
if (-not $Flyscreen) { $Flyscreen = "C:\Users\V9\flyscreen-portable\flyscreen-portable\sesame-flyscreen" }
if (-not (Test-Path (Join-Path $Flyscreen "live_webui.py"))) {
    throw "live_webui.py not found in $Flyscreen - run fruitfly\flyscreen_live\install.ps1 first"
}

# Portable flyscreen ships its own interpreter next to sesame-flyscreen.
$py = Join-Path (Split-Path $Flyscreen -Parent) "runtime\python\python.exe"
if (-not (Test-Path $py)) { $py = "python" }

Write-Host "flyscreen live -> http://127.0.0.1:$Port/  ($Flyscreen)"
Push-Location $Flyscreen
try {
    & $py live_webui.py --port $Port @ExtraArgs
} finally {
    Pop-Location
}
