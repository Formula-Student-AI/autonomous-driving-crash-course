# Install the flyscreen live-control extension into an existing flyscreen checkout.
#
#   powershell -ExecutionPolicy Bypass -File fruitfly\flyscreen_live\install.ps1
#   powershell -ExecutionPolicy Bypass -File fruitfly\flyscreen_live\install.ps1 -Flyscreen D:\fly\sesame-flyscreen
#
# Every replaced file is backed up under <flyscreen>\.flyscreen_live_backup\<timestamp>.

param(
    [string]$Flyscreen = $env:FLYSCREEN_DIR,
    [switch]$NoBackup
)

$ErrorActionPreference = "Stop"
if (-not $Flyscreen) {
    $Flyscreen = "C:\Users\V9\flyscreen-portable\flyscreen-portable\sesame-flyscreen"
}
if (-not (Test-Path (Join-Path $Flyscreen "webui.py"))) {
    throw "webui.py not found in $Flyscreen - point -Flyscreen at sesame-flyscreen"
}

$files = @(
    @{ src = "flyscreen\live.py";       dst = "flyscreen\live.py" },
    @{ src = "flyscreen\config.py";     dst = "flyscreen\config.py" },
    @{ src = "flyscreen\web\server.py"; dst = "flyscreen\web\server.py" },
    @{ src = "webui.py";                dst = "webui.py" },
    @{ src = "live_webui.py";           dst = "live_webui.py" }
)

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$backup = Join-Path $Flyscreen ".flyscreen_live_backup\$stamp"

foreach ($f in $files) {
    $src = Join-Path $PSScriptRoot $f.src
    $dst = Join-Path $Flyscreen $f.dst
    if (-not (Test-Path $src)) { throw "missing template file: $src" }
    if (-not $NoBackup -and (Test-Path $dst)) {
        $b = Join-Path $backup $f.dst
        New-Item -ItemType Directory -Force -Path (Split-Path $b -Parent) | Out-Null
        Copy-Item -Path $dst -Destination $b -Force
    }
    New-Item -ItemType Directory -Force -Path (Split-Path $dst -Parent) | Out-Null
    Copy-Item -Path $src -Destination $dst -Force
    Write-Host "installed $($f.dst)"
}

Write-Host ""
if (-not $NoBackup) { Write-Host "backup: $backup" }
Write-Host "next: fruitfly\tools\start_flyscreen_live.cmd"
