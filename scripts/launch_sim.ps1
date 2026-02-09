# Iluvatar Simulator Launcher
#
# Reads config/simulator.toml, then starts:
#   1. iluvatar-server
#   2. iluvatar-simulator (render mode)
#   3. One iluvatar-camera process per [[cameras]] entry
#
# Press Ctrl+C to stop all processes.
#
# Usage:
#   .\scripts\launch_sim.ps1
#   .\scripts\launch_sim.ps1 -Config path\to\simulator.toml

param(
    [string]$Config = "config\simulator.toml"
)

$ErrorActionPreference = "Stop"

# ---- Parse TOML config (minimal parser for what we need) ----

if (-not (Test-Path $Config)) {
    Write-Error "Config file not found: $Config"
    exit 1
}

Write-Host "Reading config from $Config" -ForegroundColor Cyan
$content = Get-Content $Config -Raw

# Extract server address
if ($content -match 'address\s*=\s*"([^"]+)"') {
    $serverAddress = $Matches[1]
} else {
    $serverAddress = "localhost:4433"
}

# Extract grid origin
$gridOriginLat = 0.0; $gridOriginLon = 0.0; $gridOriginAlt = 0.0
if ($content -match 'origin\s*=\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]') {
    $gridOriginLat = $Matches[1]
    $gridOriginLon = $Matches[2]
    $gridOriginAlt = $Matches[3]
}

# Parse camera entries
$cameras = @()
$cameraBlocks = [regex]::Matches($content, '\[\[cameras\]\](.*?)(?=\[\[cameras\]\]|\z)', [System.Text.RegularExpressions.RegexOptions]::Singleline)

foreach ($block in $cameraBlocks) {
    $text = $block.Groups[1].Value
    $cam = @{}

    if ($text -match 'id\s*=\s*(\d+)') { $cam.id = [int]$Matches[1] }
    if ($text -match 'stream_port\s*=\s*(\d+)') { $cam.stream_port = [int]$Matches[1] }
    if ($text -match 'fov_horizontal\s*=\s*([-\d.]+)') { $cam.fov_h = [double]$Matches[1] }
    if ($text -match 'fov_vertical\s*=\s*([-\d.]+)') { $cam.fov_v = [double]$Matches[1] }

    if ($text -match 'resolution\s*=\s*\[\s*(\d+)\s*,\s*(\d+)\s*\]') {
        $cam.width = [int]$Matches[1]
        $cam.height = [int]$Matches[2]
    }
    if ($text -match 'position\s*=\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]') {
        $cam.pos_x = $Matches[1]; $cam.pos_y = $Matches[2]; $cam.pos_z = $Matches[3]
    }
    if ($text -match 'look_at\s*=\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]') {
        $cam.look_x = $Matches[1]; $cam.look_y = $Matches[2]; $cam.look_z = $Matches[3]
    }

    $cameras += $cam
}

Write-Host "Found $($cameras.Count) cameras, server=$serverAddress" -ForegroundColor Cyan

# ---- Build if needed ----
Write-Host "`nBuilding workspace..." -ForegroundColor Yellow
cargo build -p iluvatar-server -p iluvatar-camera -p iluvatar-simulator
if ($LASTEXITCODE -ne 0) {
    Write-Error "Build failed"
    exit 1
}

$processes = @()
$tempFiles = @()

try {
    # ---- Start server ----
    Write-Host "`nStarting iluvatar-server..." -ForegroundColor Green
    $serverProc = Start-Process -FilePath "cargo" -ArgumentList "run -p iluvatar-server" `
        -PassThru -NoNewWindow
    $processes += $serverProc
    Start-Sleep -Seconds 2

    # ---- Start simulator ----
    Write-Host "Starting iluvatar-simulator (render mode)..." -ForegroundColor Green
    $simProc = Start-Process -FilePath "cargo" `
        -ArgumentList "run -p iluvatar-simulator -- --render --config $Config" `
        -PassThru -NoNewWindow
    $processes += $simProc
    Start-Sleep -Seconds 3

    # ---- Start camera processes ----
    foreach ($cam in $cameras) {
        $camId = $cam.id
        $tmpConfig = [System.IO.Path]::GetTempPath() + "iluvatar_cam_${camId}.toml"
        $tempFiles += $tmpConfig

        # Compute focal lengths from FOV and resolution
        $fovHRad = $cam.fov_h * [Math]::PI / 180.0
        $fovVRad = $cam.fov_v * [Math]::PI / 180.0
        $fx = ($cam.width / 2.0) / [Math]::Tan($fovHRad / 2.0)
        $fy = ($cam.height / 2.0) / [Math]::Tan($fovVRad / 2.0)
        $cx = $cam.width / 2.0
        $cy = $cam.height / 2.0

        # Compute yaw from position→look_at vector (in XZ plane, Y is up)
        $dx = [double]$cam.look_x - [double]$cam.pos_x
        $dz = [double]$cam.look_z - [double]$cam.pos_z
        # atan2(dx, -dz) gives angle from north (Bevy -Z is forward)
        $yaw = [Math]::Atan2($dx, -$dz) * 180.0 / [Math]::PI

        $dy = [double]$cam.look_y - [double]$cam.pos_y
        $dist_xz = [Math]::Sqrt($dx * $dx + $dz * $dz)
        $pitch = [Math]::Atan2($dy, $dist_xz) * 180.0 / [Math]::PI

        $configContent = @"
[identity]
camera_id = $camId

[hardware]
device = "tcp:localhost:$($cam.stream_port)"
width = $($cam.width)
height = $($cam.height)
fps = 60

[hardware.calibration]
model = "pinhole"
fx = $fx
fy = $fy
cx = $cx
cy = $cy
distortion = []

[localization]
gps_device = "none"
gps_timeout_secs = 1
fixed_orientation = [$([Math]::Round($yaw, 2)), $([Math]::Round($pitch, 2)), 0.0]

[processing]
difference_threshold = 25
motion_threshold_fraction = 0.001

[processing.grid_origin]
latitude = $gridOriginLat
longitude = $gridOriginLon
altitude = $gridOriginAlt

[processing.raymarch]
max_distance = 1500.0
step_size = 0.5

[network]
server_address = "$serverAddress"
connection_timeout_secs = 30
frame_buffer_size = 4
max_reconnect_attempts = 100
reconnect_timeout_secs = 1800
heartbeat_interval_secs = 15

[network.tls]
dangerous_skip_verification = true
"@

        $configContent | Set-Content -Path $tmpConfig -Encoding UTF8
        Write-Host "Starting camera $camId (tcp:localhost:$($cam.stream_port), config=$tmpConfig)..." -ForegroundColor Green

        $camProc = Start-Process -FilePath "cargo" `
            -ArgumentList "run -p iluvatar-camera -- $tmpConfig" `
            -PassThru -NoNewWindow
        $processes += $camProc

        Start-Sleep -Milliseconds 500
    }

    Write-Host "`n=== All processes started ===" -ForegroundColor Cyan
    Write-Host "  Server:    PID $($serverProc.Id)" -ForegroundColor White
    Write-Host "  Simulator: PID $($simProc.Id)" -ForegroundColor White
    foreach ($cam in $cameras) {
        Write-Host "  Camera $($cam.id): stream_port=$($cam.stream_port)" -ForegroundColor White
    }
    Write-Host "`nPress Ctrl+C to stop all processes...`n" -ForegroundColor Yellow

    # Wait for any process to exit
    while ($true) {
        $exited = $processes | Where-Object { $_.HasExited }
        if ($exited) {
            foreach ($p in $exited) {
                Write-Host "Process PID $($p.Id) exited with code $($p.ExitCode)" -ForegroundColor Red
            }
            break
        }
        Start-Sleep -Seconds 1
    }
}
finally {
    Write-Host "`nStopping all processes..." -ForegroundColor Yellow
    foreach ($proc in $processes) {
        if (-not $proc.HasExited) {
            try {
                Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
                Write-Host "  Stopped PID $($proc.Id)" -ForegroundColor Gray
            } catch {}
        }
    }

    # Clean up temp config files
    foreach ($tmp in $tempFiles) {
        if (Test-Path $tmp) {
            Remove-Item $tmp -Force
        }
    }

    Write-Host "All processes stopped." -ForegroundColor Green
}
