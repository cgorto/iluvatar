# Iluvatar Docker Mode Launcher
#
# Starts the simulator natively (needs GPU/Bevy), then brings up
# server + cameras in Docker containers.
#
# Usage:
#   .\scripts\launch_docker.ps1
#   .\scripts\launch_docker.ps1 -Config path\to\simulator.toml
#   .\scripts\launch_docker.ps1 -SkipBuild   # skip docker compose build

param(
    [string]$Config = "config\simulator.toml",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

# ---- Ensure cargo is in PATH ----
$cargoPath = "$env:USERPROFILE\.cargo\bin"
if (Test-Path $cargoPath) {
    $env:PATH = "$cargoPath;$env:PATH"
}

# ---- Validate config ----
if (-not (Test-Path $Config)) {
    Write-Error "Config file not found: $Config"
    exit 1
}

Write-Host "=== Iluvatar Docker Mode ===" -ForegroundColor Cyan
Write-Host "Config: $Config" -ForegroundColor White

# ---- Build simulator (native, needs GPU) ----
Write-Host "`nBuilding simulator..." -ForegroundColor Yellow
cargo build -p iluvatar-simulator
if ($LASTEXITCODE -ne 0) {
    Write-Error "Simulator build failed"
    exit 1
}

$simBin = "target\debug\iluvatar-simulator.exe"
if (-not (Test-Path $simBin)) {
    Write-Error "Simulator binary not found: $simBin"
    exit 1
}

$processes = @()

try {
    $projectRoot = (Get-Location).Path

    # ---- Start simulator natively ----
    Write-Host "`nStarting simulator (native, render mode)..." -ForegroundColor Green
    $simProc = Start-Process -FilePath $simBin `
        -ArgumentList "--render --config $Config" `
        -WorkingDirectory $projectRoot `
        -PassThru -NoNewWindow
    $processes += $simProc

    # Wait for simulator TCP frame servers to be ready
    Write-Host "Waiting for simulator TCP frame servers..." -ForegroundColor Yellow
    Start-Sleep -Seconds 8

    # ---- Start Docker containers ----
    if ($SkipBuild) {
        Write-Host "`nStarting Docker containers..." -ForegroundColor Green
        docker compose up -d
    } else {
        Write-Host "`nBuilding and starting Docker containers..." -ForegroundColor Green
        docker compose up --build -d
    }

    if ($LASTEXITCODE -ne 0) {
        Write-Error "docker compose up failed"
        throw "Docker failed"
    }

    Write-Host "`n=== All services started ===" -ForegroundColor Cyan
    Write-Host "  Simulator: PID $($simProc.Id) (native)" -ForegroundColor White
    Write-Host "  Server:    docker (QUIC :4433, WebSocket :8080)" -ForegroundColor White
    Write-Host "  Cameras:   docker (cam-0 through cam-4)" -ForegroundColor White
    Write-Host "`n  Web client: http://localhost:8080" -ForegroundColor Yellow
    Write-Host "  Logs:       docker compose logs -f" -ForegroundColor Yellow
    Write-Host "`nPress Ctrl+C to stop all services...`n" -ForegroundColor Yellow

    # Wait for simulator to exit or user interrupt
    while (-not $simProc.HasExited) {
        Start-Sleep -Seconds 1
    }
    Write-Host "Simulator exited with code $($simProc.ExitCode)" -ForegroundColor Red
}
finally {
    Write-Host "`nStopping Docker containers..." -ForegroundColor Yellow
    docker compose down 2>$null

    Write-Host "Stopping simulator..." -ForegroundColor Yellow
    foreach ($proc in $processes) {
        if (-not $proc.HasExited) {
            try {
                Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
                Write-Host "  Stopped PID $($proc.Id)" -ForegroundColor Gray
            } catch {}
        }
    }

    Write-Host "All services stopped." -ForegroundColor Green
}
