param(
    [switch]$Detach
)

Set-StrictMode -Version Latest

# Run from repository root (script expects to be in scripts/)
Push-Location (Split-Path -Parent $MyInvocation.MyCommand.Path) | Out-Null
Pop-Location

Write-Host "Ensure Rancher Desktop is running and 'nerdctl' is available on PATH..."

if (-not (Get-Command nerdctl -ErrorAction SilentlyContinue)) {
    Write-Error "nerdctl not found. Make sure Rancher Desktop is running and nerdctl is on PATH."
    exit 1
}

Write-Host "Building images (this may take a long time on first run)..."
nerdctl compose build --progress=plain

if ($Detach) {
    Write-Host "Starting services in detached mode..."
    nerdctl compose up -d
} else {
    Write-Host "Starting services (foreground). Press Ctrl+C to stop."
    nerdctl compose up
}
