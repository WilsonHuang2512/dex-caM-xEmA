Write-Host "=== Two-Stage CCTag Processing ===" -ForegroundColor Cyan

Write-Host "`n[1/2] Detecting CCTag markers..." -ForegroundColor Yellow
& .\cctag_test.exe $args[0]

if (-not (Test-Path "cctag_markers.txt")) {
    Write-Host "ERROR: CCTag detection failed!" -ForegroundColor Red
    exit 1
}

Write-Host "`n[2/2] Processing depth and planes..." -ForegroundColor Yellow
& .\xsctt.exe $args[0] $args[1] cctag_markers.txt

Write-Host "`n=== Complete! ===" -ForegroundColor Green