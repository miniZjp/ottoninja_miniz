# PowerShell script to create merged binary file
# Usage: .\create_merged_bin.ps1

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "Creating Merged Binary File for ESP32" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host ""

# Check if build directory exists
if (-not (Test-Path "build")) {
    Write-Host "Error: Build directory not found!" -ForegroundColor Red
    Write-Host "Please build the project first using: idf.py build" -ForegroundColor Yellow
    Write-Host ""
    pause
    exit 1
}

# Check if flasher_args.json exists
$flasherArgsPath = "build\flasher_args.json"
if (-not (Test-Path $flasherArgsPath)) {
    Write-Host "Error: flasher_args.json not found!" -ForegroundColor Red
    Write-Host "Please run 'idf.py build' to generate build artifacts" -ForegroundColor Yellow
    Write-Host ""
    pause
    exit 1
}

Write-Host "Running Python script to create merged binary..." -ForegroundColor Green
Write-Host ""

# Run the Python script
try {
    python scripts\create_merged_bin.py
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host ""
        Write-Host "============================================================" -ForegroundColor Green
        Write-Host "SUCCESS! Merged binary created!" -ForegroundColor Green
        Write-Host "============================================================" -ForegroundColor Green
        Write-Host ""
        
        # Get com ports
        Write-Host "Available COM ports:" -ForegroundColor Cyan
        $ports = [System.IO.Ports.SerialPort]::getportnames()
        if ($ports.Count -gt 0) {
            foreach ($port in $ports) {
                Write-Host "  - $port" -ForegroundColor Yellow
            }
            Write-Host ""
            Write-Host "To flash, use:" -ForegroundColor Cyan
            Write-Host "  esptool.py --chip esp32s3 --port COMX write_flash 0x0 build\ninjaottoAI.bin" -ForegroundColor White
        } else {
            Write-Host "  No COM ports detected" -ForegroundColor Yellow
        }
    } else {
        Write-Host ""
        Write-Host "Failed to create merged binary!" -ForegroundColor Red
        exit 1
    }
} catch {
    Write-Host ""
    Write-Host "Error running script: $_" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
