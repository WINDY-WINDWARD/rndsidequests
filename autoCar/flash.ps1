$cli = "C:\projects\rndsidequests\tools\arduino-cli.exe"
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  AUTOCAR FIRMWARE FLASH TOOL             " -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan

$port = [System.IO.Ports.SerialPort]::GetPortNames() | Where-Object { $_ -ne "COM1" } | Select-Object -First 1
if (-not $port) {
    Write-Host "[!] No NodeMCU USB serial port detected." -ForegroundColor Yellow
    exit 1
}

Write-Host "[+] Found board on $port. Uploading firmware..." -ForegroundColor Green
& $cli upload -p $port --fqbn esp8266:esp8266:nodemcuv2 "C:\projects\rndsidequests\autoCar"

if ($LASTEXITCODE -eq 0) {
    Write-Host "[OK] Firmware successfully uploaded!" -ForegroundColor Green
} else {
    Write-Host "[FAIL] Upload failed with exit code $LASTEXITCODE." -ForegroundColor Red
}
