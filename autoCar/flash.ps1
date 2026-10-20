param (
    [switch]$BuildOnly,
    [switch]$CompileOnly,
    [string]$OutputDir = ""
)

$sketchDir = $PSScriptRoot
$cli = Join-Path $PSScriptRoot "..\tools\arduino-cli.exe"
if (-not (Test-Path $cli)) {
    $cli = "C:\projects\rndsidequests\tools\arduino-cli.exe"
}

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  AUTOCAR FIRMWARE BUILD & FLASH TOOL     " -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan

if (-not $OutputDir) {
    $OutputDir = Join-Path $sketchDir "build"
}

Write-Host "[+] Compiling firmware binary..." -ForegroundColor Green
& $cli compile --fqbn esp8266:esp8266:nodemcuv2 --output-dir "$OutputDir" "$sketchDir"

if ($LASTEXITCODE -ne 0) {
    Write-Host "[FAIL] Compilation failed with exit code $LASTEXITCODE." -ForegroundColor Red
    exit $LASTEXITCODE
}

$binPath = Join-Path $OutputDir "autoCar.ino.bin"
Write-Host "[OK] Firmware binary ready: $binPath" -ForegroundColor Green
Write-Host "[OTA] Upload via Web Portal: http://192.168.4.1/update" -ForegroundColor Yellow

if ($BuildOnly -or $CompileOnly) {
    exit 0
}

$port = [System.IO.Ports.SerialPort]::GetPortNames() | Where-Object { $_ -ne "COM1" } | Select-Object -First 1
if (-not $port) {
    Write-Host "[!] No NodeMCU USB serial port detected." -ForegroundColor Yellow
    Write-Host "[*] You can upload '$binPath' directly via the web portal at http://192.168.4.1/update" -ForegroundColor Cyan
    exit 0
}

Write-Host "[+] Found board on $port. Uploading firmware via USB..." -ForegroundColor Green
& $cli upload -p $port --fqbn esp8266:esp8266:nodemcuv2 --input-dir "$OutputDir" "$sketchDir"

if ($LASTEXITCODE -eq 0) {
    Write-Host "[OK] Firmware successfully uploaded via USB!" -ForegroundColor Green
} else {
    Write-Host "[FAIL] USB upload failed with exit code $LASTEXITCODE." -ForegroundColor Red
}
