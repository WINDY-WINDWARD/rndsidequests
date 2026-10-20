param (
    [string]$OutputDir = ""
)

$flashScript = Join-Path $PSScriptRoot "flash.ps1"
if ($OutputDir) {
    & $flashScript -BuildOnly -OutputDir $OutputDir
} else {
    & $flashScript -BuildOnly
}
