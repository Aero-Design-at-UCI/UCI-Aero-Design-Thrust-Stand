param(
    [string]$Port = "COM7",
    [switch]$Wifi,
    [string]$WifiHost = "192.168.4.1",
    [int]$TcpPort = 3333
)

$ErrorActionPreference = "Stop"
$Project = Split-Path -Parent $MyInvocation.MyCommand.Path
$Venv = Join-Path $Project ".venv"
$Python = Join-Path $Venv "Scripts\python.exe"

if (-not (Test-Path $Python)) {
    $Launcher = Get-Command py -ErrorAction SilentlyContinue
    if ($Launcher) {
        & $Launcher.Source -3 -m venv $Venv
    } else {
        $Launcher = Get-Command python -ErrorAction SilentlyContinue
        if ($Launcher) {
            & $Launcher.Source -m venv $Venv
        } else {
            $BundledPython = Join-Path $env:USERPROFILE ".cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
            if (-not (Test-Path $BundledPython)) {
                throw "Python 3 was not found. Install Python 3, then run this script again."
            }
            & $BundledPython -m venv $Venv
        }
    }
    & $Python -m pip install -r (Join-Path $Project "logger\requirements.txt")
}

if ($Wifi) {
    & $Python (Join-Path $Project "logger\realtime_logger.py") --wifi --host $WifiHost --tcp-port $TcpPort
} else {
    & $Python (Join-Path $Project "logger\realtime_logger.py") --port $Port
}
