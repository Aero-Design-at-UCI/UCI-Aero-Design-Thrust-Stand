param(
    [switch]$InstallOnly
)

$ErrorActionPreference = "Stop"
$Project = Split-Path -Parent $MyInvocation.MyCommand.Path
$Venv = Join-Path $Project ".venv"
$Python = Join-Path $Venv "Scripts\python.exe"
$InteractiveFile = Join-Path $Project "logger\vscode_live_plot.py"

if (-not (Test-Path $Python)) {
    $Launcher = Get-Command py -ErrorAction SilentlyContinue
    if ($Launcher) {
        & $Launcher.Source -3 -m venv $Venv
    } else {
        $Launcher = Get-Command python -ErrorAction SilentlyContinue
        if (-not $Launcher) {
            throw "Python 3 was not found. Install Python 3, then run this script again."
        }
        & $Launcher.Source -m venv $Venv
    }
}

& $Python -m pip install -r (Join-Path $Project "logger\requirements.txt")
& $Python -m ipykernel install --user --name uci-thrust-stand --display-name "Python (UCI Thrust Stand)"

Write-Host ""
Write-Host "VS Code/Jupyter environment is ready."
Write-Host "Open logger\vscode_live_plot.py and run its cells with Shift+Enter."
Write-Host "Select the 'Python (UCI Thrust Stand)' kernel if VS Code asks."

if (-not $InstallOnly) {
    $Code = Get-Command code -ErrorAction SilentlyContinue
    if ($Code) {
        & $Code.Source -r $Project
        & $Code.Source -r $InteractiveFile
    } else {
        Write-Host "The VS Code command-line launcher was not found; open this folder manually:"
        Write-Host $Project
    }
}
