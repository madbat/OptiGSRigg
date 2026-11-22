param([switch]$SkipDesktop)

$ErrorActionPreference = "Stop"
$AppRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$AppPy = Join-Path $AppRoot "app.py"
$IcoPath = Join-Path $AppRoot "assets\icons\optigsrigg.ico"

if (-not (Test-Path $AppPy)) {
    throw "Saknar app.py i $AppRoot"
}
if (-not (Test-Path $IcoPath)) {
    throw "Saknar $IcoPath - kör 'python src/build_app_icon.py' först."
}

$PywCandidates = @(
    (Join-Path $AppRoot ".venv\Scripts\pythonw.exe"),
    "D:\OptiGSRigg\.venv\Scripts\pythonw.exe",
    (Get-Command pythonw -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source)
)
$PywPath = $PywCandidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
if (-not $PywPath) {
    throw "Saknar pythonw.exe (varken i .venv\Scripts\pythonw.exe eller PATH). Kör setup_ny_dator.bat först."
}

$ws = New-Object -ComObject WScript.Shell
$Targets = @(
    (Join-Path $AppRoot "OptiGSRigg Studio.lnk")
)

if (-not $SkipDesktop) {
    $DesktopDir = [Environment]::GetFolderPath("Desktop")
    if (-not $DesktopDir -or -not (Test-Path $DesktopDir)) {
        throw "Kunde inte nå Skrivbordsmappen ($DesktopDir)."
    }
    $Targets += (Join-Path $DesktopDir "OptiGSRigg Studio.lnk")
}

foreach ($lnk in $Targets) {
    $sc = $ws.CreateShortcut($lnk)
    $sc.TargetPath = $PywPath
    $sc.Arguments = "`"$AppPy`""
    $sc.WorkingDirectory = $AppRoot
    $sc.IconLocation = "$IcoPath,0"
    $sc.Description = "OptiGSRigg Studio - 3x DJI Action 4 till 3DGUT Pipeline"
    $sc.Save()
    Write-Host "[OK] Skapade genväg: $lnk"
}
