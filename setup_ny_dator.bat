@echo off
setlocal EnableExtensions
chcp 65001 >nul
cd /d "%~dp0"

set "VERIFY_ONLY=0"
set "NO_PAUSE=0"
for %%A in (%*) do (
    if /I "%%~A"=="--verify-only" set "VERIFY_ONLY=1"
    if /I "%%~A"=="--no-pause" set "NO_PAUSE=1"
)

echo ============================================================
echo   OptiGSRigg Studio - Installation for Ny Dator (Windows)
echo ============================================================
echo.

:: 1. Hitta riktig Python >= 3.10 (skyddar mot Windows Store-stub)
set "PY_CMD="
where py >nul 2>nul
if errorlevel 1 goto CHECK_PYTHON_EXE
py -3 -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>nul
if not errorlevel 1 set "PY_CMD=py -3"
if defined PY_CMD goto PYTHON_OK

:CHECK_PYTHON_EXE
where python >nul 2>nul
if errorlevel 1 goto PYTHON_MISSING
python -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>nul
if not errorlevel 1 set "PY_CMD=python"
if defined PY_CMD goto PYTHON_OK

:PYTHON_MISSING
echo [SAKNAS] Python 3.10 eller nyare hittades inte pa denna dator.
if "%VERIFY_ONLY%"=="1" goto FAIL_EXIT
where winget >nul 2>nul
if errorlevel 1 goto PYTHON_MANUAL
echo.
choice /C JN /M "Vill du installera Python 3.11 automatiskt via winget nu"
if errorlevel 2 goto PYTHON_MANUAL
winget install -e --id Python.Python.3.11 --scope machine
if errorlevel 1 winget install -e --id Python.Python.3.11 --scope user
echo.
echo [KLAR] Starta ett nytt terminalfonster eller dubbelklicka pa setup_ny_dator.bat igen nu nar Python ar installerat.
goto END_PAUSE

:PYTHON_MANUAL
echo Ladda ner Python 3.11 fran https://www.python.org/downloads/windows/
echo VIKTIGT: Kryssa i "Add python.exe to PATH" vid installationen, och kor sedan setup_ny_dator.bat igen.
goto FAIL_PAUSE

:PYTHON_OK
echo [OK] Anvander Python: %PY_CMD%

if "%VERIFY_ONLY%"=="1" goto VERIFY_PICK_PY

:: 2. Kontrollera lokal .venv (skapa ny om den saknas eller kopierades trasig fran gammal dator)
if not exist "%~dp0.venv\Scripts\python.exe" goto CREATE_VENV
if not exist "%~dp0.venv\Scripts\pythonw.exe" goto CREATE_VENV
"%~dp0.venv\Scripts\python.exe" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>nul
if not errorlevel 1 goto VENV_READY
echo [INFO] Befintlig .venv verkar kopierad fran en annan dator - bygger ny ren .venv...
rmdir /s /q "%~dp0.venv"
if exist "%~dp0.venv" (
    echo [FEL] Kunde inte ta bort gammal .venv. Stang eventuella Python-fonster och prova igen.
    goto FAIL_PAUSE
)

:CREATE_VENV
echo [INFO] Skapar lokal Python-miljo ^(.venv^)...
%PY_CMD% -m venv "%~dp0.venv"
if errorlevel 1 (
    echo [FEL] Kunde inte skapa .venv med %PY_CMD%.
    goto FAIL_PAUSE
)

:VENV_READY
set "RUN_PY=%~dp0.venv\Scripts\python.exe"
echo [INFO] Uppdaterar pip och installerar paket fran requirements.txt...
"%RUN_PY%" -m pip install --upgrade pip
if errorlevel 1 echo [VARNING] Kunde inte uppgradera pip - fortsatter med requirements.txt...
"%RUN_PY%" -m pip install -r "%~dp0requirements.txt"
if errorlevel 1 (
    echo [FEL] Kunde inte installera paketen i requirements.txt. Kontrollera internetanslutningen.
    goto FAIL_PAUSE
)
goto CHECK_FFMPEG

:VERIFY_PICK_PY
set "RUN_PY=%PY_CMD%"
if exist "%~dp0.venv\Scripts\python.exe" set "RUN_PY=%~dp0.venv\Scripts\python.exe"
if not exist "%~dp0.venv\Scripts\python.exe" if exist "D:\OptiGSRigg\.venv\Scripts\python.exe" set "RUN_PY=D:\OptiGSRigg\.venv\Scripts\python.exe"
goto CHECK_FFMPEG

:CHECK_FFMPEG
set "HAS_FFMPEG=1"
where ffmpeg >nul 2>nul
if errorlevel 1 set "HAS_FFMPEG=0"
where ffprobe >nul 2>nul
if errorlevel 1 set "HAS_FFMPEG=0"
if "%HAS_FFMPEG%"=="1" (
    echo [OK] FFmpeg och FFprobe finns redan i PATH.
    goto RUN_CHECKER
)
echo [VARNING] FFmpeg/FFprobe saknas i PATH - behovs for ljudsynk, miniatyrer och EXIF.
if "%VERIFY_ONLY%"=="1" goto RUN_CHECKER
where winget >nul 2>nul
if errorlevel 1 (
    echo [INFO] Installera FFmpeg manuellt eller med winget senare: winget install Gyan.FFmpeg
    goto RUN_CHECKER
)
choice /C JN /M "Vill du installera FFmpeg automatiskt via winget nu"
if errorlevel 2 goto RUN_CHECKER
winget install -e --id Gyan.FFmpeg
echo [INFO] Om FFmpeg precis installerades behover eventuellt en ny terminal oppnas for att PATH ska laddas om.

:RUN_CHECKER
if "%VERIFY_ONLY%"=="1" (
    "%RUN_PY%" "%~dp0src\setup_new_pc_check.py" --no-desktop-shortcut
) else (
    "%RUN_PY%" "%~dp0src\setup_new_pc_check.py" %*
)
if errorlevel 1 (
    echo.
    echo [FEL] Efterkontrollen hittade ett problem. Se loggen ovan.
    goto FAIL_PAUSE
)

echo.
echo ============================================================
echo   KLART! OptiGSRigg Studio ar redo pa denna dator.
echo   Starta med "OptiGSRigg Studio" pa Skrivbordet eller i mappen!
echo ============================================================
goto END_PAUSE

:FAIL_PAUSE
if "%VERIFY_ONLY%"=="1" exit /b 1
if "%NO_PAUSE%"=="1" exit /b 1
pause
exit /b 1

:FAIL_EXIT
exit /b 1

:END_PAUSE
if "%VERIFY_ONLY%"=="1" exit /b 0
if "%NO_PAUSE%"=="1" exit /b 0
pause
exit /b 0
