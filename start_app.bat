@echo off
chcp 65001 >nul
cd /d "%~dp0"
if /I "%~1"=="--silent" goto START_SILENT
if exist "%~dp0.venv\Scripts\python.exe" goto USE_LOCAL_VENV
if exist "D:\OptiGSRigg\.venv\Scripts\python.exe" goto USE_DEV_VENV
python "%~dp0app.py" %*
goto CHECK_ERR

:START_SILENT
if exist "%~dp0.venv\Scripts\pythonw.exe" (
    start "" "%~dp0.venv\Scripts\pythonw.exe" "%~dp0app.py"
    exit /b 0
)
if exist "D:\OptiGSRigg\.venv\Scripts\pythonw.exe" (
    start "" "D:\OptiGSRigg\.venv\Scripts\pythonw.exe" "%~dp0app.py"
    exit /b 0
)
start "" pythonw "%~dp0app.py"
exit /b 0

:USE_LOCAL_VENV
"%~dp0.venv\Scripts\python.exe" "%~dp0app.py" %*
goto CHECK_ERR

:USE_DEV_VENV
"D:\OptiGSRigg\.venv\Scripts\python.exe" "%~dp0app.py" %*

:CHECK_ERR
if errorlevel 1 pause
