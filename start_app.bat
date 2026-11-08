@echo off
cd /d "%~dp0"
if exist "D:\OptiGSRigg\.venv\Scripts\python.exe" (
    "D:\OptiGSRigg\.venv\Scripts\python.exe" app.py
) else (
    python app.py
)
pause
