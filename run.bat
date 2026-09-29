@echo off
setlocal enabledelayedexpansion
title N-DOF Arm Simulator

cd /d "%~dp0"

if "%1"=="--debug" goto :DEBUG_LAUNCH
if "%1"=="-d" goto :DEBUG_LAUNCH

:: 1. Check if virtual environment already exists
if exist ".venv\Scripts\python.exe" (
    goto :LAUNCH
)

:: 2. First-time setup: check for system python
echo ===================================================
echo   N-DOF Arm Simulator - First-time Setup
echo ===================================================
echo [INFO] Virtual environment not found. Setting up...

where python >nul 2>nul
if %errorlevel% equ 0 (
    set "PY_EXEC=python"
) else (
    where py >nul 2>nul
    if %errorlevel% equ 0 (
        set "PY_EXEC=py -3"
    ) else (
        echo [ERROR] Python was not found on your system!
        echo Please download and install Python 3.10+ from:
        echo   https://www.python.org/downloads/
        echo (Make sure to check "Add Python to PATH" during installation)
        echo.
        pause
        exit /b 1
    )
)

echo [INFO] Creating virtual environment in .venv...
%PY_EXEC% -m venv .venv
if %errorlevel% neq 0 (
    echo [ERROR] Failed to create virtual environment.
    pause
    exit /b 1
)

echo [INFO] Installing simulator dependencies...
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip --quiet
pip install -e .
if %errorlevel% neq 0 (
    echo [ERROR] Failed to install dependencies.
    pause
    exit /b 1
)
echo [SUCCESS] Setup complete!
echo.

:LAUNCH
echo [INFO] Launching N-DOF Arm Simulator...
start "" ".venv\Scripts\pythonw.exe" -m robokinematics.app.main
exit /b 0

:DEBUG_LAUNCH
echo [DEBUG] Launching in console mode with error tracking...
if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] .venv not found. Run without --debug first to set up.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" -m robokinematics.app.main
if %errorlevel% neq 0 (
    echo [ERROR] Application exited with code %errorlevel%
    pause
)
exit /b %errorlevel%
