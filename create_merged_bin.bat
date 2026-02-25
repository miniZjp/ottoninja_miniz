@echo off
REM Script to create merged binary file for ESP32
REM Makes flashing easier by combining all binaries into one file

echo ============================================================
echo Creating Merged Binary File
echo ============================================================
echo.

REM Check if build directory exists
if not exist "build\" (
    echo Error: Build directory not found!
    echo Please build the project first using: idf.py build
    echo.
    pause
    exit /b 1
)

REM Run the Python script to create merged binary
python scripts\create_merged_bin.py

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo Failed to create merged binary!
    pause
    exit /b 1
)

echo.
echo ============================================================
echo Merged binary created successfully in build\ directory
echo You can now flash it with a single command!
echo ============================================================
echo.
pause
