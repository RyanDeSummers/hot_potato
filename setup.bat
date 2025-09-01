@echo off
REM Hot Potato Game Setup Script for Windows
REM This script helps set up the ESP32 Hot Potato game project

echo 🥔 Hot Potato Game Setup
echo ========================

REM Check if ESP-IDF is sourced
if "%IDF_PATH%"=="" (
    echo ❌ Error: ESP-IDF not found. Please source ESP-IDF first:
    echo    %IDF_PATH%\export.bat
    echo    or
    echo    call %IDF_PATH%\export.bat
    pause
    exit /b 1
)

echo ✅ ESP-IDF found at: %IDF_PATH%

REM Check ESP-IDF version
for /f "tokens=2" %%i in ('idf.py --version ^| findstr "ESP-IDF"') do set IDF_VERSION=%%i
echo ✅ ESP-IDF version: %IDF_VERSION%

REM Install dependencies
echo 📦 Installing dependencies...
idf.py reconfigure

echo.
echo 🎉 Setup complete! You can now build the project:
echo.
echo For Mode A (Downlink) - Recommended:
echo   HOST:  idf.py -DCMAKE_CXX_FLAGS="-DZT_IS_HOST=1" build flash monitor
echo   PEER:  idf.py -DCMAKE_CXX_FLAGS="-DZT_IS_HOST=0" build flash monitor
echo.
echo For Mode B (Uplink):
echo   HOST:  idf.py -DCMAKE_CXX_FLAGS="-DHP_TEST_UPLINK=1 -DZT_IS_HOST=1" build flash monitor
echo   PEER:  idf.py -DCMAKE_CXX_FLAGS="-DHP_TEST_UPLINK=1 -DZT_IS_HOST=0" build flash monitor
echo.
echo 📖 See README.md for detailed instructions and troubleshooting.
pause
