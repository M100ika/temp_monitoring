@echo off
echo ============================================
echo  TempMonitor — Windows build
echo ============================================

:: Step 1: install / update dependencies
pip install -r requirements.txt
if errorlevel 1 ( echo [FAIL] pip install failed & pause & exit /b 1 )

:: Step 2: generate icon (needs Pillow, already installed above)
if not exist icon.ico (
    echo Generating icon.ico...
    python create_icon.py
    if errorlevel 1 ( echo [FAIL] icon generation failed & pause & exit /b 1 )
)

:: Step 3: PyInstaller
pyinstaller --onefile --windowed ^
            --name TempMonitor ^
            --icon icon.ico ^
            --hidden-import "serial.tools.list_ports" ^
            --hidden-import "serial.tools.list_ports_windows" ^
            --hidden-import "pyqtgraph.graphicsItems.ViewBox.axisCtrlTemplate_pyside6" ^
            --hidden-import "pyqtgraph.graphicsItems.PlotItem.plotConfigTemplate_pyside6" ^
            --hidden-import "pyqtgraph.imageview.ImageViewTemplate_pyside6" ^
            --collect-all pyqtgraph ^
            main.py

echo.
if exist "dist\TempMonitor.exe" (
    echo [OK] dist\TempMonitor.exe ready
) else (
    echo [FAIL] check output above
)
pause
