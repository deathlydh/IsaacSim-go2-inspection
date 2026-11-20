@echo off
setlocal
cd /d "%~dp0"
if exist "%~dp0go2.local.bat" call "%~dp0go2.local.bat"
if not defined GO2_ISAAC_DIR set "GO2_ISAAC_DIR=D:\isaacsim"
if not defined GO2_WSL_DISTRO set "GO2_WSL_DISTRO=Ubuntu-22.04"
if not defined GO2_WINDOWS_PYTHON_EXE if exist "%~dp0.venv\Scripts\python.exe" set "GO2_WINDOWS_PYTHON_EXE=%~dp0.venv\Scripts\python.exe"
if not defined GO2_WINDOWS_PYTHON_EXE set "GO2_WINDOWS_PYTHON_EXE=python"
if "%~1"=="" (
    echo [1] Nav2: drive to Gauge_01, capture and try reading
    echo [2] CV only: try reading the included image
    echo [3] Known route: visit the five gauges without ROS
    echo [4] Nav2: test cancellation
    echo [5] Nav2: test a static obstacle
    echo [6] LiDAR: build and save a new map
    echo [7] RViz: view the live LiDAR scan
    choice /c 1234567 /n /m "Select: "
    if errorlevel 255 exit /b 1
    if errorlevel 7 goto rviz
    if errorlevel 6 goto mapping
    if errorlevel 5 goto obstacle
    if errorlevel 4 goto cancel
    if errorlevel 3 goto route
    if errorlevel 2 goto cv
    goto nav
)
if /i "%~1"=="nav" goto nav
if /i "%~1"=="cv" goto cv
if /i "%~1"=="route" goto route
if /i "%~1"=="cancel" goto cancel
if /i "%~1"=="obstacle" goto obstacle
if /i "%~1"=="mapping" goto mapping
if /i "%~1"=="rviz" goto rviz
echo Usage: run_demo.bat [nav^|cv^|route^|cancel^|obstacle^|mapping^|rviz]
exit /b 2

:cv
"%GO2_WINDOWS_PYTHON_EXE%" -m cv.read_image shots/camera_Gauge_01.png
goto done

:route
"%GO2_WINDOWS_PYTHON_EXE%" tools/run_route_cli.py
goto done

:nav
set "GO2_DEMO_MODE=nav"
goto live
:cancel
set "GO2_DEMO_MODE=cancel"
goto live
:obstacle
set "GO2_DEMO_MODE=obstacle"
goto live
:mapping
set "GO2_DEMO_MODE=mapping"
goto live
:rviz
set "GO2_DEMO_MODE=rviz"
:live
"%GO2_WINDOWS_PYTHON_EXE%" tools/run_demo.py %GO2_DEMO_MODE%

:done
set "GO2_EXIT=%ERRORLEVEL%"
if not "%GO2_EXIT%"=="0" echo Failed with exit code %GO2_EXIT%. See the output above.
if "%~1"=="" pause
exit /b %GO2_EXIT%
