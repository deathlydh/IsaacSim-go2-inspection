@echo off
setlocal
if exist "%~dp0go2.local.bat" call "%~dp0go2.local.bat"
if not defined GO2_ISAAC_DIR set "GO2_ISAAC_DIR=D:\isaacsim"
if not exist "%GO2_ISAAC_DIR%\isaac-sim.bat" (
    echo Isaac Sim not found. Set GO2_ISAAC_DIR to its installation folder.
    pause
    exit /b 1
)
cd /d "%~dp0"
if not "%CD: =%"=="%CD%" (
    echo Move this checkout to a path without spaces for Kit --exec.
    pause
    exit /b 1
)
set "GO2_PROJECT_ROOT=%CD%"
call "%GO2_ISAAC_DIR%\isaac-sim.bat" --enable isaacsim.code_editor.python_server --exec "%~dp0tools\isaac\run_in_kit.py %~dp0tools\isaac\start_go2.py scene_path=%~dp0scenes\factory_hall.usda policy=robot_lab camera=True camera_window=True speed=0.5"
