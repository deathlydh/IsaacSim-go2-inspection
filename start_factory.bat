@echo off
REM One-click start: factory hall with gauges + Go2 (robot_lab trot policy) + front camera window.
REM Double-click in Explorer. To change something, edit the name=value list below:
REM   scene_path  - scene file (no spaces in the path)
REM   policy      - robot_lab | nvidia_rough | bundled
REM   camera      - True | False ;  camera_window - True | False
REM   speed       - m/s per arrow key (max 1.0)
call "D:\isaacsim\isaac-sim.claude.bat" --exec "%~dp0tools\isaac\run_in_kit.py %~dp0tools\isaac\start_go2.py scene_path=D:/go2-inspection/scenes/factory_hall.usda policy=robot_lab camera=True camera_window=True speed=0.5"
