@echo off
REM One-click start: Isaac Sim (with Claude control server) + scene + Go2 with keyboard control.
REM Double-click in Explorer. First start takes a few minutes; the scene opens by itself when the UI is ready.
REM Extra name=value parameters are passed to start_go2.py, e.g.:
REM   start_go2.bat policy=robot_lab camera=False
call "D:\isaacsim\isaac-sim.claude.bat" --exec "%~dp0tools\isaac\run_in_kit.py %~dp0tools\isaac\start_go2.py %*"
