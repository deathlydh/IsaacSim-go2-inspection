@echo off
REM One-click start: Isaac Sim (with Claude control server) + inspection scene + Go2 with keyboard control.
REM Double-click in Explorer. First start takes a few minutes; the scene opens by itself when the UI is ready.
call "D:\isaacsim\isaac-sim.claude.bat" --exec "%~dp0tools\isaac\run_in_kit.py %~dp0tools\isaac\start_go2.py"
