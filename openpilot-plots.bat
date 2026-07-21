@echo off
REM JotPluggler — openpilot's interactive signal plotter, loaded with a drive.
REM Usage: double-click (defaults to the storm route), or: openpilot-plots.bat <route-id>
REM KEEP THIS WINDOW OPEN. The plot window appears in ~20s (topmost of other windows
REM if needed: Alt+Tab / taskbar "msrdc").
REM Stage new drives: put rlogs at ~/replay_data/<route-id>--<n>/rlog inside WSL.

set ROUTE=%1
if "%ROUTE%"=="" set ROUTE=00000003--b73f9b9ea8

echo Restarting WSL for a clean graphics session...
wsl --shutdown
timeout /t 5 /nobreak >nul

title jotpluggler — %ROUTE%
wsl -d Ubuntu-24.04 -- bash -lc "cd ~/openpilot && source .venv/bin/activate && PYTHONPATH=/home/hunth/openpilot exec tools/jotpluggler/jotpluggler --layout tuning --sync-load --data-dir /home/hunth/replay_data '%ROUTE%'"
pause
