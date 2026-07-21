@echo off
REM openpilot drive replay ("unlogger") — runs in WSL, UI appears as a desktop window.
REM Usage: double-click (defaults to the storm route), or: openpilot-replay.bat <route-id>
REM
REM KEEP THIS WINDOW OPEN while watching — closing it stops the replay.
REM This window is the control console:
REM   space = pause/resume    s = +10s    shift+s = -10s    m = +60s
REM   e = next engagement     d = next disengagement    +/- = speed    q = quit
REM
REM Segments must be staged in WSL at ~/replay_data/<route-id>--<n>/rlog
REM (camera view is black unless "Road camera" files are added alongside the rlogs)

set ROUTE=%1
if "%ROUTE%"=="" set ROUTE=00000003--b73f9b9ea8

echo Restarting WSL for a clean graphics session...
wsl --shutdown
timeout /t 5 /nobreak >nul

title openpilot replay controls — %ROUTE%
wsl -d Ubuntu-24.04 -- bash -lc "cd ~/openpilot && source .venv/bin/activate && PYTHONPATH=/home/hunth/openpilot python3 selfdrive/ui/ui.py >/tmp/ui.log 2>&1 & sleep 3; exec tools/replay/replay --data_dir=/home/hunth/replay_data '%ROUTE%'"
pause
