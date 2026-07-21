@echo off
REM Preview BluePilot theme packs on the PC — UI appears as a desktop window over a
REM replayed drive, in either device layout.
REM
REM Usage:
REM   openpilot-theme-preview.bat                     -> halloween_week (most distinct palette), comma 4 layout
REM   openpilot-theme-preview.bat christmas_week      -> pick a pack (folder name in selfdrive/assets/bp_themes, "off", or "rad_racer")
REM   openpilot-theme-preview.bat christmas_week c3x  -> comma 3X layout instead of comma 4
REM   openpilot-theme-preview.bat off c3x 00000001--f5fbd93372   -> also pick the replay route
REM
REM "rad_racer" previews the built-in 8-Bit Racer theme (same selector on the device).
REM The preview always enables Minimal Driving View: the staged replays are rlog-only
REM (no camera video), so the road is drawn on a clean background instead of black.
REM Segments must be staged in WSL at ~/replay_data/<route-id>--<n>/rlog (same as openpilot-replay.bat).
REM KEEP THIS WINDOW OPEN — it is the replay control console (space=pause, s=+10s, q=quit).

set PACK=%1
if "%PACK%"=="" set PACK=halloween_week

set DEVICE=%2
if "%DEVICE%"=="" set DEVICE=c4
set BIGUI=0
if /I "%DEVICE%"=="c3x" set BIGUI=1

set ROUTE=%3
if "%ROUTE%"=="" set ROUTE=00000003--b73f9b9ea8

echo Restarting WSL for a clean graphics session...
wsl --shutdown
timeout /t 5 /nobreak >nul

title theme preview [%PACK%] %DEVICE% — %ROUTE%
wsl -d Ubuntu-24.04 -- bash -lc "cd ~/openpilot && source .venv/bin/activate && export PYTHONPATH=/home/hunth/openpilot && python3 -m openpilot.selfdrive.ui.bp.lib.theme_pack '%PACK%' minimal && (BIG=%BIGUI% python3 selfdrive/ui/ui.py >/tmp/ui.log 2>&1 &) ; sleep 3 ; exec tools/replay/replay --data_dir=/home/hunth/replay_data '%ROUTE%'"
pause
