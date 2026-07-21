@echo off
REM One click: download all rlogs for a route from comma, run the angle auto-cal
REM analysis, and open the report.
REM
REM Usage:
REM   openpilot-autocal.bat                          -> default route (00000006--319e078ab5)
REM   openpilot-autocal.bat 00000007--abcdef1234     -> another drive
REM
REM First run only: a comma login happens IN THIS WINDOW — follow the printed URL,
REM sign in like you do on connect.comma.ai, and paste the code back here.

set ROUTE=%1
if "%ROUTE%"=="" set ROUTE=00000006--319e078ab5
set DONGLE=bfef784d32f5351d

title angle auto-cal — %ROUTE%

echo === step 1/3: comma login (skipped if already done) ===
wsl -d Ubuntu-24.04 -- bash -lc "test -s ~/.comma/auth.json || (cd ~/openpilot && source .venv/bin/activate && python3 tools/lib/auth.py)"

echo.
echo === step 2/3: downloading all segments of %DONGLE%/%ROUTE% ===
wsl -d Ubuntu-24.04 -- bash -lc "cd ~/openpilot && source .venv/bin/activate && PYTHONPATH=/home/hunth/openpilot python3 tools/bp/fetch_route_rlogs.py '%DONGLE%/%ROUTE%'"

echo.
echo === step 3/3: running the auto-cal analysis ===
"C:\Users\hunth\AppData\Local\Programs\Python\Python313\python.exe" "%~dp0tools\bp\angle_autocal_analyze.py" "C:/Users/hunth/Downloads" "%ROUTE%"

start "" "%~dp0tools\bp\angle_autocal_report.html"
pause
