@echo off
REM openpilot SIMULATOR (MetaDrive) — watch openpilot drive with the current code.
REM
REM KEEP THIS WINDOW OPEN. Two windows will appear (give it 1-2 minutes on first
REM run while the driving model warms up):
REM   - the openpilot UI (road view, path, HUD)
REM   - the MetaDrive world window
REM
REM Controls (press keys in THIS console window):
REM   2 = cruise set / engage      1 = speed up     3 = cancel
REM   s = simulated driver brake (disengage)        r = reset sim
REM   i = toggle ignition          q = quit everything
REM
REM Notes:
REM  - This restarts WSL first (fixes the invisible-window graphics bug), which
REM    closes any running JotPluggler/replay windows.
REM  - Default car: YOUR MACH-E — synthesized Ford CAN-FD, the real Ford interface
REM    and angle-mode code, steering through a PSCM model measured from your logs.
REM    Miscalibrated speed factors visibly understeer here like on the road.
REM       openpilot-sim.bat            -> Mach-E
REM       openpilot-sim.bat civic      -> the old generic Honda sim car
REM  - If windows appear but stay hidden behind other apps: Alt+Tab, or look for
REM    "msrdc" windows in the taskbar.

set SIMCAR=FORD_MUSTANG_MACH_E_MK1
if /I "%1"=="civic" set SIMCAR=HONDA_CIVIC_2022

echo Restarting WSL for a clean graphics session...
wsl --shutdown
timeout /t 5 /nobreak >nul

title openpilot simulator [%SIMCAR%] — press 2 to engage, q to quit
REM GPU=1 -> driving model runs on the RX 7900 XTX via ROCm OpenCL (first launch
REM recompiles kernels for the GPU once, ~2-3 min; cached and fast afterwards)
REM BLOCK dmonitoringmodeld: no driver to monitor in the sim, and its GPU context
REM competes with the driving model for ROCm host memory (CL_OUT_OF_HOST_MEMORY)
wsl -d Ubuntu-24.04 -- bash -lc "cd ~/openpilot && source .venv/bin/activate && export GPU=1 && export BLOCK=dmonitoringmodeld,mapd && export FINGERPRINT=%SIMCAR% && (tools/sim/launch_openpilot.sh > /tmp/op_sim.log 2>&1 &) ; sleep 12 ; exec tools/sim/run_bridge.py"
pause
