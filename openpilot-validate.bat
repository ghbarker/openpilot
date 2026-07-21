@echo off
REM Pre-install validation gate. Run BEFORE pushing anything the car will install.
REM Validates the CURRENT branch of this repo (no push needed): schema build, ford
REM test suite, and a real card-process replay over stored Mach-E drive segments
REM with the auto-calibrator armed. Ends with GATE PASS or GATE FAIL.

title validation gate
wsl -d Ubuntu-24.04 -- bash -lc "bash /mnt/c/Users/hunth/Documents/GitHub/bluepilot_7-0/tools/bp/validate_gate.sh"
pause
