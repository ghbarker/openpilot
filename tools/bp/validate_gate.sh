#!/bin/bash
# BluePilot pre-install validation gate (runs in WSL).
#   validate_gate.sh [branch]
# Fetches the branch from the Windows working repo (no push needed), rebuilds the
# schema/params, runs the ford test suite, then replays the REAL card process over
# stored Mach-E drive segments with the auto-calibrator armed. Exits nonzero on any
# failure — nothing should reach installer.comma.ai without this printing GATE PASS.
set -u
WIN_REPO=/mnt/c/Users/hunth/Documents/GitHub/bluepilot_7-0
BRANCH="${1:-$(git -C "$WIN_REPO" rev-parse --abbrev-ref HEAD)}"

cd ~/openpilot
echo "=== [1/4] fetching '$BRANCH' from the Windows repo ==="
git reset --hard -q HEAD  # the WSL clone is a disposable validation environment
git fetch "$WIN_REPO" "$BRANCH" || { echo "GATE FAIL: fetch"; exit 1; }
git checkout -q --detach FETCH_HEAD || { echo "GATE FAIL: checkout"; exit 1; }
git log --oneline -1

source .venv/bin/activate
export PYTHONPATH=/home/hunth/openpilot

echo "=== [2/4] rebuilding schema/params ==="
scons -j16 common/ cereal/ > /tmp/gate_scons.log 2>&1 || { tail -5 /tmp/gate_scons.log; echo "GATE FAIL: build"; exit 1; }

echo "=== [3/4] ford test suite ==="
(cd opendbc_repo && PYTHONPATH=/home/hunth/openpilot:/home/hunth/openpilot/opendbc_repo \
  python3 -m pytest opendbc/sunnypilot/car/ford/tests/ -q 2>&1 | tail -3) || { echo "GATE FAIL: tests"; exit 1; }

echo "=== [4/4] card process replay (real drive, autocal armed) ==="
python3 tools/bp/card_replay_gate.py /mnt/c/Users/hunth/Downloads 00000006--319e078ab5 || { echo "GATE FAIL: card replay"; exit 1; }

echo
echo "GATE PASS — safe to push and install."
