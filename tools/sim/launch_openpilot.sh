#!/usr/bin/env bash

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
# BluePilot: respect a pre-set fingerprint (FINGERPRINT=FORD_MUSTANG_MACH_E_MK1 -> Mach-E sim)
export FINGERPRINT="${FINGERPRINT:-HONDA_CIVIC_2022}"

export BLOCK="${BLOCK},camerad,loggerd,encoderd,micd,logmessaged,manage_athenad,manage_sunnylinkd"
if [[ "$CI" ]]; then
  # TODO: offscreen UI should work
  export BLOCK="${BLOCK},ui"
fi

# BluePilot: MadsUnifiedEngagementMode — with fresh sim params, MADS' default blocks
# unified engagement and silently ERASES pcmEnable/buttonEnable events, so the stack
# can never engage regardless of cruise edges or button presses.
python3 -c "from openpilot.selfdrive.test.helpers import set_params_enabled; from openpilot.common.params import Params; set_params_enabled(); Params().put_bool('MadsUnifiedEngagementMode', True)"

SCRIPT_DIR=$(dirname "$0")
OPENPILOT_DIR=$SCRIPT_DIR/../../

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $OPENPILOT_DIR/system/manager && exec ./manager.py
