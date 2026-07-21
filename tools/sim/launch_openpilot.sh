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

# BluePilot: disable MADS in the simulator — MADS self-engages lateral as soon as
# cruise reports available, and while it is enabled block_unified_engagement_mode()
# ERASES pcmEnable/buttonEnable, so full engagement can never occur. The sim judges
# the lateral controller itself; stock engagement is what it needs.
python3 -c "from openpilot.selfdrive.test.helpers import set_params_enabled; from openpilot.common.params import Params; set_params_enabled(); Params().put_bool('Mads', False)"

SCRIPT_DIR=$(dirname "$0")
OPENPILOT_DIR=$SCRIPT_DIR/../../

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $OPENPILOT_DIR/system/manager && exec ./manager.py
