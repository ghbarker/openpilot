"""Mach-E sim stage-1 acceptance: car-level health, not aux-daemon perfection.

Runs the full stack + MetaDrive with FINGERPRINT=FORD_MUSTANG_MACH_E_MK1 and asserts
what actually matters for judging angle-mode behavior in sim:
  - card fingerprints the Mach-E and stays alive with no blocking car events
  - the driving model and controls run
  - openpilot engages and holds active control
Aux daemons with no bearing on the car (soundd, mapd, dm) are excluded — WSL audio
is not part of the car.

Run fresh (GPU contexts fragment across rapid restarts — `wsl --shutdown` first):
  FINGERPRINT=FORD_MUSTANG_MACH_E_MK1 GPU=1 BLOCK=dmonitoringmodeld,mapd,soundd,ui \
    pytest tools/sim/tests/test_ford_sim_drive.py -q -m slow
"""
import os
import subprocess
import time

import pytest

from multiprocessing import Queue

from cereal import messaging
from openpilot.common.basedir import BASEDIR

SIM_DIR = os.path.join(BASEDIR, "tools/sim")
CAR_PROCS = {"card", "controlsd", "selfdrived", "plannerd", "modeld", "locationd", "paramsd"}


@pytest.mark.slow
def test_mach_e_drives():
  os.environ.setdefault("FINGERPRINT", "FORD_MUSTANG_MACH_E_MK1")
  os.environ.setdefault("BLOCK", "dmonitoringmodeld,mapd,soundd,ui")

  from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

  procs = []
  try:
    p_manager = subprocess.Popen("./launch_openpilot.sh", cwd=SIM_DIR)
    procs.append(p_manager)

    sm = messaging.SubMaster(['selfdriveState', 'onroadEvents', 'managerState', 'carParams', 'controlsState'])
    q = Queue()
    bridge = MetaDriveBridge(False, False, 60, True)
    p_bridge = bridge.run(q, retries=10)
    procs.append(p_bridge)

    start = time.monotonic()
    while not bridge.started.value and time.monotonic() < start + 60:
      time.sleep(0.1)
    assert p_bridge.exitcode is None, f"bridge exited {p_bridge.exitcode}"

    # Phase 1: car-side clean — fingerprint correct, car processes running, no blocking events
    ok_once = False
    detail = "no messages"
    start = time.monotonic()
    while time.monotonic() < start + 90:
      sm.update(100)
      car_events = [e.name for e in sm['onroadEvents'] if e.noEntry or e.softDisable or e.immediateDisable]
      car_procs_down = [p.name for p in sm['managerState'].processes
                        if p.name in CAR_PROCS and p.shouldBeRunning and not p.running]
      fp = sm['carParams'].carFingerprint if sm.seen['carParams'] else ""
      detail = f"events={car_events} car_procs_down={car_procs_down} fp='{fp}'"
      if not car_events and not car_procs_down and fp == "FORD_MUSTANG_MACH_E_MK1":
        ok_once = True
        break
    assert ok_once, f"car side never clean: {detail}"

    # Phase 2: engaged, active control frames accumulate
    active_frames = 0
    start = time.monotonic()
    while time.monotonic() < start + 90 and active_frames < 100:
      sm.update(100)
      if sm.updated['controlsState'] and sm['selfdriveState'].active:
        active_frames += 1
    assert active_frames >= 100, f"only {active_frames} active control frames"
  finally:
    for p in procs:
      p.terminate()
    subprocess.run(["pkill", "-f", "manager.py"], check=False)
    time.sleep(2)
    subprocess.run(["pkill", "-9", "-f", "manager.py"], check=False)
