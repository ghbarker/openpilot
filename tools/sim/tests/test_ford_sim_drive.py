"""Mach-E sim stage-1 acceptance: car-level health, not aux-daemon perfection.

Runs the full stack + MetaDrive with FINGERPRINT=FORD_MUSTANG_MACH_E_MK1 and asserts
what actually matters for judging angle-mode behavior in sim:
  - card fingerprints the Mach-E and stays alive with no blocking car events
  - the driving model and controls run
  - openpilot engages and holds active control
Aux daemons with no bearing on the car (soundd, mapd, dm, ui) are excluded.

WSL/ROCm reality: OpenCL across processes (VisionIPC camera buffers + the tinygrad
model) is flaky at context creation — modeld sometimes dies with CL_OUT_OF_HOST_MEMORY
at boot regardless of free memory. The whole stack is retried up to MAX_ATTEMPTS on a
modeld boot failure; a real car-side regression still fails every attempt.

Run fresh (`wsl --shutdown` first):
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
MAX_ATTEMPTS = 3


def _cleanup(procs):
  for p in procs:
    try:
      p.terminate()
    except Exception:
      pass
  subprocess.run(["pkill", "-f", "manager.py"], check=False)
  subprocess.run(["pkill", "-f", "selfdrive"], check=False)
  time.sleep(3)
  subprocess.run(["pkill", "-9", "-f", "manager.py"], check=False)
  time.sleep(2)


def _attempt():
  """One full stack boot + drive check. Returns (ok, retryable, detail)."""
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
    if p_bridge.exitcode is not None:
      return False, True, f"bridge exited {p_bridge.exitcode}"

    # Phase 1: car-side clean — correct fingerprint, car processes running, no blocking events
    detail = "no messages"
    modeld_down_samples = 0
    samples = 0
    start = time.monotonic()
    while time.monotonic() < start + 90:
      sm.update(100)
      car_events = [e.name for e in sm['onroadEvents'] if e.noEntry or e.softDisable or e.immediateDisable]
      car_procs_down = [p.name for p in sm['managerState'].processes
                        if p.name in CAR_PROCS and p.shouldBeRunning and not p.running]
      fp = sm['carParams'].carFingerprint if sm.seen['carParams'] else ""
      detail = f"events={car_events} car_procs_down={car_procs_down} fp='{fp}'"
      if not car_events and not car_procs_down and fp == "FORD_MUSTANG_MACH_E_MK1":
        break
      # WSL CL flakiness: modeld crash-LOOPS (manager keeps restarting it), so detect
      # cumulative downtime, not a continuous stretch — then retry the whole stack.
      if sm.seen['managerState']:
        samples += 1
        if "modeld" in car_procs_down:
          modeld_down_samples += 1
        if samples > 200 and modeld_down_samples / samples > 0.3:
          return False, True, f"modeld crash-looping (WSL CL): {detail}"
    else:
      return False, False, f"car side never clean: {detail}"

    # Phase 2: engaged, active control frames accumulate
    active_frames = 0
    start = time.monotonic()
    while time.monotonic() < start + 90 and active_frames < 100:
      sm.update(100)
      if sm.updated['controlsState'] and sm['selfdriveState'].active:
        active_frames += 1
    if active_frames < 100:
      return False, False, f"only {active_frames} active control frames"
    return True, False, f"ok: {active_frames} active frames"
  finally:
    _cleanup(procs)


@pytest.mark.slow
@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
def test_mach_e_drives():
  os.environ.setdefault("FINGERPRINT", "FORD_MUSTANG_MACH_E_MK1")
  os.environ.setdefault("BLOCK", "dmonitoringmodeld,mapd,soundd,ui")

  last_detail = ""
  for attempt in range(1, MAX_ATTEMPTS + 1):
    ok, retryable, detail = _attempt()
    print(f"attempt {attempt}: {detail}", flush=True)
    last_detail = detail
    if ok:
      return
    if not retryable:
      break
  pytest.fail(f"Mach-E sim did not drive after {attempt} attempt(s): {last_detail}")
