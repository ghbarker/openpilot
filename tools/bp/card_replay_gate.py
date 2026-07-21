#!/usr/bin/env python3
"""BluePilot: card process replay gate.

Runs the REAL card process (Ford interface, angle mode, autocal armed) against
locally stored rlog segments of a real drive. Any crash in the car-side code —
import errors, attribute errors, math errors — fails the gate before the code
can reach a device. This is the gate that provably catches the class of bug
that crash-looped card on 2026-07-21 (stale attribute in the autocal glue).

Usage:
  python3 tools/bp/card_replay_gate.py <search_dir> <route_fragment> [seg [seg ...]]
Defaults: two segments spread across the drive.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from angle_autocal_analyze import find_segments  # noqa: E402


def main():
  search_dir = sys.argv[1] if len(sys.argv) > 1 else "/mnt/c/Users/hunth/Downloads"
  route = sys.argv[2] if len(sys.argv) > 2 else "00000006--319e078ab5"
  segs = find_segments(search_dir, route)
  if not segs:
    print(f"GATE FAIL: no rlog segments matching '{route}' under {search_dir}")
    sys.exit(1)

  if len(sys.argv) > 3:
    wanted = [int(s) for s in sys.argv[3:]]
    chosen = [p for p in segs if any(f"--{w}--" in os.path.basename(p) for w in wanted)]
  else:
    chosen = [segs[len(segs) // 4], segs[3 * len(segs) // 4]] if len(segs) > 1 else segs
  # Arm the auto-calibrator during replay so its full pipeline executes, not just imports.
  custom_params = {"FordAngleAutoCal": True, "FordAngleAutoCalState": ""}

  failures = []
  for path in chosen:
    name = os.path.basename(path)
    print(f"== card replay: {name}")
    try:
      lr = LogReader(path)
      out = replay_process_with_name("card", lr, custom_params=custom_params, disable_progress=True)
      n_out = sum(1 for m in out if m.which() in ("carControl", "carOutput", "carState"))
      if n_out < 100:
        failures.append(f"{name}: only {n_out} car messages produced — card did not run properly")
        print(f"   FAIL: {failures[-1]}")
      else:
        print(f"   ok: {n_out} car messages")
    except Exception as exc:
      import traceback
      traceback.print_exc()
      failures.append(f"{name}: {type(exc).__name__}: {exc}")
      print(f"   FAIL: {failures[-1]}")

  print()
  if failures:
    print("GATE FAIL — do NOT install this build:")
    for f in failures:
      print(f"  - {f}")
    sys.exit(1)
  print(f"GATE PASS — card ran clean on {len(chosen)} segment(s) with autocal armed.")


if __name__ == "__main__":
  main()
