#!/usr/bin/env python3
"""BluePilot: offline analysis environment for the angle-mode factor auto-calibration.

Replays logged drives through the exact estimator the car will run
(opendbc/sunnypilot/car/ford/angle_autocal.py): reconstructs the pinion-derived
measured curvature, gates steady engaged curves with the logged strategy flags,
and solves for the FordLowSpeedFactor_ang / FordHighSpeedFactor_ang values that
would make the actual turn match the requested turn. Writes an HTML report with
every accepted sample so the recommendation can be inspected before anything is
changed on the car.

Usage (Windows or WSL, from the repo root):
  python tools/bp/angle_autocal_analyze.py <route-search-dir> <route-id-fragment>
  python tools/bp/angle_autocal_analyze.py C:/Users/hunth/Downloads 00000006--319e078ab5
"""
import bz2
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import numpy as np

from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.ford.angle_autocal import (
  AutoCalPipeline, platform_gains, V_LOW, V_HIGH,
  CONVERGE_MIN_WEIGHT, CONVERGE_MAX_STDERR,
)


def find_segments(search_dir: str, route_fragment: str) -> list[str]:
  """Locate rlog files for the route: bare .zst files and folder-style downloads."""
  out = {}
  pat = re.compile(re.escape(route_fragment) + r"--(\d+)--rlog")
  for root, _dirs, files in os.walk(search_dir):
    for f in files:
      m = pat.search(f)
      if m and "(" not in f and (f.endswith("rlog.zst") or f.endswith("rlog.bz2") or f.endswith("--rlog")):
        seg = int(m.group(1))
        if seg not in out:
          out[seg] = os.path.join(root, f)
  return [out[k] for k in sorted(out)]


def load_events(path: str):
  import zstandard as zstd
  from cereal import log as capnp_log
  with open(path, "rb") as f:
    dat = f.read()
  if dat.startswith(b"\x28\xB5\x2F\xFD"):
    dat = zstd.ZstdDecompressor().stream_reader(dat).read()
  elif dat.startswith(b"BZh9"):
    dat = bz2.decompress(dat)
  return capnp_log.Event.read_multiple_bytes(dat)


class DriveExtractor:
  """Streams a drive's events in time order, emitting one row per controllerStateBP frame
  (the 20 Hz lateral cadence) with the latest state of everything the estimator needs."""

  def __init__(self):
    self.VM = None
    self.fingerprint = ""
    self.lp_offset = 0.0
    self.lp_roll = 0.0
    self.v_ego = 0.0
    self.steering_deg = 0.0
    self.steering_pressed = False
    self.steering_torque = 0.0
    self.lat_active = False
    self.kappa_desired = 0.0
    self.flags = dict(angle_rate=False, deviation=False, human_turn=False, stall=False)
    self.mode_angle = False
    self.bp_lat_disabled = True
    self.low_factor = 1.0
    self.high_factor = 1.0
    self.rows = []

  def kappa_measured(self) -> float:
    if self.VM is None:
      return 0.0
    return -self.VM.calc_curvature(math.radians(self.steering_deg - self.lp_offset),
                                   max(self.v_ego, 0.1), self.lp_roll)

  def feed(self, events):
    for e in events:
      w = e.which()
      if w == "carParams":
        cp = e.carParams
        self.fingerprint = cp.carFingerprint
        try:
          self.VM = VehicleModel(cp)
        except Exception:
          self.VM = None
      elif w == "liveParameters":
        lp = e.liveParameters
        self.lp_offset = lp.angleOffsetDeg
        self.lp_roll = lp.roll
        if self.VM is not None and lp.steerRatio > 0.1:
          self.VM.update_params(max(lp.stiffnessFactor, 0.1), lp.steerRatio)
      elif w == "carState":
        cs = e.carState
        self.v_ego = cs.vEgoRaw
        self.steering_deg = cs.steeringAngleDeg
        self.steering_pressed = cs.steeringPressed
        self.steering_torque = cs.steeringTorque
      elif w == "carControl":
        cc = e.carControl
        self.lat_active = cc.latActive
        self.kappa_desired = cc.actuators.curvature
      elif w == "controllerStateBP":
        st = e.controllerStateBP
        self.flags = dict(
          angle_rate=st.angleRateLimited,
          deviation=st.curvatureDeviationLimited,
          human_turn=st.humanTurnLateralPaused,
          stall=st.stallBlipActive,
          saturated=st.angleSaturated,  # absent in pre-field logs -> capnp default False
        )
        self.mode_angle = int(st.bmsPrimaryControlVariable) == 1
        self.bp_lat_disabled = st.bmsDisableBpLateralControl
        if st.bmsLowSpeedAdjustmentFactor > 0:
          self.low_factor = st.bmsLowSpeedAdjustmentFactor
        if st.bmsHighSpeedAdjustmentFactor > 0:
          self.high_factor = st.bmsHighSpeedAdjustmentFactor
        self.rows.append(dict(
          t=e.logMonoTime * 1e-9,
          v=self.v_ego,
          kappa_cmd=self.kappa_desired,
          kappa_meas=self.kappa_measured(),
          lat_active=self.lat_active and self.mode_angle and not self.bp_lat_disabled,
          pressed=self.steering_pressed,
          torque=self.steering_torque,
          **self.flags,
        ))


def main():
  search_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/Downloads")
  route = sys.argv[2] if len(sys.argv) > 2 else "00000006--319e078ab5"

  segs = find_segments(search_dir, route)
  if not segs:
    print(f"no rlog segments matching '{route}' under {search_dir}")
    sys.exit(1)
  print(f"found {len(segs)} segment(s)")

  ex = DriveExtractor()
  for path in segs:
    print(f"  loading {os.path.basename(path)}")
    ex.feed(load_events(path))
  print(f"fingerprint: {ex.fingerprint}   frames: {len(ex.rows)}")
  print(f"factors active during drive: low={ex.low_factor:.2f} high={ex.high_factor:.2f}")

  g_low, g_high = platform_gains(ex.fingerprint)
  pipe = AutoCalPipeline(g_high, ex.low_factor, ex.high_factor)
  est = pipe.est
  accepted = []
  angle_frames = 0
  for row in ex.rows:
    if not row["lat_active"]:
      pipe.idle()
      continue
    angle_frames += 1
    for v, kc, km in pipe.update(row["v"], row["kappa_cmd"], row["kappa_meas"],
                                 row["pressed"], row["angle_rate"], row["deviation"],
                                 row["human_turn"], row["stall"],
                                 saturated=row["saturated"], driver_torque=row["torque"]):
      accepted.append(dict(v=v, kappa_cmd=kc, kappa_meas=km))

  print(f"angle-mode active frames: {angle_frames} ({angle_frames/20:.0f}s)   accepted samples: {len(accepted)} ({len(accepted)/20:.0f}s)")
  if angle_frames == 0:
    print("\nNO ANGLE-MODE DRIVING in this data. Set Primary Control Variable = Angle and drive with lateral engaged.")
    sys.exit(2)

  result = est.solve()
  if result is None:
    print("\nnot enough steady-curve samples to fit — need engaged, hands-off, steady curves.")
    sys.exit(3)
  low_new, high_new, stats = result
  conv = est.converged()

  print("\n=== recommendation ===")
  print(f"FordLowSpeedFactor_ang:  {ex.low_factor:.2f} -> {low_new:.2f}   "
        f"(evidence {stats['weight_low']:.0f}s / need {CONVERGE_MIN_WEIGHT:.0f}s, stderr {stats['stderr_low']:.3f} / max {CONVERGE_MAX_STDERR})")
  print(f"FordHighSpeedFactor_ang: {ex.high_factor:.2f} -> {high_new:.2f}   "
        f"(evidence {stats['weight_high']:.0f}s / need {CONVERGE_MIN_WEIGHT:.0f}s, stderr {stats['stderr_high']:.3f} / max {CONVERGE_MAX_STDERR})")
  print(f"converged (would lock onboard): {conv}")

  write_report(ex, est, accepted, (low_new, high_new, stats), conv,
               os.path.join(os.path.dirname(os.path.abspath(__file__)), "angle_autocal_report.html"))


def write_report(ex, est, accepted, result, converged, out_path):
  low_new, high_new, stats = result
  pts = []
  for row in accepted:
    r = row["kappa_meas"] / row["kappa_cmd"]
    pts.append((row["v"], r))

  # SVG scatter: ratio vs speed, with anchor lines
  w, h = 900, 420
  x0, x1 = 5.0, 35.0
  y0, y1 = 0.5, 1.5
  def sx(v):
    return 60 + (v - x0) / (x1 - x0) * (w - 90)
  def sy(r):
    return 20 + (y1 - r) / (y1 - y0) * (h - 70)
  dots = "".join(
    f'<circle cx="{sx(v):.1f}" cy="{sy(min(max(r, y0), y1)):.1f}" r="3" fill="#e4781c" fill-opacity="0.55"/>'
    for v, r in pts)
  anchors = (f'<line x1="{sx(V_LOW)}" y1="20" x2="{sx(V_LOW)}" y2="{h-50}" stroke="#888" stroke-dasharray="4"/>'
             f'<line x1="{sx(V_HIGH)}" y1="20" x2="{sx(V_HIGH)}" y2="{h-50}" stroke="#888" stroke-dasharray="4"/>'
             f'<line x1="60" y1="{sy(1.0)}" x2="{w-30}" y2="{sy(1.0)}" stroke="#4a4" stroke-width="1.5"/>')
  labels = (f'<text x="{sx(V_LOW)}" y="{h-34}" fill="#aaa" font-size="12" text-anchor="middle">{V_LOW*2.237:.0f} mph anchor</text>'
            f'<text x="{sx(V_HIGH)}" y="{h-34}" fill="#aaa" font-size="12" text-anchor="middle">{V_HIGH*2.237:.0f} mph anchor</text>'
            f'<text x="50" y="{sy(1.0)+4}" fill="#4a4" font-size="12" text-anchor="end">1.0</text>'
            f'<text x="50" y="{sy(1.25)+4}" fill="#888" font-size="12" text-anchor="end">1.25</text>'
            f'<text x="50" y="{sy(0.75)+4}" fill="#888" font-size="12" text-anchor="end">0.75</text>')

  html = f"""<meta charset="utf-8"><title>Angle auto-cal — {ex.fingerprint}</title>
<body style="background:#14161a;color:#ddd;font-family:system-ui;max-width:960px;margin:24px auto;padding:0 16px">
<h2>Angle-mode factor auto-calibration</h2>
<p>{ex.fingerprint} — {len(accepted)} accepted steady-curve samples ({len(accepted)/20:.0f}s)</p>
<table style="border-collapse:collapse;margin:12px 0">
<tr><th style="text-align:left;padding:4px 16px 4px 0">Factor</th><th style="padding:4px 16px">during drive</th><th style="padding:4px 16px">recommended</th><th style="padding:4px 16px">evidence</th><th style="padding:4px 16px">stderr</th></tr>
<tr><td style="padding:4px 16px 4px 0">Low speed (&le;{V_LOW*2.237:.0f} mph)</td><td align="center">{ex.low_factor:.2f}</td><td align="center"><b>{low_new:.2f}</b></td><td align="center">{stats['weight_low']:.0f}s / {CONVERGE_MIN_WEIGHT:.0f}s</td><td align="center">{stats['stderr_low']:.3f}</td></tr>
<tr><td style="padding:4px 16px 4px 0">High speed (&ge;{V_HIGH*2.237:.0f} mph)</td><td align="center">{ex.high_factor:.2f}</td><td align="center"><b>{high_new:.2f}</b></td><td align="center">{stats['weight_high']:.0f}s / {CONVERGE_MIN_WEIGHT:.0f}s</td><td align="center">{stats['stderr_high']:.3f}</td></tr>
</table>
<p><b>{"CONVERGED — the onboard calibrator would lock these values." if converged else "Not yet converged — more steady engaged curves needed (see evidence column)."}</b></p>
<h3>Actual / requested turn ratio vs speed</h3>
<p style="color:#999">Each dot is a steady engaged curve moment. Below the green line = car turns less than requested (factor should rise). Above = turns more (factor should drop).</p>
<svg width="{w}" height="{h}" style="background:#1b1e24;border-radius:8px">{anchors}{dots}{labels}
<text x="{w/2}" y="{h-8}" fill="#aaa" font-size="13" text-anchor="middle">speed (m/s)</text></svg>
</body>"""
  with open(out_path, "w", encoding="utf-8") as f:
    f.write(html)
  print(f"report: {out_path}")


if __name__ == "__main__":
  main()
