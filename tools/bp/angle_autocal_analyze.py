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
  AutoCalPipeline, platform_gains, speed_alpha, V_LOW, V_HIGH, LOW_ANCHOR_BASE,
  CONVERGE_MIN_WEIGHT, CONVERGE_MAX_STDERR, VERIFY_TOL,
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
  print(f"converged: {conv}")
  delta = max(abs(low_new - ex.low_factor), abs(high_new - ex.high_factor))
  if conv and delta <= VERIFY_TOL:
    print("recommendation is a NO-CHANGE within tolerance — calibration is VERIFIED and would lock onboard.")
  elif conv:
    print(f"next step: apply these factors (settings +/- or the onboard toggle), drive again, re-run this "
          f"tool — when the new recommendation moves less than {VERIFY_TOL:.2f}, it's verified and locks.")

  here = os.path.dirname(os.path.abspath(__file__))
  write_report(ex, est, accepted, (low_new, high_new, stats), conv, g_high,
               os.path.join(here, "angle_autocal_report.html"))
  write_replay(ex, est, low_new, high_new, g_high,
               os.path.join(here, "angle_autocal_replay_template.html"),
               os.path.join(here, "angle_autocal_replay.html"))


def write_replay(ex, est, low_new, high_new, g_high, template_path, out_path):
  """Dump the drive as a JS frame stream for the animated replay viewer.

  Per angle-active frame: [t_rel, v, kappa_cmd, kappa_meas, kappa_sim_after, clean]
  where kappa_sim_after applies the recommended factors' gain shift (first-order)
  and clean marks frames free of grip/limit contamination.
  """
  import json
  def gain_new(v):
    a = speed_alpha(v)
    return (1.0 - a) * (LOW_ANCHOR_BASE * low_new) + a * (g_high * high_new)

  frames = []
  t_acc, last_t = 0.0, None
  for row in ex.rows:
    if not row["lat_active"]:
      last_t = None
      continue
    if last_t is not None:
      t_acc += min(row["t"] - last_t, 0.5)  # compress gaps
    last_t = row["t"]
    shift = gain_new(row["v"]) / est.applied_gain(row["v"])
    clean = not (row["pressed"] or row["human_turn"] or row["stall"]
                 or row["saturated"] or row["angle_rate"] or row["deviation"])
    frames.append([round(t_acc, 2), round(row["v"], 1),
                   round(row["kappa_cmd"], 5), round(row["kappa_meas"], 5),
                   round(row["kappa_meas"] * shift, 5), 1 if clean else 0])
  payload = dict(
    fingerprint=ex.fingerprint,
    low_old=ex.low_factor, high_old=ex.high_factor,
    low_new=round(low_new, 2), high_new=round(high_new, 2),
    frames=frames,
  )
  # Inline the frame stream into the template -> a single self-contained HTML file
  # that opens from anywhere (Explorer double-click, the bat, the browser pane).
  with open(template_path, encoding="utf-8") as f:
    html = f.read()
  data_tag = "<script>const DRIVE = " + json.dumps(payload, separators=(",", ":")) + ";</script>"
  html = html.replace('<script src="angle_autocal_frames.js"></script>', data_tag)
  with open(out_path, "w", encoding="utf-8") as f:
    f.write(html)
  print(f"replay viewer: {out_path} ({len(frames)} frames, {os.path.getsize(out_path)//1024//1024} MB, self-contained)")


def write_report(ex, est, accepted, result, converged, g_high, out_path):
  low_new, high_new, stats = result

  def gain_new(v):
    a = speed_alpha(v)
    return (1.0 - a) * (LOW_ANCHOR_BASE * low_new) + a * (g_high * high_new)

  def gain_shift(v):
    """Multiplier the new factors apply to the achieved curvature at speed v."""
    return gain_new(v) / est.applied_gain(v)

  # --- Scatter: ratio vs speed, before (orange) and predicted after (green) ---
  w, h = 900, 420
  x0, x1 = 5.0, 35.0
  y0, y1 = 0.5, 1.5
  def sx(v):
    return 60 + (v - x0) / (x1 - x0) * (w - 90)
  def sy(r):
    return 20 + (y1 - min(max(r, y0), y1)) / (y1 - y0) * (h - 70)
  before_dots, after_dots = [], []
  err_before = dict(low=[], high=[])
  err_after = dict(low=[], high=[])
  for row in accepted:
    v = row["v"]
    r = row["kappa_meas"] / row["kappa_cmd"]
    r_after = r * gain_shift(v)
    before_dots.append(f'<circle cx="{sx(v):.1f}" cy="{sy(r):.1f}" r="3" fill="#e4781c" fill-opacity="0.5"/>')
    after_dots.append(f'<circle cx="{sx(v):.1f}" cy="{sy(r_after):.1f}" r="3" fill="#3fbf6f" fill-opacity="0.5"/>')
    band = "low" if speed_alpha(v) < 0.5 else "high"
    err_before[band].append(abs(r - 1.0))
    err_after[band].append(abs(r_after - 1.0))

  def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")

  anchors = (f'<line x1="{sx(V_LOW)}" y1="20" x2="{sx(V_LOW)}" y2="{h-50}" stroke="#888" stroke-dasharray="4"/>'
             f'<line x1="{sx(V_HIGH)}" y1="20" x2="{sx(V_HIGH)}" y2="{h-50}" stroke="#888" stroke-dasharray="4"/>'
             f'<line x1="60" y1="{sy(1.0)}" x2="{w-30}" y2="{sy(1.0)}" stroke="#4a4" stroke-width="1.5"/>')
  labels = (f'<text x="{sx(V_LOW)}" y="{h-34}" fill="#aaa" font-size="12" text-anchor="middle">{V_LOW*2.237:.0f} mph anchor</text>'
            f'<text x="{sx(V_HIGH)}" y="{h-34}" fill="#aaa" font-size="12" text-anchor="middle">{V_HIGH*2.237:.0f} mph anchor</text>'
            f'<text x="50" y="{sy(1.0)+4}" fill="#4a4" font-size="12" text-anchor="end">1.0</text>')

  # --- Drive timeline: requested vs actual vs predicted-after, angle-active frames only ---
  active = [row for row in ex.rows if row["lat_active"]]
  tw, th = 900, 300
  strip = ""
  if active:
    t0 = active[0]["t"]
    # Collapse replay gaps (segment boundaries / disengaged stretches) into a compact timeline
    xs, last_t, xacc = [], None, 0.0
    for row in active:
      if last_t is not None:
        xacc += min(row["t"] - last_t, 0.5)
      last_t = row["t"]
      xs.append(xacc)
    span = max(xs[-1], 1.0)
    k_lim = 0.008
    step = max(1, len(active) // 2400)
    def tx(x):
      return 50 + x / span * (tw - 70)
    def ty(k):
      return 10 + (k_lim - min(max(k, -k_lim), k_lim)) / (2 * k_lim) * (th - 40)
    def polylines(key, color, scale_fn=None):
      out, seg_pts, last_x = [], [], None
      for x, row in zip(xs[::step], active[::step]):
        if last_x is not None and x - last_x > 3.0:
          if len(seg_pts) > 1:
            out.append(f'<polyline points="{" ".join(seg_pts)}" fill="none" stroke="{color}" stroke-width="1.2" stroke-opacity="0.85"/>')
          seg_pts = []
        val = row[key] * (scale_fn(row["v"]) if scale_fn else 1.0)
        seg_pts.append(f"{tx(x):.1f},{ty(val):.1f}")
        last_x = x
      if len(seg_pts) > 1:
        out.append(f'<polyline points="{" ".join(seg_pts)}" fill="none" stroke="{color}" stroke-width="1.2" stroke-opacity="0.85"/>')
      return "".join(out)
    strip = (f'<line x1="50" y1="{ty(0)}" x2="{tw-20}" y2="{ty(0)}" stroke="#444"/>'
             + polylines("kappa_cmd", "#cccccc")
             + polylines("kappa_meas", "#e4781c")
             + polylines("kappa_meas", "#3fbf6f", scale_fn=gain_shift))

  html = f"""<meta charset="utf-8"><title>Angle auto-cal — {ex.fingerprint}</title>
<body style="background:#14161a;color:#ddd;font-family:system-ui;max-width:960px;margin:24px auto;padding:0 16px">
<h2>Angle-mode factor auto-calibration</h2>
<p>{ex.fingerprint} — {len(accepted)} accepted steady-curve samples ({len(accepted)/20:.0f}s)</p>
<table style="border-collapse:collapse;margin:12px 0">
<tr><th style="text-align:left;padding:4px 16px 4px 0">Factor</th><th style="padding:4px 16px">during drive</th><th style="padding:4px 16px">recommended</th><th style="padding:4px 16px">evidence</th><th style="padding:4px 16px">stderr</th></tr>
<tr><td style="padding:4px 16px 4px 0">Low speed (&le;{V_LOW*2.237:.0f} mph)</td><td align="center">{ex.low_factor:.2f}</td><td align="center"><b>{low_new:.2f}</b></td><td align="center">{stats['weight_low']:.0f}s / {CONVERGE_MIN_WEIGHT:.0f}s</td><td align="center">{stats['stderr_low']:.3f}</td></tr>
<tr><td style="padding:4px 16px 4px 0">High speed (&ge;{V_HIGH*2.237:.0f} mph)</td><td align="center">{ex.high_factor:.2f}</td><td align="center"><b>{high_new:.2f}</b></td><td align="center">{stats['weight_high']:.0f}s / {CONVERGE_MIN_WEIGHT:.0f}s</td><td align="center">{stats['stderr_high']:.3f}</td></tr>
</table>
<p><b>{"CONVERGED." if converged else "Not yet converged — more steady engaged curves needed (see evidence column)."}</b>
This tool and the onboard calibrator verify in rounds: apply the recommendation, drive, re-run —
when a round recommends a change smaller than {VERIFY_TOL:.2f}, the calibration is verified and locks.</p>

<h3>Drive timeline: requested vs actual turn</h3>
<p style="color:#999"><span style="color:#ccc">requested</span> ·
<span style="color:#e4781c">actual (this drive)</span> ·
<span style="color:#3fbf6f">predicted actual with recommended factors</span> — gaps compressed, engaged angle-mode frames only.</p>
<svg width="{tw}" height="{th}" style="background:#1b1e24;border-radius:8px">{strip}</svg>

<h3>Actual / requested ratio vs speed — before and after</h3>
<p style="color:#999">Green line = perfect tracking. <span style="color:#e4781c">Orange: this drive.</span>
<span style="color:#3fbf6f">Green: predicted with the recommended factors.</span></p>
<svg width="{w}" height="{h}" style="background:#1b1e24;border-radius:8px">{anchors}{"".join(before_dots)}{"".join(after_dots)}{labels}
<text x="{w/2}" y="{h-8}" fill="#aaa" font-size="13" text-anchor="middle">speed (m/s)</text></svg>

<table style="border-collapse:collapse;margin:14px 0">
<tr><th style="text-align:left;padding:4px 16px 4px 0">Mean tracking error |ratio − 1|</th><th style="padding:4px 16px">this drive</th><th style="padding:4px 16px">predicted after</th></tr>
<tr><td style="padding:4px 16px 4px 0">low-speed band</td><td align="center">{mean(err_before['low'])*100:.1f}%</td><td align="center"><b>{mean(err_after['low'])*100:.1f}%</b></td></tr>
<tr><td style="padding:4px 16px 4px 0">high-speed band</td><td align="center">{mean(err_before['high'])*100:.1f}%</td><td align="center"><b>{mean(err_after['high'])*100:.1f}%</b></td></tr>
</table>
<p style="color:#777;font-size:13px">The "after" traces are first-order predictions (achieved turn scales with the gain change);
the verification round on the real car is what proves them.</p>
</body>"""
  with open(out_path, "w", encoding="utf-8") as f:
    f.write(html)
  print(f"report: {out_path}")


if __name__ == "__main__":
  main()
