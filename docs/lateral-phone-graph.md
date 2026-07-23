# Live Steering Graph on Your Phone — User Guide

The comma four's lateral debug screen shows desired vs actual steering on a display
that's honestly too small for more than a gut feeling. Any phone on the device's hotspot
can show the same signals, bigger, live.

---

## Setup (once)

1. Enable **Web Routes Server** (Settings → BluePilot → Device).
2. Join your phone to the device's WiFi hotspot.
3. Open **`http://192.168.43.1:8088/lateral`** in the phone browser. Bookmark it.

That's the standard comma hotspot address; if you reach the device some other way (LAN,
tethering), use that IP instead, port 8088, path `/lateral`.

## What you get

- A rolling 12-second graph of **desired** (blue) vs **actual** (green) steering angle —
  the same "do the peaks line up" view the on-device debug screen gives you, at ~20 fps
  with about a graph-tick of latency.
- Header stats: speed, lateral engaged, live angle error, and the current low/high
  adjustment factors.
- **Tap anywhere to pause** the trace (tap again to resume). The screen stays awake
  while the page is open, and it reconnects by itself after signal drops — a stale trace
  is never shown as live ("waiting for data…" appears instead).

## The calibration dashboard

When **Auto-Calibrate Factors** is armed, two cards appear under the graph — one per
speed band:

| Card element | Meaning |
|---|---|
| `41 of 10s` / `45s evidence` | Clean-curve evidence collected (seconds) vs what's needed before the calibrator may act. |
| **turns 93% of requested** | What the car is measured doing right now in that band. 100% = perfectly calibrated. |
| `factor 1.00 → try 1.08` | The current factor and the step the calibrator wants to make next. |
| **checking 1.08…** | A step was just applied; the calibrator is collecting fresh curves at the new value to confirm the car responded as predicted. |
| `last step confirmed ✓` | The previous step verified against fresh data. `didn't verify` means the data contradicted it — that factor is held until more evidence arrives. |
| **locked ✓** | Calibration finished for that band; the factor shown is final. |

The pill in the top corner ("42 mph · blend zone") shows which band your current speed
feeds: **low band** under 30 mph, **high band** over 60 mph, and the blend zone between
feeds both proportionally.

The dashboard is **view-only by design** — nothing on the page changes anything on the
device. Adjustments stay on the on-device menus, including the **Erase Calibration
Memory** button (see the [auto-calibration guide](ford-angle-autocal.md)).

## Notes

- Zero cost while nobody watches: the feed starts when the first phone connects and
  stops ~10 s after the last one leaves.
- Works with lateral disengaged too — the traces then just follow your own steering,
  which is correct but worth knowing when reading it.
- Keep your eyes on the road; the page is for a passenger or a parked review.
