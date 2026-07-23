"""Auto-calibration band gauges for the on-device lateral debug graph.

Two phone-battery-style vertical gauges — BLUE = low band (curves under 30 mph),
RED = high band (over 60 mph) — in the free strip between the y-axis scale labels
and the plot. Fill rises bottom-up:
  collecting  -> evidence progress toward acting (dim fill)
  checking    -> fresh-evidence progress of the verify window (bright)
  measuring   -> response as a fraction of 110%, with a thin tick marking exactly
                 100%-of-requested (fill at the tick = calibrated)
  locked      -> full
Hidden entirely while auto-calibration is off.

Cost discipline (the UI process runs ~75% of a core on the MICI): the status string
is parsed only when it CHANGES (~1 Hz publisher cadence), the socket is conflated so
one message is parsed per poll regardless of the 100 Hz publish rate, and the render
is a few rectangles per gauge.

The status socket is created at MODULE IMPORT (UI process boot) — never lazily when
the screen first opens, which would register a new message-queue reader mid-drive
(the /lateral lesson, 2026-07-23: readers register at boot, full stop).
"""
import json

import pyray as rl

from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app, FontWeight

try:
  import cereal.messaging as _messaging
  _STATUS_SOCK = _messaging.sub_sock("controllerStateBP", conflate=True, timeout=0)
except Exception:  # PC/dev hosts without cereal sockets: gauges simply stay hidden
  _messaging = None
  _STATUS_SOCK = None

_BLUE = (77, 163, 255)
_RED = (230, 88, 88)
_BATT_W, _BATT_H = 22, 44
_NUB_W, _NUB_H = 10, 4
_LABEL_GAP = 4
_LABEL_FONT = 12
_ROW_PITCH = _NUB_H + _BATT_H + _LABEL_GAP + _LABEL_FONT + 10
_RESP_SPAN = 1.1  # response fill spans 0..110% of requested; the tick marks 100%


def poll_status() -> str | None:
  """Newest bmsAngleAutoCalState string, or None when nothing new / no socket."""
  if _STATUS_SOCK is None:
    return None
  msg = _messaging.recv_one_or_none(_STATUS_SOCK)
  if msg is None:
    return None
  return str(msg.controllerStateBP.bmsAngleAutoCalState)


class AutoCalBars(Widget):
  """Hidden whenever auto-calibration is off / status is not renderable."""

  WIDTH = _BATT_W + 8
  HEIGHT = 2 * _ROW_PITCH

  def __init__(self):
    super().__init__()
    self._raw = None
    self._st = None   # dict (armed JSON) | "locked" | None

  def update_status(self, status: str | None):
    if status is None or status == self._raw:
      return
    self._raw = status
    if status.startswith("{"):
      try:
        d = json.loads(status)
        self._st = d if ("low" in d and "high" in d) else None
      except ValueError:
        self._st = None
    elif status == "locked":
      self._st = "locked"
    else:
      self._st = None

  @property
  def active(self) -> bool:
    return self._st is not None

  def _gauge(self, band: str):
    """(fill 0..1, tick 0..1 or None, bright) for one band."""
    if self._st == "locked":
      return 1.0, None, True
    b = self._st.get(band, {})
    ph = b.get("ph", "collect")
    if ph == "collect":
      need = max(float(b.get("need", 10)), 1e-6)
      return min(1.0, float(b.get("w", 0.0)) / need), None, False
    if ph == "verify":
      need = max(float(b.get("vneed", 6)), 1e-6)
      return min(1.0, float(b.get("vw", 0.0)) / need), None, True
    r = b.get("r")  # propose / good: response fill with the 100% tick
    fill = (min(_RESP_SPAN, max(0.0, float(r))) / _RESP_SPAN) if r is not None else 0.0
    return fill, 1.0 / _RESP_SPAN, True

  def _render(self, rect: rl.Rectangle):
    if self._st is None:
      return
    font = gui_app.font(FontWeight.NORMAL)
    y = rect.y
    x = int(rect.x)
    for band, rgb, label in (("low", _BLUE, "<30"), ("high", _RED, ">60")):
      try:
        fill, tick, bright = self._gauge(band)
      except (TypeError, ValueError, KeyError):
        continue
      alpha = 235 if bright else 140
      col = rl.Color(rgb[0], rgb[1], rgb[2], alpha)
      # nub (battery cap), body outline, bottom-up fill
      body_y = int(y + _NUB_H)
      rl.draw_rectangle(x + (_BATT_W - _NUB_W) // 2, int(y), _NUB_W, _NUB_H,
                        rl.Color(150, 155, 165, 200))
      body = rl.Rectangle(x, body_y, _BATT_W, _BATT_H)
      rl.draw_rectangle_rounded(body, 0.25, 6, rl.Color(38, 42, 52, 200))
      fh = int((_BATT_H - 4) * fill)
      if fh > 0:
        rl.draw_rectangle(x + 2, body_y + _BATT_H - 2 - fh, _BATT_W - 4, fh, col)
      rl.draw_rectangle_rounded_lines_ex(body, 0.25, 6, 1.5,
                                         rl.Color(150, 155, 165, 200))
      if tick is not None:
        ty = body_y + _BATT_H - 2 - int((_BATT_H - 4) * tick)
        rl.draw_rectangle(x - 3, ty, _BATT_W + 6, 2, rl.Color(235, 235, 240, 220))
      rl.draw_text_ex(font, label,
                      rl.Vector2(x + (_BATT_W - _LABEL_FONT * len(label) * 0.55) / 2,
                                 body_y + _BATT_H + _LABEL_GAP),
                      _LABEL_FONT, 0, rl.Color(160, 165, 175, 210))
      y += _ROW_PITCH
