"""Anti-weave smoothing: unit tests for the pure AngleSmoother.

The load-bearing guarantees, in order of importance:
  1. menu 1.0 (or toggle off) is a bit-identical passthrough — the stock contract;
  2. curve-entry behavior is strength-independent and fast;
  3. mid-drive enable/strength changes can never act on stale state;
  4. the wire hold cannot chatter and releases on any real step.
"""
import math
import random

from opendbc.sunnypilot.car.ford.angle_smoothing import (
  AngleSmoother, GAIN_RC_UP, GAIN_RC_DOWN, PRED_RC, ENTER_HYST, BLEND_SLEW,
  WIRE_HOLD, MENU_MIN, MENU_MAX,
  HOLD_ENTRY_RATIO, HOLD_TAU_S, HOLD_KNEE_LO, HOLD_KNEE_HI,
)

DT = 0.05


def _smoother(menu=2.0, enabled=True):
  s = AngleSmoother(dt=DT)
  s.configure(enabled, menu)
  return s


class TestStockContract:
  def test_menu_one_is_bit_identical_passthrough(self):
    s = _smoother(menu=1.0)
    rng = random.Random(7)
    for _ in range(500):
      pred = rng.uniform(-0.01, 0.01)
      b = rng.uniform(0.0, 0.6)
      k = rng.uniform(0.0, 0.02)
      pa = rng.uniform(-0.5, 0.5)
      raw_ent = rng.random() < 0.5
      assert s.prediction(pred) == pred
      assert s.blend(b) == b
      assert s.kappa_schedule(k) == k
      assert s.wire(pa) == pa
      assert s.entering(rng.uniform(-0.001, 0.001), raw_ent) == raw_ent

  def test_toggle_off_is_bit_identical_passthrough(self):
    s = _smoother(menu=2.5, enabled=False)
    for i in range(100):
      v = math.sin(i * 0.3) * 0.01
      assert s.prediction(v) == v
      assert s.kappa_schedule(abs(v)) == abs(v)
      assert s.wire(v) == v

  def test_configure_clamps_menu(self):
    s = AngleSmoother(dt=DT)
    s.configure(True, 0.2)
    assert s.strength == 0.0 and not s.active   # below MENU_MIN -> stock
    s.configure(True, 99.0)
    assert abs(s.strength - (MENU_MAX - 1.0)) < 1e-12


class TestEntryGuarantee:
  def test_gain_schedule_rise_is_fast_at_any_strength(self):
    # Discrete one-pole with RC=GAIN_RC_UP at 20 Hz (a = dt/(rc+dt) = 1/3 per frame):
    # 80% of a step at 0.2 s, 90% at 0.3 s — and the rise must NOT slow down as
    # strength increases (entry behavior is strength-independent by design).
    for menu in (1.5, 2.0, MENU_MAX):
      s = _smoother(menu=menu)
      s.kappa_schedule(0.0)               # seed at zero (straight road)
      out = 0.0
      for i in range(round(0.3 / DT)):
        out = s.kappa_schedule(0.004)
        if i == round(0.2 / DT) - 1:
          assert out >= 0.80 * 0.004, (menu, out)
      assert out >= 0.90 * 0.004, (menu, out)

  def test_release_slows_with_strength(self):
    outs = {}
    for menu in (1.5, 2.5):
      s = _smoother(menu=menu)
      s.kappa_schedule(0.004)             # seed high
      for _ in range(round(0.4 / DT)):
        out = s.kappa_schedule(0.0)
      outs[menu] = out
    assert outs[2.5] > outs[1.5] > 0.0    # stronger damping decays slower


class TestTransitionSafety:
  def test_enable_mid_curve_seeds_at_current_kappa(self):
    # Regression: the schedule used to start from 0, momentarily reading a curve
    # as a straight (gain dip on enable). First active frame must pass through.
    s = _smoother(menu=1.0)               # stock
    for _ in range(50):
      s.kappa_schedule(0.005)             # driving a curve, passthrough
    s.configure(True, 2.0)                # driver steps strength mid-curve
    assert s.kappa_schedule(0.005) == 0.005

  def test_reenable_reseeds_prediction(self):
    # Regression: pred filter kept a stale value across a disable/enable cycle.
    s = _smoother(menu=2.0)
    s.prediction(0.009)                   # seeds at 0.009
    s.configure(True, 1.0)                # stock: passthrough, must clear seeding
    assert s.prediction(0.0) == 0.0
    s.configure(True, 2.0)                # re-enable minutes later
    assert s.prediction(0.002) == 0.002   # seeds fresh at the live value

  def test_reset_reseeds_everything(self):
    s = _smoother(menu=2.0)
    s.kappa_schedule(0.004)
    s.prediction(0.004)
    s.blend(0.6)
    s.wire(0.3)
    s.reset()
    assert s.kappa_schedule(0.001) == 0.001   # re-seeded, not averaged across the gap
    assert s.prediction(0.001) == 0.001
    assert s.blend(0.15) == 0.15
    # wire re-seeds via its normal release (any step from 0 exceeds the band)
    assert s.wire(0.2) == 0.2


class TestElements:
  def test_entering_hysteresis_holds_in_band(self):
    s = _smoother(menu=2.0)
    assert s.entering(2 * ENTER_HYST, False) is True     # crossed up
    assert s.entering(0.0, False) is True                # in band: holds
    assert s.entering(-0.5 * ENTER_HYST, False) is True  # still in band
    assert s.entering(-2 * ENTER_HYST, True) is False    # crossed down

  def test_blend_slew_is_bounded(self):
    s = _smoother(menu=2.0)
    s.blend(0.6)
    out = s.blend(0.15)                    # 4x exit step requested
    assert abs(out - 0.6) <= BLEND_SLEW + 1e-12
    steps = 0
    while abs(out - 0.15) > 1e-9 and steps < 100:
      out = s.blend(0.15)
      steps += 1
    assert steps <= round(0.5 / DT) + 1      # full 0.60 -> 0.15 within ~0.5 s

  def test_prediction_rc_scales_with_strength(self):
    outs = {}
    for menu in (1.5, 2.5):
      s = _smoother(menu=menu)
      s.prediction(0.0)                    # seed at 0
      outs[menu] = s.prediction(0.01)      # one step toward 0.01
    assert outs[1.5] > outs[2.5]           # stronger -> heavier filtering

  def test_wire_hold_no_chatter_and_clean_release(self):
    s = _smoother(menu=2.0)                # band = WIRE_HOLD * 1.0
    band = WIRE_HOLD * s.strength
    s.wire(0.10)                           # establish the held value
    rng = random.Random(3)
    for _ in range(200):                   # dither strictly inside the band
      out = s.wire(0.10 + rng.uniform(-0.9, 0.9) * band)
      assert out == 0.10                   # wire never moves
    assert s.wire(0.10 + 1.5 * band) == 0.10 + 1.5 * band   # real step releases

  def test_wire_tracks_when_stock(self):
    s = _smoother(menu=1.0)
    for i in range(20):
      v = 0.0001 * i
      assert s.wire(v) == v


class TestHoldComp:
  """Hold-time gain compensation: mirrors the PSCM's decaying fresh-response boost so
  one calibrated factor is right at curve entry AND on sustained sweepers. Independent
  of the smoothing strength — its own toggle, exactly 1.0 when off."""

  def test_disabled_is_exactly_one(self):
    s = _smoother(menu=2.0)          # smoothing active — hold comp still off by default
    rng = random.Random(11)
    for _ in range(300):
      assert s.hold_comp(rng.uniform(-0.01, 0.01)) == 1.0

  def test_fresh_curve_starts_at_entry_ratio(self):
    s = _smoother(menu=1.0)          # strength-independent: works at stock smoothing
    s.hold_comp_enabled = True
    assert s.hold_comp(0.0) == 1.0                       # straight
    m = s.hold_comp(0.003)                               # first above-knee frame
    assert abs(m - HOLD_ENTRY_RATIO) < 1e-9

  def test_relaxes_toward_full_gain(self):
    s = _smoother(menu=1.0)
    s.hold_comp_enabled = True
    m = 0.0
    for _ in range(int(3 * HOLD_TAU_S / DT)):            # hold the curve 3 tau
      m = s.hold_comp(0.003)
    assert m > 0.99
    # monotone: never exceeds 1.0
    assert s.hold_comp(0.003) <= 1.0

  def test_straight_and_sign_flip_restart_the_clock(self):
    s = _smoother(menu=1.0)
    s.hold_comp_enabled = True
    for _ in range(int(3 * HOLD_TAU_S / DT)):
      s.hold_comp(0.003)
    s.hold_comp(0.0)                                     # straight resets
    assert abs(s.hold_comp(0.003) - HOLD_ENTRY_RATIO) < 1e-9
    for _ in range(int(3 * HOLD_TAU_S / DT)):
      s.hold_comp(0.003)
    assert abs(s.hold_comp(-0.003) - HOLD_ENTRY_RATIO) < 1e-9  # S-curve flip resets

  def test_sub_knee_fades_to_neutral(self):
    s = _smoother(menu=1.0)
    s.hold_comp_enabled = True
    mid = 0.5 * (HOLD_KNEE_LO + HOLD_KNEE_HI)
    m_mid = s.hold_comp(mid)                             # inside the fade band
    assert HOLD_ENTRY_RATIO < m_mid < 1.0
    assert s.hold_comp(HOLD_KNEE_LO * 0.5) == 1.0        # below the band: untouched

  def test_reset_restarts_the_clock(self):
    s = _smoother(menu=1.0)
    s.hold_comp_enabled = True
    for _ in range(int(3 * HOLD_TAU_S / DT)):
      s.hold_comp(0.003)
    s.reset()                                            # mode-0 discontinuity
    assert abs(s.hold_comp(0.003) - HOLD_ENTRY_RATIO) < 1e-9
