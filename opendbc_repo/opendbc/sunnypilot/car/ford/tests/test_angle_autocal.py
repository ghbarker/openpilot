"""Tests for the one-time angle-mode factor auto-calibration (angle_autocal.py)."""
import random

from opendbc.sunnypilot.car.ford.angle_autocal import (
  AngleFactorEstimator, SteadyStateGate, speed_alpha,
  V_LOW, V_HIGH, LOW_ANCHOR_BASE, STEADY_TIME_S, MIN_KAPPA,
)

PLATFORM_GAIN_HIGH = 1.05  # Mach-E


def simulate_plant(est, true_low_factor, true_high_factor, speeds, kappa=0.002, n_per_speed=200, noise=0.0):
  """Feed samples from a plant whose true gain corresponds to the given ideal factors.

  The plant's response ratio r = applied_gain / ideal_gain: if the applied factors already
  matched the true ones, r would be 1 everywhere.
  """
  rng = random.Random(42)
  for v in speeds:
    a = speed_alpha(v)
    ideal = (1.0 - a) * (LOW_ANCHOR_BASE * true_low_factor) + a * (PLATFORM_GAIN_HIGH * true_high_factor)
    for _ in range(n_per_speed):
      r = est.applied_gain(v) / ideal
      r *= 1.0 + (rng.uniform(-noise, noise) if noise else 0.0)
      est.add_sample(v, kappa, kappa * r, weight=0.05)


class TestAngleFactorEstimator:
  def test_recovers_true_factors(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    simulate_plant(est, true_low_factor=0.92, true_high_factor=1.21,
                   speeds=[10, 12, 15, 18, 21, 24, 27, 29], n_per_speed=200, noise=0.03)
    low, high, stats = est.solve()
    assert abs(low - 0.92) < 0.02, low
    assert abs(high - 1.21) < 0.02, high
    assert est.converged()

  def test_accounts_for_applied_factors(self):
    # A drive made with non-default factors must still recover the same truth.
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.10, 0.90)
    simulate_plant(est, true_low_factor=0.92, true_high_factor=1.21,
                   speeds=[10, 15, 20, 25, 29], n_per_speed=300, noise=0.03)
    low, high, _ = est.solve()
    assert abs(low - 0.92) < 0.02, low
    assert abs(high - 1.21) < 0.02, high

  def test_not_converged_with_one_sided_speeds(self):
    # Only low-speed driving: high anchor must not report convergence.
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    simulate_plant(est, 0.95, 1.10, speeds=[10, 11, 12], n_per_speed=400, noise=0.02)
    assert not est.converged()

  def test_rejects_bad_samples(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    assert not est.add_sample(20.0, 0.0005, 0.0005)   # below curvature threshold
    assert not est.add_sample(5.0, 0.002, 0.002)      # below speed threshold
    assert not est.add_sample(20.0, 0.002, -0.002)    # sign mismatch
    assert not est.add_sample(20.0, 0.002, 0.02)      # absurd ratio
    assert est.n == 0

  def test_factor_clamp(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    simulate_plant(est, 2.5, 0.2, speeds=[10, 20, 29], n_per_speed=100)
    low, high, _ = est.solve()
    assert low == 1.5 and high == 0.5  # clamped to the +/- button range


class TestSteadyStateGate:
  def test_requires_sustained_steady(self):
    gate = SteadyStateGate(dt=0.05)
    needed = int(STEADY_TIME_S / 0.05)
    results = [gate.update(True, MIN_KAPPA * 2, False, False, False, False, False)
               for _ in range(needed + 2)]
    assert not any(results[:needed - 1])
    assert results[-1]

  def test_resets_on_any_flag(self):
    gate = SteadyStateGate(dt=0.05)
    for _ in range(int(STEADY_TIME_S / 0.05) + 1):
      gate.update(True, MIN_KAPPA * 2, False, False, False, False, False)
    assert gate.update(True, MIN_KAPPA * 2, False, False, False, False, False)
    gate.update(True, MIN_KAPPA * 2, True, False, False, False, False)  # pressed
    assert gate.steady_s == 0.0

  def test_resets_on_curvature_jump(self):
    gate = SteadyStateGate(dt=0.05)
    for _ in range(int(STEADY_TIME_S / 0.05) + 1):
      gate.update(True, 0.002, False, False, False, False, False)
    assert gate.update(True, 0.002, False, False, False, False, False)
    assert not gate.update(True, 0.004, False, False, False, False, False)  # jump
    assert gate.steady_s == 0.0
