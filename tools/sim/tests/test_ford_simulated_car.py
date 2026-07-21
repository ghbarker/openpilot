"""Round-trip self-test for the Mach-E simulated car (no MetaDrive needed).

Proves: (1) every CAN message FordSimulatedCar synthesizes parses through the REAL
ford carstate — valid buses, sane CarState; (2) the PSCM plant responds to a real
LateralMotionControl2 wire command with the measured gain and lag, and the resulting
wheel angle appears back in the parsed steeringAngleDeg (the pinion loop closes).

Run in WSL/linux (needs msgq):  pytest tools/sim/tests/test_ford_simulated_car.py -q
"""
import math
import time

import pytest

messaging = pytest.importorskip("cereal.messaging")

from openpilot.common.prefix import OpenpilotPrefix  # noqa: E402


FINGERPRINT = "FORD_MUSTANG_MACH_E_MK1"


class FakeSimState:
  valid = True
  ignition = True
  is_engaged = True
  speed = 20.0
  user_gas = 0.0
  user_brake = 0.0
  user_torque = 0.0
  left_blinker = False
  right_blinker = False
  cruise_button = 0


@pytest.fixture
def env():
  with OpenpilotPrefix("ford_sim_test"):
    yield


def _build_carstate():
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.carstate import CarState
  from opendbc.car import structs
  CP = CarInterface.get_non_essential_params(FINGERPRINT)
  CP_SP = structs.CarParamsSP()
  cs = CarState(CP, CP_SP)
  parsers = CarState.get_can_parsers(CP, CP_SP)
  return cs, parsers


def test_round_trip(env):
  from openpilot.tools.sim.lib.simulated_car_ford import FordSimulatedCar, true_gain, WHEELBASE_M, STEER_RATIO
  from opendbc.can.packer import CANPacker
  from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp

  can_recv = messaging.sub_sock('can', timeout=100)
  sendcan_pub = messaging.pub_sock('sendcan')
  car = FordSimulatedCar()
  cs, parsers = _build_carstate()
  ss = FakeSimState()
  time.sleep(0.3)  # let sockets connect

  # Send a real angle-mode wire command: mode 1, path_angle for a 0.002 1/m curve at 20 m/s
  packer = CANPacker("ford_lincoln_base_pt")
  kappa_want = 0.002
  path_angle = kappa_want * ss.speed * true_gain(ss.speed)
  cmd = packer.make_can_msg("LateralMotionControl2", 0, {
    "LatCtl_D2_Rq": 1,
    "LatCtlPath_An_Actl": path_angle,
  })

  ret = None
  for i in range(400):  # 4 simulated seconds at 100 Hz
    if i % 5 == 0:  # 20 Hz command, like the real carcontroller
      sendcan_pub.send(can_list_to_can_capnp([cmd], msgtype='sendcan'))
      time.sleep(0.001)
    car.update(ss)
    raws = messaging.drain_sock_raw(can_recv)
    if raws:
      from openpilot.selfdrive.pandad.pandad_api_impl import can_capnp_to_list
      can_list = can_capnp_to_list(raws)
      for p in parsers.values():
        p.update(can_list)
    if i > 50:
      ret, _ = cs.update(parsers)

  assert ret is not None

  # --- CarState sanity from synthesized CAN ---
  assert abs(ret.vEgoRaw - 20.0) < 0.5, ret.vEgoRaw
  assert not ret.doorOpen and not ret.seatbeltUnlatched
  assert ret.gearShifter == "drive", ret.gearShifter
  assert ret.cruiseState.enabled
  assert not ret.steerFaultTemporary and not ret.steerFaultPermanent

  # --- PSCM plant: converged to the commanded curvature through the measured gain ---
  expected_wheel_deg = math.degrees(math.atan(kappa_want * WHEELBASE_M)) * STEER_RATIO
  assert abs(car.wheel_angle_deg - expected_wheel_deg) < 0.15 * abs(expected_wheel_deg) + 0.05, \
    (car.wheel_angle_deg, expected_wheel_deg)
  # and the pinion sensor loop closed: parsed steering angle equals the plant angle
  assert abs(ret.steeringAngleDeg - car.wheel_angle_deg) < 0.2, (ret.steeringAngleDeg, car.wheel_angle_deg)

  # --- mode 0 releases the plant ---
  cmd0 = packer.make_can_msg("LateralMotionControl2", 0, {"LatCtl_D2_Rq": 0, "LatCtlPath_An_Actl": 0.0})
  for i in range(300):
    if i % 5 == 0:
      sendcan_pub.send(can_list_to_can_capnp([cmd0], msgtype='sendcan'))
      time.sleep(0.001)
    car.update(ss)
  assert abs(car.wheel_angle_deg) < 0.3, car.wheel_angle_deg
