"""BluePilot: simulate a Ford Mustang Mach-E (CAN FD) to openpilot in MetaDrive.

Counterpart of SimulatedCar (the Honda Civic fake), with one crucial upgrade: the
steering plant is closed over the REAL wire command. openpilot's emitted
LateralMotionControl2 message is parsed back off sendcan, its path_angle runs
through a PSCM model whose true gain and lag were measured from the owner's
logged drives, and the resulting (lagged) wheel angle both steers MetaDrive and
feeds back into SteeringPinion_Data — so the pinion curvature measurement, the
deviation clip, and the angle auto-calibrator all see road-like physics. A build
with miscalibrated speed factors visibly understeers in the sim exactly as it
did on the road; a calibrated build tracks.

The PSCM truth can be overridden for experiments:
  SIM_TRUE_LOW / SIM_TRUE_HIGH   (default: the factors measured on the real car)
"""
import math
import os
import traceback

import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.ford.values import FordSafetyFlags
from openpilot.common.params import Params
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp, can_capnp_to_list
from openpilot.tools.sim.lib.common import SimulatorState

# Mach-E geometry (matches opendbc values; used to convert curvature <-> wheel angle)
WHEELBASE_M = 2.984
STEER_RATIO = 17.0

# Measured plant truth (angle auto-cal result on the owner's car, 2026-07-21 drive)
TRUE_LOW = float(os.environ.get("SIM_TRUE_LOW", "1.02"))
TRUE_HIGH = float(os.environ.get("SIM_TRUE_HIGH", "1.15"))
V_LOW, V_HIGH = 13.5, 26.82
LOW_ANCHOR_BASE = 1.30
PLATFORM_GAIN_HIGH = 1.05  # CANFD unibody SUV table

PSCM_TAU_S = 0.35  # first-order response lag, from logged step behavior
DT = 0.01          # SimulatedCar runs at 100 Hz


def true_gain(v_ego: float) -> float:
  """The gain the PSCM actually delivers: path_angle = kappa * v * true_gain(v)."""
  if v_ego <= V_LOW:
    a = 0.0
  elif v_ego >= V_HIGH:
    a = 1.0
  else:
    a = (v_ego - V_LOW) / (V_HIGH - V_LOW)
  return (1.0 - a) * (LOW_ANCHOR_BASE * TRUE_LOW) + a * (PLATFORM_GAIN_HIGH * TRUE_HIGH)


class FordSimulatedCar:
  """Publishes Mach-E CAN + pandaStates; models the PSCM steering plant."""

  packer = CANPacker("ford_lincoln_base_pt")

  def __init__(self):
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'carParams', 'selfdriveState'])
    self.can_sock = messaging.sub_sock('sendcan', timeout=0)
    self.cp_cmd = CANParser("ford_lincoln_base_pt", [("LateralMotionControl2", 20)], 0)
    self.params = Params()
    self.idx = 0
    self.obd_multiplexing = False
    # PSCM plant state
    self.lat_mode = 0
    self.path_angle_cmd = 0.0
    self.kappa_plant = 0.0        # current achieved curvature (lagged)
    self.wheel_angle_deg = 0.0    # current wheel angle (drives MetaDrive + pinion sensor)
    # Cruise state machine: Ford is pcmCruise — the CAR owns cruise engagement, so the
    # fake must latch it from the bridge's button presses (Honda CruiseButtons values:
    # 1=MAIN, 2=CANCEL, 3=DECEL_SET, 4=RES_ACCEL) or openpilot waits forever.
    self.cruise_enabled = False
    self._cruise_unengaged_frames = 0
    self._cruise_drop_frames = 0

  # ------------------------------------------------------------------
  # Wire command -> PSCM plant
  # ------------------------------------------------------------------

  def _update_cruise(self, ss: SimulatorState):
    # pcmCruise engagement is edge-triggered, and the first rising edge usually lands
    # inside the boot-time veto storm and is consumed unengaged. A real PCM holds cruise
    # steady; this fake adds patient retry: if openpilot hasn't engaged ~1.5s after
    # cruise latched, drop cruise briefly and let the bridge's SET press produce a fresh
    # rising edge on a clean event board. Stops as soon as engagement sticks.
    if self._cruise_drop_frames > 0:
      self._cruise_drop_frames -= 1
      self.cruise_enabled = False
      return
    if ss.cruise_button in (3, 4):    # SET / RESUME
      self.cruise_enabled = True
    elif ss.cruise_button == 2 or ss.user_brake > 0:  # CANCEL or brake
      self.cruise_enabled = False
    if ss.is_engaged:
      self._cruise_unengaged_frames = 0
    elif self.cruise_enabled:
      self._cruise_unengaged_frames += 1
      if self._cruise_unengaged_frames > 150:   # 1.5 s at 100 Hz
        self._cruise_unengaged_frames = 0
        self._cruise_drop_frames = 30           # 300 ms clean falling edge

  def _update_plant(self, simulator_state: SimulatorState):
    raws = messaging.drain_sock_raw(self.can_sock)
    if raws:
      self.cp_cmd.update(can_capnp_to_list(raws, msgtype='sendcan'))
      vl = self.cp_cmd.vl["LateralMotionControl2"]
      self.lat_mode = int(vl["LatCtl_D2_Rq"])
      self.path_angle_cmd = float(vl["LatCtlPath_An_Actl"])

    v = max(simulator_state.speed, 1.0)
    if self.lat_mode != 0 and simulator_state.is_engaged:
      kappa_target = self.path_angle_cmd / (v * true_gain(v))
    else:
      kappa_target = 0.0  # mode 0 / disengaged: PSCM releases toward center

    alpha = DT / (PSCM_TAU_S + DT)
    self.kappa_plant += alpha * (kappa_target - self.kappa_plant)
    self.wheel_angle_deg = math.degrees(math.atan(self.kappa_plant * WHEELBASE_M)) * STEER_RATIO

  # ------------------------------------------------------------------
  # CAN synthesis (message set mirrors ford/carstate.py get_can_parsers)
  # ------------------------------------------------------------------

  def send_can_messages(self, ss: SimulatorState):
    if not ss.valid:
      return
    msg = []
    speed_kph = ss.speed * 3.6
    engaged = ss.is_engaged

    # *** powertrain bus (0) ***
    msg.append(self.packer.make_can_msg("VehicleOperatingModes", 0, {}))
    msg.append(self.packer.make_can_msg("BrakeSysFeatures", 0, {"Veh_V_ActlBrk": speed_kph}))
    msg.append(self.packer.make_can_msg("BrakeSysFeatures_2", 0, {}))
    msg.append(self.packer.make_can_msg("Yaw_Data_FD1", 0, {
      "VehYaw_W_Actl": self.kappa_plant * ss.speed,  # honest yaw for the sim's actual curvature
    }))
    msg.append(self.packer.make_can_msg("DesiredTorqBrk", 0, {
      "VehStop_D_Stat": 1 if ss.speed < 0.1 else 0,
      "PrkBrkStatus": 0,
    }))
    msg.append(self.packer.make_can_msg("EngVehicleSpThrottle", 0, {
      "ApedPos_Pc_ActlArb": max(0.0, ss.user_gas) * 100,
    }))
    msg.append(self.packer.make_can_msg("EngVehicleSpThrottle2", 0, {
      "Veh_V_ActlEng": speed_kph,
    }))
    msg.append(self.packer.make_can_msg("BrakeSnData_4", 0, {}))
    msg.append(self.packer.make_can_msg("EngBrakeData", 0, {
      "BpedDrvAppl_D_Actl": 2 if ss.user_brake > 0 else 1,
      "CcStat_D_Actl": 5 if self.cruise_enabled else 4,   # 4 = standby, 5 = active (car-owned)
      "Veh_V_DsplyCcSet": 65,                             # cruise setpoint display (kph)
    }))
    msg.append(self.packer.make_can_msg("Cluster_Info1_FD1", 0, {"DrvSlipCtlMde_D_Rq": 0}))
    msg.append(self.packer.make_can_msg("Cluster_Info_3_FD1", 0, {"DISPLAY_SPEED_SCALING": 100}))
    msg.append(self.packer.make_can_msg("EPAS_INFO", 0, {
      "SteeringColumnTorque": ss.user_torque,
      "EPAS_Failure": 0,
    }))
    msg.append(self.packer.make_can_msg("Steering_Data_FD1", 0, {
      "TurnLghtSwtch_D_Stat": 1 if ss.left_blinker else (2 if ss.right_blinker else 0),
      "CcAslButtnCnclPress": 0,
    }))
    msg.append(self.packer.make_can_msg("BodyInfo_3_FD1", 0, {}))  # all doors closed = 0
    msg.append(self.packer.make_can_msg("RCMStatusMessage2_FD1", 0, {
      "FirstRowBuckleDriver": 1,  # buckled
    }))
    msg.append(self.packer.make_can_msg("SteeringPinion_Data", 0, {
      "StePinComp_An_Est": self.wheel_angle_deg,          # the lagged PSCM plant angle
      "StePinCompAnEst_D_Qf": 3,                          # quality good (else vehicleSensorsInvalid)
    }))
    msg.append(self.packer.make_can_msg("Lane_Assist_Data3_FD1", 0, {
      "LatCtlSte_D_Stat": 2 if self.lat_mode != 0 else 1,  # active / ready (not-faulted set)
    }))
    msg.append(self.packer.make_can_msg("Gear_Shift_by_Wire_FD1", 0, {
      "TrnRng_D_RqGsm": 3,  # Drive (4 is Sport — see the DBC VAL_ table)
    }))
    msg.append(self.packer.make_can_msg("PowertrainData_10", 0, {}))  # required for automatics

    # *** camera bus (2): stock IPMA chatter the cam parser expects ***
    # LateralMotionControl2 at its 8-byte DBC size is also the interface's "not a
    # TRON/SecOC platform" proof (16 bytes there -> dashcamOnly).
    msg.append(self.packer.make_can_msg("LateralMotionControl2", 2, {}))
    msg.append(self.packer.make_can_msg("ACCDATA", 2, {}))
    msg.append(self.packer.make_can_msg("ACCDATA_2", 2, {}))
    msg.append(self.packer.make_can_msg("ACCDATA_3", 2, {}))
    msg.append(self.packer.make_can_msg("IPMA_Data", 2, {}))
    msg.append(self.packer.make_can_msg("IPMA_Data2", 2, {"IsaVLimUnit_D_Rq": 0}))
    msg.append(self.packer.make_can_msg("Traffic_RecognitnData", 2, {}))
    # Mach-E has BSM: the cam parser requires the side radars (5 Hz on the real car)
    msg.append(self.packer.make_can_msg("Side_Detect_L_Stat", 2, {}))
    msg.append(self.packer.make_can_msg("Side_Detect_R_Stat", 2, {}))
    # CANFD pseudo-radar: radard parses Steer_Assist_Data off the camera bus; without it
    # radarState never validates and longitudinalPlan/commIssue block engagement forever.
    # Empty payload = radar sees no lead, which is a perfectly valid radar state.
    msg.append(self.packer.make_can_msg("Steer_Assist_Data", 2, {}))

    self.pm.send('can', can_list_to_can_capnp(msg))

  def send_panda_state(self, ss: SimulatorState):
    self.sm.update(0)
    if self.params.get_bool("ObdMultiplexingEnabled") != self.obd_multiplexing:
      self.obd_multiplexing = not self.obd_multiplexing
      self.params.put_bool("ObdMultiplexingChanged", True, block=True)
    # Mirror the safety config carParams expects — selfdrived raises controlsMismatch
    # forever on any difference (model or param), which blocks engagement.
    safety_model, safety_param = 'ford', FordSafetyFlags.CANFD.value
    cp = self.sm["carParams"]
    if len(cp.safetyConfigs):
      safety_model = str(cp.safetyConfigs[-1].safetyModel)
      safety_param = cp.safetyConfigs[-1].safetyParam
    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': ss.ignition,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'controlsAllowedLateral': True,
      'controlsAllowedLongitudinal': True,
      'safetyModel': safety_model,
      'alternativeExperience': cp.alternativeExperience,
      'safetyParam': safety_param,
    }
    self.pm.send('pandaStates', dat)

  def update(self, simulator_state: SimulatorState):
    try:
      # Before the MetaDrive world publishes its first state, velocity is None and
      # simulator_state.speed raises — everything below must wait for valid.
      if simulator_state.valid:
        self._update_cruise(simulator_state)
        self._update_plant(simulator_state)
        self.send_can_messages(simulator_state)
      if self.idx % 50 == 0:
        self.send_panda_state(simulator_state)
      self.idx += 1
    except Exception:
      traceback.print_exc()
      raise
