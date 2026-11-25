"""
BluePilot Ford lateral: the one object the Ford CarController calls.

Holds the two strategies, LateralCurvExt and LateralAngleExt, with the inputs and state they
share, and does the strategy dispatch and message assembly. Lives above both strategy modules
(rather than inside either) so it can call into both without the circular import that would
result from one depending on the other.

The CarController owns apply_curvature_last, which the stock and BluePilot paths share: it is
passed into step_lateral and the result returned, so switching BluePilot lateral off mid-drive
continues from the last command, as before.
"""

import math
from types import SimpleNamespace

try:
  from ...car import scale_rot_inertia_per_mass, scale_tire_stiffness_per_mass
except ImportError:
  # a host on opendbc from before the per-mass vehicle model (IQ.Pilot): its mass-scaled helpers at unit
  # mass are the per-mass values
  from ...car import scale_rot_inertia, scale_tire_stiffness

  def scale_rot_inertia_per_mass(wheelbase):
    return scale_rot_inertia(1.0, wheelbase)

  def scale_tire_stiffness_per_mass(wheelbase, center_to_front, tire_stiffness_factor):
    return scale_tire_stiffness(1.0, wheelbase, center_to_front, tire_stiffness_factor)
from ...car import ACCELERATION_DUE_TO_GRAVITY
from ...car.ford.values import CarControllerParams, FordFlags
from ...car.vehicle_model import VehicleModel, calc_slip_factor
from ..can import fordcan_ext
from .lateral_angle_ext import LateralAngleExt
from .lateral_curv_ext import LateralCurvExt, PrimaryLateralControl
from ..params.manifest import value
from ..params.param_store import ParamStore
from .values_ext import (
  FORD_PINION_FITTED_GEOMETRY, FORD_PINION_WHEELBASE, FordSafetyFlagsSP, PINION_SHADOW_BAND, PINION_SHADOW_BAND_MARGIN,
)
from .human_turn import HumanTurnDetector
from ..host_types import LateralInputs, LateralStatus
from .lateral_inputs import ModelView, VehicleParamsView, lateral_inputs_complete


def pinion_vehicle_model(CP):
  """Vehicle model for the pinion-angle curvature measurement: CarSpecs, except where BluePilot's
  pinion geometry uses another wheelbase (FORD_PINION_WHEELBASE, the same value as the C table),
  keeping the platform's weight split and tire model."""
  wheelbase = FORD_PINION_WHEELBASE.get(CP.carFingerprint)
  if wheelbase is None:
    return VehicleModel(CP)
  center_to_front = wheelbase * CP.centerToFront / CP.wheelbase
  front, rear = scale_tire_stiffness_per_mass(wheelbase, center_to_front, CP.tireStiffnessFactor)
  inertia = scale_rot_inertia_per_mass(wheelbase)
  # Both VehicleModel signatures: upstream's per-mass fields, and the mass-scaled ones an older host's
  # model reads, at unit mass (mass cancels out of the model, so the two are the same model).
  return VehicleModel(SimpleNamespace(
    rotationalInertiaPerMass=inertia, wheelbase=wheelbase,
    centerToFront=center_to_front, steerRatio=CP.steerRatio,
    tireStiffnessFrontPerMass=front, tireStiffnessRearPerMass=rear,
    mass=1.0, rotationalInertia=inertia, tireStiffnessFront=front, tireStiffnessRear=rear, steerRatioRear=0.))


def panda_pinion_geometry(CP):
  """(slip factor, steer ratio, wheelbase) of this platform's row in the panda's ford_pinion_geometry table
  (bluepilot_lateral/safety/ford.h): the fitted row where there is one, else CarSpecs through the same vehicle model the
  table was generated from. safety/tests/test_ford_bluepilot_panda_band.py checks it against the C table for every
  platform."""
  fitted = FORD_PINION_FITTED_GEOMETRY.get(CP.carFingerprint)
  if fitted is not None:
    return fitted
  VM = pinion_vehicle_model(CP)
  return calc_slip_factor(VM), CP.steerRatio, VM.l


def fitted_pinion_curvature(geometry, steering_angle_rad: float, v: float, roll: float) -> float:
  """VehicleModel.calc_curvature with a fitted row's fixed slip factor, steer ratio and wheelbase in place of the
  learned vehicle model: curvature from the steering angle (already less its offset), plus roll compensation. Same
  sign as calc_curvature."""
  slip_factor, steer_ratio, wheelbase = geometry
  v2 = v * v
  roll_compensation = 0.0 if abs(slip_factor) < 1e-6 else ACCELERATION_DUE_TO_GRAVITY * roll / ((1.0 / slip_factor) - v2)
  return steering_angle_rad / (steer_ratio * wheelbase * (1.0 - slip_factor * v2)) + roll_compensation


class InputTracker:
  """The model / vehicle-parameter inputs both strategies read, and the car's measured curvature.
  Owned by BluePilotLateral and updated once per frame from the host's LateralInputs, before either
  strategy runs."""

  def __init__(self, CP, safety_param: int):
    # Model / vehicle-parameter inputs arrive per frame from the host (see update)
    self.VM = pinion_vehicle_model(CP)  # used only for the pinion-angle curvature measurement
    self.model = None
    self.lp = None
    self.lateral_delay = 0.0

    # BluePilot: steering-angle curvature measurement (bad-yaw-sensor workaround).
    # Mirrors the STEER_ANGLE_CURVATURE flag the safety firmware reads from
    # current_safety_param_sp -- both layers must always agree, so this is init-time
    # state from the startup safety bits (values_ext.ford_safety_param), never a live settings read:
    # a live flip against stale firmware would fight the panda.
    self.bp_pinion_curvature_enabled = bool(safety_param & FordSafetyFlagsSP.STEER_ANGLE_CURVATURE)
    # A platform whose pinion geometry is fitted to drive data measures with that row here too (get_current_curvature).
    self.fitted_geometry = FORD_PINION_FITTED_GEOMETRY.get(CP.carFingerprint)

    # The panda's own geometry row, for panda_band: only with the pinion source on. With the yaw source the
    # panda measures exactly as get_current_curvature does, and there is nothing to reconcile.
    self.panda_geometry = panda_pinion_geometry(CP) if self.bp_pinion_curvature_enabled else None
    # telemetry for the last lateral step (LateralAngleExt clears both at the start of each step)
    self.bp_panda_band_limited = False   # clip_command_to_panda_band moved the command
    self.bp_panda_band_disjoint = False  # the deviation window and the panda's band did not overlap

  def get_current_curvature(self, CS):
    """Measured curvature of the car right now (OP sign convention).

    The single measurement source for every BluePilot lateral consumer: the deviation
    clip, the stall detector, and the shadow curvature published to ford.h's angle-mode
    deviation check. The default source is the RCM yaw rate -- the same family stock
    ford.h derives its angle_meas from. The shadow value judged against that check must
    always come from the same measurement as the check's own reference, so route all
    reads through here.

    With the steering-angle curvature measurement enabled (FordPrefSteerAngleCurvature
    toggle -> FordSafetyFlagsSP.STEER_ANGLE_CURVATURE), the pinion angle via the vehicle
    model is used instead: some vehicles (e.g. a 2021 Explorer with a faulty RCM)
    broadcast implausible VehYaw_W_Actl (sign-inverted vs IMU and steering geometry)
    while its CAN quality flag still reads OK. The pinion angle (SteeringPinion_Data,
    PSCM) is an equivalent measurement, independently validated against the comma IMU
    (corr +0.99), and the panda safety angle_meas switches to the same source (see
    bluepilot_lateral/safety/ford.h) -- the layers always agree. angleOffsetDeg/roll come from
    liveParameters (paramsd, IMU-derived, not the car yaw sensor).

    A platform with a fitted pinion row (FORD_PINION_FITTED_GEOMETRY) uses that row's geometry here as well, with the
    same angle offset and roll, instead of the learned steer ratio and stiffness. On the Expedition the learned ratio
    follows the kind of driving (18.7 in town, 20.3 after two hours of highway), because the vehicle model's speed
    term does not fit the truck; at 20.3 this measurement read 11 to 14% under the truck's curvature in a low-speed
    turn, and up to 142 units away from the panda's. With the fitted row it is within 71 units of the panda's on the
    same drives, the difference being the offset and roll the panda does not apply.
    """
    if self.bp_pinion_curvature_enabled:
      angle_offset_deg = self.lp.angleOffsetDeg if self.lp is not None else 0.0
      roll = self.lp.roll if self.lp is not None else 0.0
      steering_angle_rad = math.radians(CS.out.steeringAngleDeg - angle_offset_deg)
      if self.fitted_geometry is not None:
        return -fitted_pinion_curvature(self.fitted_geometry, steering_angle_rad, CS.out.vEgoRaw, roll)
      return -self.VM.calc_curvature(steering_angle_rad, CS.out.vEgoRaw, roll)
    return -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

  def get_panda_curvature(self, CS):
    """The curvature the panda measures right now (OP sign), by its own arithmetic: the raw pinion angle through its
    fixed geometry row, with no angle offset, roll or learned parameter (ford_bp_rx_hook). None with the yaw source,
    where the panda's measurement is get_current_curvature's."""
    if self.panda_geometry is None:
      return None
    slip_factor, steer_ratio, wheelbase = self.panda_geometry
    v = max(CS.out.vEgoRaw, 0.1)
    return -math.radians(CS.out.steeringAngleDeg) / (steer_ratio * wheelbase * (1.0 - slip_factor * v * v))

  def panda_band(self, CS):
    """(low, high), OP sign: the range the panda will accept as shadow curvature right now, less the margin
    (values_ext.PINION_SHADOW_BAND / _MARGIN). None with the yaw source."""
    center = self.get_panda_curvature(CS)
    if center is None:
      return None
    half = PINION_SHADOW_BAND - PINION_SHADOW_BAND_MARGIN
    return center - half, center + half

  def clip_command_to_panda_band(self, kappa: float, low: float, high: float, CS) -> float:
    """kappa, already inside the deviation window [low, high], moved into the part of that window the panda will also
    accept. It never leaves [low, high]: the panda's band only narrows the window, it does not replace it. Where the two
    do not overlap no command satisfies both; kappa is then returned as it came, the panda is left to judge the frame as
    it would without this, and bp_panda_band_disjoint is set. With the yaw source kappa is returned untouched.

    Against the alternative this is never further from the request: a dropped frame leaves the PSCM on the previous
    frame's command for another 50 ms."""
    self.bp_panda_band_limited = False
    self.bp_panda_band_disjoint = False
    band = self.panda_band(CS)
    if band is None:
      return kappa
    lo, hi = max(low, band[0]), min(high, band[1])
    if lo > hi:
      self.bp_panda_band_disjoint = True
      return kappa
    out = min(max(kappa, lo), hi)
    self.bp_panda_band_limited = out != kappa
    return out

  def get_shadow_of_measurement(self, CS) -> float:
    """What angle mode publishes as shadow curvature while nothing is steered from it (inactive, driver override,
    stall blip, pressed): the measured curvature, kept inside the panda's band so the value the panda has latched at
    the next enabled frame is one it accepts. get_current_curvature itself with the yaw source.

    The same limit as the command: where the deviation window around the measurement and the panda's band do not
    overlap, the measurement is published as it is. The two sides then disagree by more than this is meant to
    reconcile, and a pressed frame must not pass the panda's check on a value moved to meet it."""
    measured = self.get_current_curvature(CS)
    band = self.panda_band(CS)
    if band is None:
      return measured
    if measured - CarControllerParams.CURVATURE_ERROR > band[1] or measured + CarControllerParams.CURVATURE_ERROR < band[0]:
      return measured
    return min(max(measured, band[0]), band[1])

  def update(self, li: LateralInputs):
    """Take this frame's model / vehicle-parameter inputs from the host and update the vehicle model.

    A frame is used only if it has the full modelV2 shape this code indexes into (33 path points,
    both inner lane lines, 3+ probs/stds); anything else is ignored and the previous inputs are
    kept, exactly as a stale SubMaster message was. That also makes fuzzed or partial inputs
    safe: the producer's valid flag is necessary but not trusted on its own."""
    if li.valid and lateral_inputs_complete(li):
      self.model = ModelView(li)
      self.lp = VehicleParamsView(li)
      self.lateral_delay = li.lateralDelay

    if self.lp is not None:
      x = max(self.lp.stiffnessFactor, 0.1)
      sr = max(self.lp.steerRatio, 0.1)
      self.VM.update_params(x, sr)


class SharedLateralState:
  """The values both strategies read and write, so either mode sees what the other left, as when
  the strategies were mixins on one object. bp_curvature_deviation_limited is the BluePilot UI's
  deviation flag, from whichever mode ran."""

  def __init__(self):
    self.path_angle_last = 0.0
    self.precision_type = 1  # 1=Precise, 0=Comfortable
    self.bp_curvature_deviation_limited = False
    self.human_turn_detector = HumanTurnDetector()


class BluePilotLateral:
  def __init__(self, CP, safety_param: int, packer, CAN):
    """safety_param: the startup safety bits the panda was given (values_ext.ford_safety_param)."""
    self.CP = CP
    self.frame = 0
    self.inputs = InputTracker(CP, safety_param)
    self.shared = SharedLateralState()
    self.curv = LateralCurvExt(self.inputs, self.shared)
    self.angle = LateralAngleExt(CP, self.inputs, self.shared)
    self.packer = packer
    self.CAN = CAN

    self.disable_BP_lat_UI = False
    self.lateral_uncertainty = 0.0  # from whichever strategy ran last, for status()
    # the panda only accepts 4-signal messages when this bit was set at init
    self.bp_lateral_allowed = bool(safety_param & FordSafetyFlagsSP.BP_LATERAL)

  def update(self, inputs: LateralInputs, params: ParamStore, frame):
    """Every frame: take this frame's model / vehicle-parameter inputs and settings, both from the host
    (HOSTS.md). Returns whether BluePilot lateral is active."""
    self.frame = frame
    self.inputs.update(inputs)
    self.curv.update_lateral_params(params)
    self.angle.update_angle_params(params)
    # One flag for every BP consumer (LatCtl dispatch, LKA angle-mode bits, mode reporting): the UI
    # toggle, and the panda bit that was set at init. No 4-signal traffic to a stock-configured panda.
    self.disable_BP_lat_UI = value(params, "disable_BP_lat_UI") or not self.bp_lateral_allowed
    return not self.disable_BP_lat_UI

  def step_lateral(self, CC, CS, actuators, apply_curvature_last):
    """At STEER_STEP: the steer message(s). Returns (can_sends, apply_curvature): the CarController
    keeps it and passes it back next time as apply_curvature_last."""
    # BluePilot: select the BP lateral strategy by primary control variable.
    #   angle     -> LateralAngleExt: kappa -> path_angle (c1), apply_curvature held at 0.
    #   curvature -> LateralCurvExt: full 4-signal curvature-primary (default).
    # Both return a LateralResult packed identically below.
    # Do not run the stock curvature limits here or overwrite apply_curvature_last before the
    # strategy runs. Panda rate-checks desired_curvature vs the last TX on the bus; that must match
    # the prior frame's lat.apply_curvature only (not an intermediate stock-limited value).
    angle_mode = self.curv.primary_lateral_control == PrimaryLateralControl.angle
    if angle_mode:
      lat = self.angle.update_angle_strategy(CC, CS, actuators)
    else:
      lat = self.curv.update(CC, CS, actuators, apply_curvature_last, self.CP)
    apply_curvature = float(lat.apply_curvature)
    self.lateral_uncertainty = float(lat.lateralUncertainty)

    # BluePilot: angle-mode human-turn override -- send lateral inactive (mode 0) while the
    # driver manually turns, so the PSCM releases cleanly instead of stalling 2-3 s on
    # re-engage (observed on Mach-E). Panda-clean: every ford.h check has a legitimate
    # !steer_control_enabled branch for the zeroed frames; on release, path_angle ramps back
    # from 0 through the soft ROC. Curvature mode keeps its own reset_steering path (zeroed
    # signals with mode still active) in LateralCurvExt.
    # The stall blip (lateral_angle_ext.py) rides the same mode-0 path: a short pulse that
    # resets the PSCM's post-override attenuation when the deviation clip deadlocks hands-free.
    lat_active = CC.latActive and not (angle_mode and (self.angle.angle_human_turn_active or self.angle.angle_stall_blip_active))

    can_sends = []
    if self.CP.flags & FordFlags.CANFD:
      mode = 1 if lat_active else 0
      counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
      can_sends.append(fordcan_ext.create_lat_ctl2_msg(
        self.packer, self.CAN, mode, lat.ramp_type, lat.precision_type,
        -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate, counter
      ))
    else:
      can_sends.append(fordcan_ext.create_lat_ctl_msg(
        self.packer, self.CAN, lat_active, lat.ramp_type, lat.precision_type,
        -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate
      ))
    return can_sends, apply_curvature

  def status(self) -> LateralStatus:
    """This frame's values for a BluePilot UI (HOSTS.md), as BP7 sets them on its car controller."""
    angle_mode = self.curv.primary_lateral_control == PrimaryLateralControl.angle
    return LateralStatus(
      lateralUncertainty=self.lateral_uncertainty,
      angleRateLimited=bool(angle_mode and self.angle.bp_angle_rate_limited),
      curvatureRateLimited=bool(angle_mode and self.angle.bp_curvature_rate_limited),
      curvatureDeviationLimited=bool(self.shared.bp_curvature_deviation_limited),
      humanTurnLateralPaused=bool(angle_mode and self.angle.angle_human_turn_active),
      stallBlipActive=bool(angle_mode and self.angle.angle_stall_blip_active),
      disable_BP_lat_UI=bool(self.disable_BP_lat_UI),
      primary_lateral_control=int(self.curv.primary_lateral_control),
    )

  def step_lka(self, CC, hud_control):
    """At LKA_STEP: the Lane_Assist_Data1 message."""
    # BluePilot: tell bluepilot_lateral/safety/ford.h whether angle mode is engaged, packed into
    # Lane_Assist_Data1's unused bits. shadow_curvature is negated to match the
    # path_angle/apply_curvature wire convention that ford.h's angle_meas is calibrated against.
    # Only called while BluePilot lateral is active (update returned True this frame), so the mode
    # alone decides.
    angle_mode_engaged = self.curv.primary_lateral_control == PrimaryLateralControl.angle
    shadow_curvature = -self.angle.bp_kappa_cmd if angle_mode_engaged else 0.0
    return fordcan_ext.create_lka_msg(
      self.packer, self.CAN, CC.latActive, hud_control, angle_mode_engaged, shadow_curvature
    )
