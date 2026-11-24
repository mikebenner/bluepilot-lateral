#!/usr/bin/env python3
"""
BluePilot Ford lateral, pinion source: the Python side keeps what it sends inside the panda's own band.

With STEER_ANGLE_CURVATURE the panda measures curvature from the raw pinion angle through its fixed geometry row,
while the Python side measures with the learned steer ratio, stiffness, angle offset and roll. A command at the edge of
the Python deviation clip (+-0.002 of the Python measurement) can then sit outside the panda's band (+-0.003 of the
panda's), and the panda drops the frame. InputTracker computes the panda's measurement by the panda's own arithmetic
and keeps the command in the part of the deviation window that is also inside the panda's band, less a margin.

These tests pin that arithmetic to the C table for every platform, run the angle strategy tick after tick with the two
measurements apart, and put the failing case (learned values at their defaults, 60 deg of wheel at 13 m/s on the
Expedition) through the compiled firmware with and without the clip.
"""
import math
import unittest
from types import SimpleNamespace

from ....can import CANPacker
from ....car.ford import fordcan
from ....car.ford.interface import CarInterface
from ....car.ford.values import CAR, CarControllerParams
from ...core.carcontroller_ext import BluePilotLateral, InputTracker, panda_pinion_geometry
from ...core.values_ext import (
  FordSafetyFlagsSP, FORD_PINION_FITTED_GEOMETRY, FORD_PINION_GEOMETRY_INDEX, FORD_PINION_GEOMETRY_SHIFT,
  PINION_SHADOW_BAND, PINION_SHADOW_BAND_MARGIN,
)
from .ford_bluepilot_common import (
  BPFordTestCase, CURVATURE_TO_CAN, CURVATURE_ERROR_MIN_SPEED, MAX_CURVATURE_ERROR_CAN_PINION, SMALL_ANGLE, pinion_geometry_table,
)

EXPEDITION = "FORD_EXPEDITION_MK4"
GEOMETRY = pinion_geometry_table()


def pinion_param_sp(platform: str) -> int:
  return int(FordSafetyFlagsSP.BP_LATERAL) | int(FordSafetyFlagsSP.STEER_ANGLE_CURVATURE) | (FORD_PINION_GEOMETRY_INDEX[platform] << FORD_PINION_GEOMETRY_SHIFT)


def car_state(speed: float, angle_deg: float, pressed: bool = False):
  return SimpleNamespace(out=SimpleNamespace(vEgoRaw=speed, steeringAngleDeg=angle_deg, yawRate=0.0, steeringPressed=pressed))


def tracker(platform: str, pinion: bool = True) -> InputTracker:
  CP = CarInterface.get_non_essential_params(platform)
  return InputTracker(CP, pinion_param_sp(platform) if pinion else int(FordSafetyFlagsSP.BP_LATERAL))


class TestPandaBandGeometry(unittest.TestCase):
  def test_python_geometry_is_the_c_row(self):
    """panda_pinion_geometry gives the C table's row for every platform that has one, fitted or CarSpecs, so the
    band the Python side computes is centered where the firmware's is."""
    checked = 0
    for name, idx in FORD_PINION_GEOMETRY_INDEX.items():
      if name not in set(CAR):
        continue
      with self.subTest(platform=name):
        slip, steer_ratio, wheelbase = panda_pinion_geometry(CarInterface.get_non_essential_params(name))
        c_slip, c_steer_ratio, c_wheelbase = GEOMETRY[idx]
        self.assertAlmostEqual(slip, c_slip, delta=abs(c_slip) * 1e-4)
        self.assertAlmostEqual(steer_ratio, c_steer_ratio, places=3)
        self.assertAlmostEqual(wheelbase, c_wheelbase, places=3)
        checked += 1
    self.assertGreaterEqual(checked, 10)
    self.assertEqual(panda_pinion_geometry(CarInterface.get_non_essential_params(EXPEDITION)), FORD_PINION_FITTED_GEOMETRY[EXPEDITION])

  def test_band_constants(self):
    # the band is the firmware's, and the clipped half-width still contains the whole Python deviation clip, so
    # when the two measurements agree the clip never binds
    self.assertAlmostEqual(PINION_SHADOW_BAND, MAX_CURVATURE_ERROR_CAN_PINION / CURVATURE_TO_CAN, places=9)
    self.assertGreater(PINION_SHADOW_BAND_MARGIN, 0)
    self.assertGreaterEqual(PINION_SHADOW_BAND - PINION_SHADOW_BAND_MARGIN, CarControllerParams.CURVATURE_ERROR)

  def test_measurement_by_hand(self):
    # curvature = pinion angle [rad] / (21.12 * 3.112 * (1 + 0.000684 * v^2)), OP sign (a positive pinion angle is negative)
    t = tracker(EXPEDITION)
    for angle_deg, speed, can in ((60.0, 13.0, -714.1), (20.0, 20.0, -208.5), (-5.0, 30.0, 41.09)):
      with self.subTest(angle_deg=angle_deg, speed=speed):
        self.assertAlmostEqual(t.get_panda_curvature(car_state(speed, angle_deg)) * CURVATURE_TO_CAN, can, delta=0.1)
    self.assertAlmostEqual(t.get_panda_curvature(car_state(0.0, 60.0)) * CURVATURE_TO_CAN, -796.6, delta=0.1)  # speed floored at 0.1


E = CarControllerParams.CURVATURE_ERROR
HALF = PINION_SHADOW_BAND - PINION_SHADOW_BAND_MARGIN


class TestPandaBandClip(unittest.TestCase):
  def test_yaw_source_is_untouched(self):
    """With the yaw source there is no second measurement: the command comes back as it went in, the same object,
    and the measurement published as shadow is get_current_curvature's."""
    t = tracker(EXPEDITION, pinion=False)
    cs = car_state(13.0, 60.0)
    self.assertIsNone(t.panda_geometry)
    self.assertIsNone(t.get_panda_curvature(cs))
    self.assertIsNone(t.panda_band(cs))
    for kappa in (-0.02, -0.0143, 0.0, 1e-9, 0.02):
      self.assertIs(t.clip_command_to_panda_band(kappa, kappa - E, kappa + E, cs), kappa)
    self.assertFalse(t.bp_panda_band_limited)
    self.assertFalse(t.bp_panda_band_disjoint)
    self.assertEqual(t.get_shadow_of_measurement(cs), t.get_current_curvature(cs))

  def test_band(self):
    t = tracker(EXPEDITION)
    cs = car_state(13.0, 60.0)
    low, high = t.panda_band(cs)
    self.assertAlmostEqual(low, t.get_panda_curvature(cs) - HALF, places=12)
    self.assertAlmostEqual(high, t.get_panda_curvature(cs) + HALF, places=12)

  def test_command_never_leaves_the_deviation_window(self):
    """Whatever the two measurements are, the result is inside [low, high], the deviation clip's own window, and
    inside the panda's band whenever the two overlap."""
    t = tracker(EXPEDITION)
    for speed in (9.5, 13.0, 20.0, 31.0):
      for angle_deg in (-90.0, -10.0, 0.0, 3.0, 45.0):
        cs = car_state(speed, angle_deg)
        p_low, p_high = t.panda_band(cs)
        center = (p_low + p_high) / 2
        for gap in (-0.006, -0.0046, -0.003, -0.0012, 0.0, 0.0007, 0.0025, 0.0046, 0.006):  # Python measurement minus the panda's
          low, high = center + gap - E, center + gap + E
          for kappa in (low, (low + high) / 2, high):
            with self.subTest(speed=speed, angle_deg=angle_deg, gap=gap, kappa=kappa):
              out = t.clip_command_to_panda_band(kappa, low, high, cs)
              self.assertGreaterEqual(out, low)
              self.assertLessEqual(out, high)
              overlap = max(low, p_low) <= min(high, p_high)
              self.assertEqual(t.bp_panda_band_disjoint, not overlap)
              if overlap:
                self.assertGreaterEqual(out, p_low)
                self.assertLessEqual(out, p_high)
                self.assertEqual(t.bp_panda_band_limited, out != kappa)
              else:
                self.assertEqual(out, kappa)  # nothing satisfies both: the deviation clip's result stands
                self.assertFalse(t.bp_panda_band_limited)

  def test_agreeing_measurements_never_bind(self):
    """When the Python measurement is the panda's, the whole deviation window is inside the band and nothing moves."""
    t = tracker(EXPEDITION)
    for speed in (9.5, 13.0, 20.0, 31.0):
      for angle_deg in (-90.0, -10.0, 0.0, 3.0, 45.0):
        cs = car_state(speed, angle_deg)
        center = t.get_panda_curvature(cs)
        for kappa in (center - E, center, center + E):
          self.assertEqual(t.clip_command_to_panda_band(kappa, center - E, center + E, cs), kappa)
          self.assertFalse(t.bp_panda_band_limited)
          self.assertFalse(t.bp_panda_band_disjoint)

  def test_disjoint_example_from_review(self):
    """Learned steer ratio 30 at 60 deg and 13 m/s: the Python measurement (-439 units) is 275 from the panda's (-714),
    further than the two half-widths together (230). An unwind to zero stays where the deviation clip put it, 100
    units from the Python measurement; it is not pulled back toward the panda's."""
    t = tracker(EXPEDITION)
    t.VM.update_params(1.0, 30.0)
    cs = car_state(13.0, 60.0)
    measured = t.get_current_curvature(cs)
    self.assertAlmostEqual(measured * CURVATURE_TO_CAN, -439.25, delta=0.5)
    unwind = measured + E
    self.assertEqual(t.clip_command_to_panda_band(unwind, measured - E, measured + E, cs), unwind)
    self.assertTrue(t.bp_panda_band_disjoint)
    self.assertFalse(t.bp_panda_band_limited)

  def test_shadow_of_measurement(self):
    t = tracker(EXPEDITION)
    cs = car_state(13.0, 60.0)
    low, high = t.panda_band(cs)
    measured = t.get_current_curvature(cs)  # default model: -775 units, inside the band around -714
    self.assertEqual(t.get_shadow_of_measurement(cs), measured)
    t.VM.update_params(1.0, 30.0)           # -439: outside, published at the band's edge
    self.assertEqual(t.get_shadow_of_measurement(cs), high)
    t.clip_command_to_panda_band(measured, measured - E, measured + E, cs)
    limited, disjoint = t.bp_panda_band_limited, t.bp_panda_band_disjoint
    t.get_shadow_of_measurement(cs)         # does not touch the command's flags
    self.assertEqual((t.bp_panda_band_limited, t.bp_panda_band_disjoint), (limited, disjoint))


def strategy(pinion: bool = True):
  CP = CarInterface.get_non_essential_params(EXPEDITION)
  bp = BluePilotLateral(CP, pinion_param_sp(EXPEDITION) if pinion else int(FordSafetyFlagsSP.BP_LATERAL),
                             CANPacker("ford_lincoln_base_pt"), fordcan.CanBus(CP))
  bp.angle.update_angle_params(None)
  bp.angle.low_speed_plan_blend = 0.0  # no model plan in these tests: the command is the request
  return bp.angle


def ticks(angle, request: float, speed: float, angle_deg: float, pressed: bool = False, frames: int = 40):
  """(kappa_cmd published, path_angle) per frame for a held request, speed and wheel angle. Yaw rate is given as the
  default model's curvature for that wheel angle, so the yaw and pinion sources measure the same thing here."""
  yaw = -tracker(EXPEDITION).get_current_curvature(car_state(speed, angle_deg)) * speed
  CS = SimpleNamespace(out=SimpleNamespace(vEgoRaw=speed, steeringPressed=pressed, steeringAngleDeg=angle_deg, yawRate=yaw))
  out = []
  for _ in range(frames):
    pa = angle.update_angle_strategy(SimpleNamespace(latActive=True), CS, SimpleNamespace(curvature=request)).path_angle
    out.append((angle.bp_kappa_cmd, pa))
  return out


class TestPandaBandInTheStrategy(unittest.TestCase):
  """The angle strategy itself, tick after tick: these fail if the calls in lateral_angle_ext.py are removed."""
  SPEED, ANGLE = 13.0, 60.0

  def setUp(self):
    cs = car_state(self.SPEED, self.ANGLE)
    t = tracker(EXPEDITION)
    self.measured = t.get_current_curvature(cs)   # the default model, as before the first learned values: -775 units
    self.panda = t.get_panda_curvature(cs)        # -714 units
    self.assertAlmostEqual((self.panda - self.measured) * CURVATURE_TO_CAN, 61, delta=1)

  def test_lead_is_held_inside_the_panda_band(self):
    a = strategy()
    out = ticks(a, self.measured * 3, self.SPEED, self.ANGLE)   # asks for far more, the same way
    for kappa, pa in out:
      self.assertTrue(math.isfinite(pa))
      self.assertAlmostEqual(kappa, self.panda - HALF, places=12)        # the band's edge, 844 units
      self.assertGreaterEqual(kappa, self.measured - E - 1e-12)          # and still inside the deviation window
    self.assertTrue(a.inputs.bp_panda_band_limited)
    self.assertTrue(a.shared.bp_curvature_deviation_limited)
    self.assertGreater(abs(out[-1][1]), abs(out[0][1]))                  # the path angle still builds toward the turn

  def test_same_request_with_the_yaw_source_is_the_deviation_edge(self):
    """What this build does with the setting off: exactly the deviation clip, as before."""
    a = strategy(pinion=False)
    for kappa, _ in ticks(a, self.measured * 3, self.SPEED, self.ANGLE):
      self.assertAlmostEqual(kappa, self.measured - E, places=12)
    self.assertFalse(a.inputs.bp_panda_band_limited)

  def test_unwind_is_not_held_back_when_inside_the_band(self):
    a = strategy()
    for kappa, _ in ticks(a, 0.0, self.SPEED, self.ANGLE):
      self.assertAlmostEqual(kappa, self.measured + E, places=12)        # 675 units: inside the band (584 to 844)
    self.assertFalse(a.inputs.bp_panda_band_limited)

  def test_disjoint_measurements_leave_the_deviation_clip_alone(self):
    a = strategy()
    a.inputs.lp = SimpleNamespace(steerRatio=30.0, stiffnessFactor=1.0, roll=0.0, angleOffsetDeg=0.0)
    a.inputs.VM.update_params(1.0, 30.0)
    measured = a.inputs.get_current_curvature(car_state(self.SPEED, self.ANGLE))
    for request, edge in ((0.0, measured + E), (measured * 5, measured - E)):
      for kappa, pa in ticks(a, request, self.SPEED, self.ANGLE, frames=10):
        self.assertTrue(math.isfinite(pa))
        self.assertAlmostEqual(kappa, edge, places=12)
      self.assertTrue(a.inputs.bp_panda_band_disjoint)

  def test_pressed_publishes_the_measurement_inside_the_band(self):
    a = strategy()
    a.inputs.lp = SimpleNamespace(steerRatio=30.0, stiffnessFactor=1.0, roll=0.0, angleOffsetDeg=0.0)
    a.inputs.VM.update_params(1.0, 30.0)
    kappa, _ = ticks(a, 0.0, self.SPEED, self.ANGLE, pressed=True, frames=3)[-1]
    self.assertAlmostEqual(kappa, self.panda + HALF, places=12)

  def test_below_the_speed_gate_nothing_is_clipped(self):
    a = strategy()
    speed = 8.5
    request = tracker(EXPEDITION).get_current_curvature(car_state(speed, self.ANGLE)) * 1.5
    for kappa, _ in ticks(a, request, speed, self.ANGLE, frames=5):
      self.assertAlmostEqual(kappa, request, places=12)


class TestPandaBandThroughFirmware(BPFordTestCase):
  """The case both reviews of the fitted row raised, through libsafety: the Python side still on its default model
  (no learned values yet) at 60 deg of wheel and 13 m/s, commanding its full same-direction lead."""
  CANFD = True
  PARAM_SP = pinion_param_sp(EXPEDITION)
  cnt = 0

  def _measure(self, speed: float, angle_deg: float):
    for _ in range(14):  # enough frames for the rx counter to sync and all six samples to hold this angle
      self.rx(self.stock._speed_msg(speed))
      self.rx(self.stock._speed_msg_2(speed))
      values = {"StePinComp_An_Est": angle_deg, "StePinCompAnEst_D_Qf": 3, "StePinAn_No_Cnt": self.cnt % 16}
      self.__class__.cnt += 1
      self.rx(self.stock.packer.make_can_msg_safety("SteeringPinion_Data", 0, values))

  def _accepted(self, kappa_op: float) -> bool:
    self.safety.set_controls_allowed(True)
    self.tx(self.lka_bp_status_msg(True, -kappa_op))  # the shadow rides in Ford sign
    return self.tx(self.lat(True, 0, SMALL_ANGLE, 0, 0))

  def test_default_model_lead(self):
    speed, angle_deg = 13.0, 60.0
    self.assertGreater(speed, CURVATURE_ERROR_MIN_SPEED)
    t = tracker(EXPEDITION)  # no update(): the vehicle model is CarSpecs, as before the first learned values arrive
    cs = car_state(speed, angle_deg)
    self._measure(speed, angle_deg)
    self.assertAlmostEqual(self.meas()[1], 714, delta=1)

    measured = t.get_current_curvature(cs)
    self.assertAlmostEqual(-measured * CURVATURE_TO_CAN, 775, delta=1)  # the Python default model, 61 units from the panda's 714
    lead = measured - CarControllerParams.CURVATURE_ERROR  # the deviation clip's edge, turning further the same way
    self.assertFalse(self._accepted(lead), "the unclipped lead is outside the panda's band: this is the failure")

    self._measure(speed, angle_deg)
    clipped = t.clip_command_to_panda_band(lead, measured - E, measured + E, cs)
    self.assertTrue(t.bp_panda_band_limited)
    self.assertAlmostEqual(-clipped * CURVATURE_TO_CAN, 714.1 + 130, delta=0.2)
    self.assertTrue(self._accepted(clipped))
    # the other side of the Python clip was inside the band all along and is not moved
    unwind = measured + CarControllerParams.CURVATURE_ERROR
    self.assertEqual(t.clip_command_to_panda_band(unwind, measured - E, measured + E, cs), unwind)
    self.assertTrue(self._accepted(unwind))

  def test_band_edges_are_accepted_at_any_speed(self):
    """Both edges of the band the Python side computes pass the compiled check, across speeds and wheel angles."""
    t = tracker(EXPEDITION)
    for speed in (10.5, 13.0, 20.0, 31.0):
      for angle_deg in (-120.0, -30.0, 0.0, 7.5, 60.0):
        cs = car_state(speed, angle_deg)
        if abs(t.get_panda_curvature(cs)) + PINION_SHADOW_BAND > 0.0327:  # the shadow is an int16 of 1e-6 1/m
          continue
        self._measure(speed, angle_deg)
        for edge in t.panda_band(cs):
          with self.subTest(speed=speed, angle_deg=angle_deg, edge=edge):
            self.assertTrue(self._accepted(edge))


if __name__ == "__main__":
  unittest.main()
