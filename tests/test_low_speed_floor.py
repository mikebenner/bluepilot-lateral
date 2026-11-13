import unittest
from types import SimpleNamespace

from ...can import CANPacker
from ...car.ford import fordcan
from ...car.ford.interface import CarInterface
from ...car.ford.values import CAR
from ..core.carcontroller_ext import BluePilotLateral
from ..core.lateral_angle_ext import FORD_DBC_PATH_ANGLE_MAX, path_angle_speed
from ..core.values_ext import FordSafetyFlagsSP
from ..params.manifest import value
from ..params.param_store import ParamStore

MPH = 0.44704
FLOOR_MPH = 45.5
FLOOR_MS = FLOOR_MPH * MPH


def _angle(floor_mph):
  CP = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  bp = BluePilotLateral(CP, FordSafetyFlagsSP.BP_LATERAL, CANPacker("ford_lincoln_base_pt"), fordcan.CanBus(CP))
  bp.angle.update_angle_params(None)
  bp.angle.low_speed_floor_mph = floor_mph
  return bp.angle


def _settle(angle, v, kappa, frames=200):
  """path_angle after holding one speed and curvature, with the car turning exactly as asked."""
  CS = SimpleNamespace(out=SimpleNamespace(vEgoRaw=v, steeringPressed=False, steeringAngleDeg=0.0, yawRate=-kappa * v))
  CC = SimpleNamespace(latActive=True)
  out = []
  for _ in range(frames):
    out.append(angle.update_angle_strategy(CC, CS, SimpleNamespace(curvature=kappa)).path_angle)
  return out


class TestPathAngleSpeed(unittest.TestCase):
  def test_off(self):
    for v in (0.0, 3.0, 9.0, 15.0, 30.0):
      self.assertEqual(path_angle_speed(v, 0.0), v)

  def test_held_up_to_the_floor_from_20_mph(self):
    for v in (9.0, 12.0, 20.0):
      self.assertAlmostEqual(path_angle_speed(v, FLOOR_MPH), FLOOR_MS)

  def test_unchanged_at_and_above_the_floor(self):
    for v in (FLOOR_MS, 25.0, 35.0):
      self.assertEqual(path_angle_speed(v, FLOOR_MPH), v)

  def test_fades_out_below_20_mph(self):
    for v in (0.0, 2.0, 4.5):
      self.assertEqual(path_angle_speed(v, FLOOR_MPH), v)
    self.assertAlmostEqual(path_angle_speed(6.75, FLOOR_MPH), 6.75 + 0.5 * (FLOOR_MS - 6.75))

  def test_never_falls_as_speed_rises(self):
    last = 0.0
    for i in range(0, 4001):
      s = path_angle_speed(i / 100, FLOOR_MPH)
      self.assertGreaterEqual(s, last - 1e-9)
      self.assertLess(s - last, 0.05)  # no step: at most 3.5 m/s per m/s, in the fade
      last = s

  def test_a_floor_below_the_fade_does_nothing(self):
    for v in (0.0, 2.0, 4.0, 10.0):
      self.assertEqual(path_angle_speed(v, 10.0), v)  # 10 mph = 4.47 m/s

  def test_negative_speed(self):
    self.assertEqual(path_angle_speed(-1.0, FLOOR_MPH), 0.0)


class TestLowSpeedFloor(unittest.TestCase):
  def test_setting_default_and_clamp(self):
    self.assertEqual(value(ParamStore({}.items()), "FordLowSpeedFloor_ang"), 0.0)
    self.assertEqual(value(ParamStore(dict(FordLowSpeedFloor_ang=b"45.5").items()), "FordLowSpeedFloor_ang"), 45.5)
    self.assertEqual(value(ParamStore(dict(FordLowSpeedFloor_ang=b"99").items()), "FordLowSpeedFloor_ang"), 50.0)
    self.assertEqual(value(ParamStore(dict(FordLowSpeedFloor_ang=b"-3").items()), "FordLowSpeedFloor_ang"), 0.0)

  def test_default_is_off(self):
    self.assertEqual(_angle(0.0).low_speed_floor_mph, 0.0)
    CP = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
    bp = BluePilotLateral(CP, FordSafetyFlagsSP.BP_LATERAL, CANPacker("ford_lincoln_base_pt"), fordcan.CanBus(CP))
    self.assertEqual(bp.angle.low_speed_floor_mph, 0.0)

  def test_command_scales_by_floor_over_speed(self):
    # 27 mph, a 250 m radius: the settled command is floor / speed times the one without the floor
    v, kappa = 12.0, 0.004
    off, on = _settle(_angle(0.0), v, kappa)[-1], _settle(_angle(FLOOR_MPH), v, kappa)[-1]
    self.assertGreater(off, 0.0)
    self.assertAlmostEqual(on / off, FLOOR_MS / v, places=6)

  def test_same_command_at_every_speed_below_the_floor(self):
    # where the gain is flat (below 13.5 m/s), one curvature gives one path angle whatever the speed
    kappa = 0.004
    a, b = _settle(_angle(FLOOR_MPH), 9.5, kappa)[-1], _settle(_angle(FLOOR_MPH), 13.0, kappa)[-1]
    self.assertAlmostEqual(a, b, places=6)

  def test_identical_above_the_floor(self):
    for v in (21.0, 27.0, 33.0):
      self.assertEqual(_settle(_angle(0.0), v, 0.002), _settle(_angle(FLOOR_MPH), v, 0.002))

  def test_identical_at_walking_speed(self):
    for v in (1.0, 4.5):
      self.assertEqual(_settle(_angle(0.0), v, 0.01), _settle(_angle(FLOOR_MPH), v, 0.01))

  def test_rate_limit_and_range_still_bound_the_command(self):
    # a curvature far beyond the range at 22 mph: each call moves by no more than the soft ROC there
    # (0.055 rad), and the command stops at the DBC maximum
    out = _settle(_angle(FLOOR_MPH), 10.0, 0.05, frames=40)
    steps = [b - a for a, b in zip([0.0] + out, out, strict=False)]
    self.assertLessEqual(max(steps), 0.055 + 1e-9)
    self.assertLessEqual(max(out), FORD_DBC_PATH_ANGLE_MAX)
    self.assertGreater(max(out), 0.45)


if __name__ == "__main__":
  unittest.main()
