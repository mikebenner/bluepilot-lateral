import unittest
from types import SimpleNamespace

from ...can import CANPacker
from ...car.ford import fordcan
from ...car.ford.interface import CarInterface
from ...car.ford.values import CAR
from ..core.carcontroller_ext import BluePilotLateral
from ..core.values_ext import ANGLE_GAIN_CANFD_BOF, FordSafetyFlagsSP
from ..params.manifest import value
from ..params.param_store import ParamStore

KEY = "FordLowSpeedDampening_ang"
GENTLE, TIGHT = 0.0004, 0.002  # 2500 m and 500 m radius: either side of the 0.0007-0.001 blend


def _angle(factor=None, **settings):
  CP = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  bp = BluePilotLateral(CP, FordSafetyFlagsSP.BP_LATERAL, CANPacker("ford_lincoln_base_pt"), fordcan.CanBus(CP))
  bp.angle.update_angle_params(None)
  if factor is not None:
    bp.angle.low_speed_dampening_factor = factor
  for k, v in settings.items():
    setattr(bp.angle, k, v)
  return bp.angle


def _settle(angle, v, kappa, frames=200):
  """path_angle after holding one speed and one commanded curvature, with the car turning exactly as commanded.
  These tests give the strategy no model plan, so its blend with the plan (path_angle_blend_ratio) scales the
  request down: the request is set to what leaves kappa as the curvature the gain acts on."""
  CS = SimpleNamespace(out=SimpleNamespace(vEgoRaw=v, steeringPressed=False, steeringAngleDeg=0.0, yawRate=-kappa * v))
  CC = SimpleNamespace(latActive=True)
  request = kappa / (1.0 - angle.path_angle_blend_ratio)
  out = []
  for _ in range(frames):
    out.append(angle.update_angle_strategy(CC, CS, SimpleNamespace(curvature=request)).path_angle)
  return out


class TestLowSpeedLowCurveGain(unittest.TestCase):
  def test_setting_default_and_clamp(self):
    self.assertEqual(value(ParamStore({}.items()), KEY), 1.0)
    self.assertEqual(value(ParamStore({KEY: b"1.3"}.items()), KEY), 1.3)
    self.assertEqual(value(ParamStore({KEY: b"9"}.items()), KEY), 1.5)
    self.assertEqual(value(ParamStore({KEY: b"0"}.items()), KEY), 0.5)
    self.assertEqual(value(ParamStore({KEY: b"junk"}.items()), KEY), 1.0)

  def test_default_is_the_gain_it_replaced(self):
    self.assertEqual(_angle().low_speed_dampening_factor, 1.0)
    # 1.0 is what the code held fixed: a gentle curve at low speed is commanded at kappa * v
    for v in (6.0, 10.0, 13.5):
      self.assertAlmostEqual(_settle(_angle(), v, GENTLE)[-1], GENTLE * v, places=9)

  def test_the_strategy_reads_it_from_the_host_params(self):
    a = _angle()
    a.update_angle_params(ParamStore({KEY: b"1.3"}.items()))
    self.assertEqual(a.low_speed_dampening_factor, 1.3)
    a.update_angle_params(ParamStore({}.items()))
    self.assertEqual(a.low_speed_dampening_factor, 1.0)

  def test_scales_a_gentle_curve_at_low_speed(self):
    for v in (6.0, 10.0, 13.5):
      for kappa in (0.0002, GENTLE, 0.0007, -GENTLE):
        off, on = _settle(_angle(1.0), v, kappa)[-1], _settle(_angle(1.3), v, kappa)[-1]
        self.assertAlmostEqual(on / off, 1.3, places=6, msg=(v, kappa))

  def test_tighter_curves_are_unchanged(self):
    for v in (6.0, 13.0, 20.0, 30.0):
      for kappa in (0.001, TIGHT, 0.006, -TIGHT):
        self.assertEqual(_settle(_angle(1.0), v, kappa), _settle(_angle(1.5), v, kappa), (v, kappa))

  def test_nothing_changes_from_60_mph(self):
    for v in (26.82, 30.0, 35.0):
      for kappa in (0.0002, GENTLE, 0.0008):
        self.assertEqual(_settle(_angle(1.0), v, kappa), _settle(_angle(1.5), v, kappa), (v, kappa))

  def test_blends_into_the_high_speed_low_curve_gain(self):
    # halfway between 13.5 and 26.82 m/s the gain is halfway between the setting and the high speed value
    v = (13.5 + 26.82) / 2
    high = ANGLE_GAIN_CANFD_BOF[0] * 1.25
    got = _settle(_angle(1.3, user_dampening_factor=1.25), v, GENTLE)[-1]  # 1.3 is the cap at the default low speed factor
    self.assertAlmostEqual(got / (GENTLE * v), (1.3 + high) / 2, places=6)

  def test_at_the_high_curve_gain_the_radius_no_longer_matters(self):
    # 1.30 * FordLowSpeedFactor_ang is the low-speed gain of a curve under 1000 m: at that value one
    # gain applies at every radius, so path_angle / kappa is the same number across the blend
    for v in (8.0, 13.5):
      a = _angle(1.3)
      ratios = [_settle(a, v, k)[-1] / k for k in (0.0002, 0.0006, 0.0008, 0.00095, 0.001, 0.002)]
      for r in ratios:
        self.assertAlmostEqual(r, ratios[0], places=6)

  def test_raising_it_stops_at_the_high_curve_gain(self):
    for low_factor in (0.8, 1.0):
      cap = max(1.0, 1.30 * low_factor)
      at_cap = _settle(_angle(cap, low_speed_curv_factor=low_factor), 12.0, GENTLE)
      self.assertEqual(_settle(_angle(1.5, low_speed_curv_factor=low_factor), 12.0, GENTLE), at_cap, low_factor)

  def test_the_default_is_untouched_whatever_the_low_speed_factor(self):
    # 1.0 stays 1.0 even where the high-curve gain is below it (FordLowSpeedFactor_ang under 0.77)
    for low_factor in (0.5, 0.7, 1.0, 1.5):
      self.assertAlmostEqual(_settle(_angle(1.0, low_speed_curv_factor=low_factor), 12.0, GENTLE)[-1], GENTLE * 12.0, places=9)

  def test_raising_it_never_makes_the_command_fall_as_curvature_rises(self):
    # wherever the command rises with curvature at the default, it still does with the setting at any value above
    def rises(low_factor, factor):
      a, last = _angle(factor, low_speed_curv_factor=low_factor), 0.0
      for i in range(1, 300):
        pa = _settle(a, 12.0, i * 1e-5, frames=60)[-1]
        if pa <= last:
          return False
        last = pa
      return True
    for low_factor in (0.8, 1.0, 1.5):
      self.assertTrue(rises(low_factor, 1.0), low_factor)
      for factor in (1.15, 1.3, 1.5):
        self.assertTrue(rises(low_factor, factor), (low_factor, factor))

  def test_combines_with_the_floor(self):
    # floor 42 mph (18.78 m/s) and the gain at 1.30: a gentle curve at 27 mph is commanded as kappa * 18.78 * 1.30
    a = _angle(1.3, low_speed_floor_mph=42.0)
    self.assertAlmostEqual(_settle(a, 12.0, GENTLE)[-1], GENTLE * 42.0 * 0.44704 * 1.3, places=9)


if __name__ == "__main__":
  unittest.main()
