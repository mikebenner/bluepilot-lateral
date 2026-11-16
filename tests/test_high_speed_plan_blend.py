import unittest
from types import SimpleNamespace

from ..params import manifest
from ..params.manifest import value
from ..params.param_store import ParamStore
from .test_low_speed_low_curve_gain import _angle

KEY = "FordPlanBlendHighSpeed_ang"
MPH = 0.44704
REQUEST = 0.0004


def _run(angle, v, frames=60):
  """(kappa_cmd, path_angle) per frame for a held request. No model plan is given, so the plan curvature is 0
  and the commanded curvature is request * (1 - blend)."""
  CS = SimpleNamespace(out=SimpleNamespace(vEgoRaw=v, steeringPressed=False, steeringAngleDeg=0.0, yawRate=-0.0003 * v))
  CC = SimpleNamespace(latActive=True)
  out = []
  for _ in range(frames):
    pa = angle.update_angle_strategy(CC, CS, SimpleNamespace(curvature=REQUEST)).path_angle
    out.append((angle.bp_kappa_cmd, pa))
  return out


class TestHighSpeedPlanBlend(unittest.TestCase):
  def test_setting_default_and_clamp(self):
    self.assertEqual(value(ParamStore({}.items()), KEY), 0.5)
    self.assertEqual(value(ParamStore({KEY: b"0.25"}.items()), KEY), 0.25)
    self.assertEqual(value(ParamStore({KEY: b"9"}.items()), KEY), 0.5)
    self.assertEqual(value(ParamStore({KEY: b"-1"}.items()), KEY), 0.0)
    self.assertEqual(value(ParamStore({KEY: b"junk"}.items()), KEY), 0.5)
    self.assertIn(KEY, manifest.PER_FRAME_KEYS)

  def test_the_strategy_reads_it_from_the_host_params(self):
    a = _angle()
    self.assertEqual(a.high_speed_plan_blend, 0.5)
    a.update_angle_params(ParamStore({KEY: b"0.25"}.items()))
    self.assertEqual(a.high_speed_plan_blend, 0.25)
    a.update_angle_params(ParamStore({}.items()))
    self.assertEqual(a.high_speed_plan_blend, 0.5)

  def test_default_is_the_fixed_ratio_at_every_speed(self):
    for mph in (10, 30, 40, 45, 50, 55, 70, 85):
      base = _run(_angle(), mph * MPH)
      self.assertEqual(base[-1][0], REQUEST * 0.5, mph)
      # the same floats whether the attribute is the default or the fixed ratio written out
      self.assertEqual(base, _run(_angle(high_speed_plan_blend=0.50), mph * MPH), mph)

  def test_nothing_changes_at_40_mph_and_under(self):
    for mph in (10, 25, 39, 40):
      for blend in (0.0, 0.25):
        self.assertEqual(_run(_angle(), mph * MPH), _run(_angle(high_speed_plan_blend=blend), mph * MPH), (mph, blend))

  def test_the_setting_is_the_ratio_from_55_mph(self):
    for mph in (55, 65, 80):
      for blend in (0.0, 0.25, 0.4):
        k = _run(_angle(high_speed_plan_blend=blend), mph * MPH)[-1][0]
        self.assertAlmostEqual(k, REQUEST * (1.0 - blend), places=12, msg=(mph, blend))

  def test_ramp_between_40_and_55_mph(self):
    k = _run(_angle(high_speed_plan_blend=0.0), 47.5 * MPH)[-1][0]
    self.assertAlmostEqual(k, REQUEST * 0.75, places=9)
    last = REQUEST * 0.5
    for mph in (41, 44, 47, 50, 53, 55):
      k = _run(_angle(high_speed_plan_blend=0.0), mph * MPH)[-1][0]
      self.assertGreater(k, last)
      last = k

  def test_low_speed_ratio_still_comes_from_the_fixed_one(self):
    a = _angle(high_speed_plan_blend=0.0)
    a.path_angle_blend_ratio = 0.3
    self.assertAlmostEqual(_run(a, 20 * MPH)[-1][0], REQUEST * 0.7, places=12)
    self.assertAlmostEqual(_run(a, 70 * MPH)[-1][0], REQUEST, places=12)


if __name__ == "__main__":
  unittest.main()
