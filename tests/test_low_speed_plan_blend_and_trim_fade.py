import unittest
from types import SimpleNamespace

from ..core.lane_center_trim import LaneCenterTrim
from ..core.values_ext import MODEL_T_IDXS
from ..params import manifest
from ..params.manifest import value
from ..params.param_store import ParamStore
from .test_high_speed_plan_blend import REQUEST, _run
from .test_lane_center_trim import _good_model
from .test_low_speed_low_curve_gain import _angle

BLEND = "FordPlanBlendLowSpeed_ang"
FADE = "FordLaneTrimFadeRadius_ang"
MPH = 0.44704


def _fade(angle, request, plan=None, v=17.0):
  """The lane trim's authority after one frame at this request, with an optional model plan (yaw rate over
  MODEL_T_IDXS)."""
  if plan is not None:
    angle.inputs.model = SimpleNamespace(orientationRate=SimpleNamespace(z=list(plan)),
                                         meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0))
  CS = SimpleNamespace(out=SimpleNamespace(vEgoRaw=v, steeringPressed=False, steeringAngleDeg=0.0, yawRate=-request * v))
  angle.update_angle_strategy(SimpleNamespace(latActive=True), CS, SimpleNamespace(curvature=request))
  return angle.bp_lane_trim_fade


class TestLowSpeedPlanBlend(unittest.TestCase):
  def test_setting_default_and_clamp(self):
    self.assertEqual(value(ParamStore({}.items()), BLEND), 0.5)
    self.assertEqual(value(ParamStore({BLEND: b"0.1"}.items()), BLEND), 0.1)
    self.assertEqual(value(ParamStore({BLEND: b"9"}.items()), BLEND), 0.5)
    self.assertEqual(value(ParamStore({BLEND: b"-1"}.items()), BLEND), 0.0)
    self.assertTrue(BLEND in manifest.PER_FRAME_KEYS)

  def test_the_strategy_reads_it_from_the_host_params(self):
    a = _angle()
    self.assertEqual(a.low_speed_plan_blend, 0.5)
    a.update_angle_params(ParamStore({BLEND: b"0.2"}.items()))
    self.assertEqual(a.low_speed_plan_blend, 0.2)
    a.update_angle_params(ParamStore({}.items()))
    self.assertEqual(a.low_speed_plan_blend, 0.5)

  def test_default_gives_the_same_floats_as_the_fixed_ratio(self):
    for mph in (10, 30, 40, 47, 55, 70):
      self.assertEqual(_run(_angle(), mph * MPH), _run(_angle(low_speed_plan_blend=0.50), mph * MPH), mph)

  def test_the_setting_is_the_ratio_up_to_40_mph(self):
    for mph in (22, 30, 40):
      for blend in (0.0, 0.25, 0.4):
        k = _run(_angle(low_speed_plan_blend=blend), mph * MPH)[-1][0]
        self.assertAlmostEqual(k, REQUEST * (1.0 - blend), places=12, msg=(mph, blend))

  def test_nothing_changes_from_55_mph(self):
    for mph in (55, 70):
      self.assertEqual(_run(_angle(), mph * MPH), _run(_angle(low_speed_plan_blend=0.0), mph * MPH), mph)

  def test_mixes_with_the_high_speed_setting_between_40_and_55_mph(self):
    k = _run(_angle(low_speed_plan_blend=0.0, high_speed_plan_blend=0.5), 47.5 * MPH)[-1][0]
    self.assertAlmostEqual(k, REQUEST * 0.75, places=9)
    k = _run(_angle(low_speed_plan_blend=0.0, high_speed_plan_blend=0.0), 47.5 * MPH)[-1][0]
    self.assertAlmostEqual(k, REQUEST, places=12)


class TestLaneTrimFade(unittest.TestCase):
  def test_setting_default_and_clamp(self):
    self.assertEqual(value(ParamStore({}.items()), FADE), 0.0)
    self.assertEqual(value(ParamStore({FADE: b"400"}.items()), FADE), 400.0)
    self.assertEqual(value(ParamStore({FADE: b"5000"}.items()), FADE), 1000.0)
    self.assertEqual(value(ParamStore({FADE: b"-3"}.items()), FADE), 0.0)
    self.assertTrue(FADE in manifest.PER_FRAME_KEYS)

  def test_the_strategy_reads_it_from_the_host_params(self):
    a = _angle()
    self.assertEqual(a.lane_trim_fade_radius, 0.0)
    a.update_angle_params(ParamStore({FADE: b"400"}.items()))
    self.assertEqual(a.lane_trim_fade_radius, 400.0)

  def test_off_by_default_at_any_curvature(self):
    for request in (0.0, 0.002, 0.02):
      self.assertEqual(_fade(_angle(), request), 1.0)

  def test_full_beyond_twice_the_radius_none_inside_it(self):
    for request, want in ((0.0, 1.0), (1 / 800, 1.0), (-1 / 800, 1.0), (1 / 400, 0.0), (-1 / 100, 0.0)):
      self.assertAlmostEqual(_fade(_angle(lane_trim_fade_radius=400.0), request), want, places=9, msg=request)
    self.assertAlmostEqual(_fade(_angle(lane_trim_fade_radius=400.0), 0.75 / 400), 0.5, places=9)

  def test_a_curve_in_the_plans_next_second_fades_it_before_the_request_rises(self):
    v = 17.0
    plan = [(1 / 200 * v if 0.5 <= t <= 1.0 else 0.0) for t in MODEL_T_IDXS]
    self.assertEqual(_fade(_angle(lane_trim_fade_radius=400.0), 0.0, plan, v), 0.0)
    later = [(1 / 200 * v if t > 1.5 else 0.0) for t in MODEL_T_IDXS]
    self.assertEqual(_fade(_angle(lane_trim_fade_radius=400.0), 0.0, later, v), 1.0)
    self.assertEqual(_fade(_angle(), 0.0, plan, v), 1.0)

  def test_a_tiny_radius_is_treated_as_50_m(self):
    self.assertEqual(_fade(_angle(lane_trim_fade_radius=1.0), 1 / 150), 1.0)
    self.assertEqual(_fade(_angle(lane_trim_fade_radius=1.0), 1 / 50), 0.0)

  def _strategy_trim(self, radius, request):
    """The trim the strategy applies after 20 s on a held curve, the truck 0.4 m off the lane center."""
    a = _angle(lane_trim_fade_radius=radius, enable_lane_positioning_ang=True, lane_centering_strength_ang=0.5)
    model = _good_model(lane_center_y=0.4)
    model.orientationRate = SimpleNamespace(z=[request * 25.0] * len(MODEL_T_IDXS))
    model.meta = SimpleNamespace(laneChangeState=0, laneChangeDirection=0)
    a.inputs.model = model
    CS = SimpleNamespace(out=SimpleNamespace(vEgoRaw=25.0, steeringPressed=False, steeringAngleDeg=0.0, yawRate=-request * 25.0))
    for _ in range(400):
      a.update_angle_strategy(SimpleNamespace(latActive=True), CS, SimpleNamespace(curvature=request))
    return a.lane_center_trim.correction

  def test_the_strategy_hands_the_fade_to_the_trim(self):
    gentle, tight = 1 / 2000, 1 / 300
    self.assertNotEqual(self._strategy_trim(0.0, tight), 0.0)
    self.assertEqual(self._strategy_trim(0.0, tight), self._strategy_trim(0.0, gentle))
    self.assertAlmostEqual(self._strategy_trim(400.0, tight), 0.0, places=12)
    self.assertEqual(self._strategy_trim(400.0, gentle), self._strategy_trim(0.0, gentle))

  def _settled(self, fade):
    trim = LaneCenterTrim()
    model = _good_model(lane_center_y=0.4)
    out = 0.0
    for _ in range(400):
      out = trim.update(0.0, model, 25.0, True, 0.0, 0.5, True, False, *fade)
    return out

  def test_the_trim_scales_with_the_fade(self):
    full = self._settled(())
    self.assertNotEqual(full, 0.0)
    self.assertEqual(self._settled((1.0,)), full)
    self.assertAlmostEqual(self._settled((0.5,)), 0.5 * full, places=12)
    self.assertAlmostEqual(self._settled((0.0,)), 0.0, places=12)
    self.assertAlmostEqual(self._settled((float("nan"),)), 0.0, places=12)

  def test_it_eases_out_at_the_trims_own_rate(self):
    trim = LaneCenterTrim()
    model = _good_model(lane_center_y=0.4)
    for _ in range(400):
      before = trim.update(0.0, model, 25.0, True, 0.0, 0.5, True, False)
    after = trim.update(0.0, model, 25.0, True, 0.0, 0.5, True, False, 0.0)
    self.assertLess(abs(after), abs(before))
    self.assertGreater(abs(after), 0.5 * abs(before))


if __name__ == "__main__":
  unittest.main()
