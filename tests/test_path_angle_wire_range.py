"""The path angle the angle strategy commands must stay inside LatCtlPath_An_Actl's range as it goes on the wire.

The strategy holds path_angle in its own sign and the car controller sends -path_angle. The signal is 11 bits unsigned
with an offset (-0.5 to +0.5235 rad), so a wire value just below -0.5 does not saturate: it packs as the top of the
range, a full-scale command the other way. The strategy's clamp used the signal's limits without the sign flip
(-0.5 to +0.5235 on its own value), which let a tight turn to one side reach -0.5235 on the wire.

These tests run the car controller's lateral step to the limit on both sides, on CAN FD and CAN, and read the path
angle back out of the frame it built."""
import unittest
from types import SimpleNamespace

from ...can.dbc import DBC
from ...can.packer import CANPacker
from ...car.ford import fordcan
from ...car.ford.interface import CarInterface
from ...car.ford.values import CAR, FordFlags
from ..core.carcontroller_ext import BluePilotLateral
from ..core.lateral_angle_ext import FORD_DBC_PATH_ANGLE_MAX, FORD_DBC_PATH_ANGLE_MIN, PATH_ANGLE_LIMIT
from ..core.lateral_curv_ext import PrimaryLateralControl
from ..core.values_ext import FordSafetyFlagsSP
from .test_lat_ctl2_checksum import DBC_NAME, _raw

SPEED = 8.0        # m/s: a tight low-speed turn, below the speed where the deviation clip starts
RESOLUTION = 0.0005


def _lateral(platform):
  CP = CarInterface.get_non_essential_params(platform)
  bp = BluePilotLateral(CP, FordSafetyFlagsSP.BP_LATERAL, CANPacker(DBC_NAME), fordcan.CanBus(CP))
  bp.angle.update_angle_params(None)
  bp.angle.low_speed_plan_blend = 0.0   # no model plan here: the command is the request
  bp.curv.primary_lateral_control = PrimaryLateralControl.angle
  bp.frame = 0
  return bp, CP


def _wire_path_angle(CP, can_sends):
  """LatCtlPath_An_Actl as a receiver decodes it from the frame the lateral step built."""
  name = "LateralMotionControl2" if CP.flags & FordFlags.CANFD else "LateralMotionControl"
  msg = next(m for m in DBC(DBC_NAME).msgs.values() if m.name == name)
  sig = msg.sigs["LatCtlPath_An_Actl"]
  dat = next(c[1] for c in can_sends if c[0] == msg.address)
  return _raw(sig, dat) * sig.factor + sig.offset


def _step(bp, CP, request):
  CS = SimpleNamespace(out=SimpleNamespace(vEgoRaw=SPEED, steeringPressed=False, steeringAngleDeg=0.0, yawRate=-request * SPEED))
  can_sends, _ = bp.step_lateral(SimpleNamespace(latActive=True), CS, SimpleNamespace(curvature=request), 0.0)
  bp.frame += 5
  return bp.angle.bp_path_angle_final, _wire_path_angle(CP, can_sends)


class TestPathAngleWireRange(unittest.TestCase):
  def test_limit_fits_the_signal_both_ways(self):
    self.assertEqual(PATH_ANGLE_LIMIT, min(-FORD_DBC_PATH_ANGLE_MIN, FORD_DBC_PATH_ANGLE_MAX))
    sig = next(m for m in DBC(DBC_NAME).msgs.values() if m.name == "LateralMotionControl2").sigs["LatCtlPath_An_Actl"]
    self.assertAlmostEqual(sig.offset, FORD_DBC_PATH_ANGLE_MIN, places=9)
    self.assertAlmostEqual((2 ** sig.size - 1) * sig.factor + sig.offset, FORD_DBC_PATH_ANGLE_MAX, places=9)

  def test_no_wrap_at_either_limit(self):
    """Approach the limit as a turn does, a little under it and then asked for much more, on each side. Every frame's
    wire value is the command negated, never the other end of the range.

    The approach stops just short of where the strategy stops letting the path angle grow (0.471 on the positive
    side, 0.45 on the negative), so the next frame is one full rate step further before the clamp: 0.523 on the
    positive side, which is past the wire's -0.5, and -0.503 on the negative side. Both are held at the limit."""
    for platform in (CAR.FORD_EXPEDITION_MK4, CAR.FORD_ESCAPE_MK4):
      for side, approach in ((1, 0.468), (-1, 0.448)):
        with self.subTest(platform=str(platform), side=side):
          bp, CP = _lateral(platform)
          # what one unit of requested curvature commands, once settled (the high-curvature gain at this speed)
          probe = 0.01
          for _ in range(40):
            settled, _ = _step(bp, CP, probe)
          per_curvature = settled / probe
          self.assertGreater(per_curvature, 0)

          bp, CP = _lateral(platform)
          seen = []
          for request in [side * approach / per_curvature] * 40 + [side * 5.0 / per_curvature] * 40:
            path_angle, wire = _step(bp, CP, request)
            seen.append(path_angle)
            self.assertAlmostEqual(wire, -path_angle, delta=RESOLUTION / 2 + 1e-9)
            self.assertLessEqual(abs(path_angle), PATH_ANGLE_LIMIT)
            self.assertLessEqual(abs(wire), PATH_ANGLE_LIMIT + 1e-9)
            if abs(path_angle) > RESOLUTION:
              self.assertEqual(wire > 0, path_angle < 0, "the wire value turns the same way as the command")
          # the approach really got there, and the frame after it asked for a full rate step more: held at the limit
          self.assertAlmostEqual(side * seen[39], approach, delta=0.002)
          self.assertEqual(side * seen[-1], PATH_ANGLE_LIMIT)


if __name__ == "__main__":
  unittest.main()
