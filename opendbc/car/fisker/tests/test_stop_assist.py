"""
Stop assist: with the car's own ACC (the Hydra) driving, the port lowers its 0x121 request for a stop the driving
model predicts (stop_assist.py), by sending 0x121 candidates for panda to swap in (fisker_stop_assist_modify).
"""
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.fisker.carcontroller import ACCEL_CANDIDATE_ADDR, LAT_CANDIDATE_BUS, CarController
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.fiskercan import FiskerCAN, fisker_plain_checksum
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.stop_assist import StopAssist, StopAssistState
from opendbc.car.fisker.values import CANBUS, CAR, DBC, CarControllerParams as P, FiskerSafetyFlags
from opendbc.sunnypilot.car.fisker.values_ext import FiskerFlagsSP
from opendbc.testing import parameterized

KPH = 1 / 3.6
DOWN = P.STOP_ASSIST_JERK_DOWN * 0.01
UP = P.STOP_ASSIST_JERK_UP * 0.01


def accel_of(data: bytes) -> float:
  return int.from_bytes(data[2:4], 'big') * P.ACCEL_SCALE + P.ACCEL_OFFSET


class TestStopAssist(unittest.TestCase):
  def setUp(self):
    self.sa = StopAssist()

  def frames(self, n, allowed=True, predicted=True, model=-1.0, hydra=0.5, v=50 * KPH):
    return [self.sa.update(allowed, predicted, model, hydra, v) for _ in range(n)]

  def test_idle_without_a_stop(self):
    assert self.frames(10, predicted=False) == [None] * 10
    assert self.sa.state == StopAssistState.off

  def test_ramps_down_from_the_hydra_to_the_model(self):
    out = self.frames(100, model=-1.0, hydra=0.5)
    self.assertAlmostEqual(out[0], 0.5 - DOWN)
    for a, b in zip(out, out[1:]):
      assert -DOWN - 1e-9 <= b - a <= 0
    self.assertAlmostEqual(out[-1], -1.0)
    assert self.sa.state == StopAssistState.slowing

  @parameterized("model,settles", [(-3.0, P.STOP_ASSIST_ACCEL_MIN), (0.8, 0.0)])
  def test_target_clipped(self, model, settles):
    self.assertAlmostEqual(self.frames(200, model=model, hydra=1.0)[-1], settles)

  @parameterized("v", [10 * KPH, 14 * KPH, 81 * KPH])
  def test_starts_only_between_15_and_80_kph(self, v):
    assert self.frames(10, v=v) == [None] * 10

  def test_keeps_going_below_start_speed(self):
    self.frames(5)
    assert self.frames(1, v=13 * KPH)[0] is not None
    assert self.sa.state == StopAssistState.slowing

  def test_hands_back_to_the_hydra(self):
    self.frames(100, model=-1.0, hydra=0.5)
    out = self.frames(200, predicted=False, hydra=0.5)
    released = out.index(None)
    for a, b in zip(out[:released], out[1:released]):
      self.assertAlmostEqual(b - a, UP)
    self.assertAlmostEqual(released, 1.5 / UP, delta=2)
    assert all(o is None for o in out[released:])
    assert self.sa.state == StopAssistState.off

  def test_stops_at_once_when_not_allowed(self):
    self.frames(50)
    assert self.frames(1, allowed=False) == [None]
    assert self.sa.state == StopAssistState.off
    self.assertAlmostEqual(self.frames(1, hydra=0.3)[0], 0.3 - DOWN, msg="starts again from the Hydra's request")

  def test_below_handoff_speed_never_lets_go_by_itself(self):
    self.frames(100, model=-1.0)
    self.frames(1, v=10 * KPH)
    assert self.sa.state == StopAssistState.brakeToStop
    assert self.frames(300, predicted=False, v=5 * KPH)[-1] == -1.0
    assert self.frames(300, model=-0.2, v=0.0)[-1] == -1.0
    self.assertAlmostEqual(self.frames(300, model=-1.4, v=0.0)[-1], -1.4)
    assert self.frames(1, allowed=False) == [None]

  def test_releasing_through_handoff_speed_still_lets_go(self):
    # the stop clears at 14 km/h (a light turning green): slowing under 12 km/h while handing back mustn't latch
    self.frames(100, model=-1.0, hydra=0.3)
    out = self.frames(200, predicted=False, hydra=0.3, v=11 * KPH)
    assert self.sa.state == StopAssistState.off
    assert out[-1] is None
    for a, b in zip(out, out[1:]):
      if a is not None and b is not None:
        self.assertAlmostEqual(b - a, UP)


def make_car(stop_assist=True):
  CP = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  CP.secOcKeyAvailable = True
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.FISKER_OCEAN)
  if stop_assist:
    CP_SP.flags |= FiskerFlagsSP.STOP_ASSIST.value
  return CP, CP_SP


class TestStopAssistToggle(unittest.TestCase):
  @parameterized("value,long,on", [("0", False, False), ("1", False, True), (None, False, False), ("1", True, False)])
  def test_toggle_sets_the_flags(self, value, long, on):
    from opendbc.sunnypilot.car.interfaces import _initialize_fisker
    CP, CP_SP = make_car(stop_assist=False)
    CP.openpilotLongitudinalControl = long
    _initialize_fisker(CP, CP_SP, {} if value is None else {"FiskerStopAssist": value})
    assert bool(CP_SP.flags & FiskerFlagsSP.STOP_ASSIST) == on
    assert bool(CP.safetyConfigs[0].safetyParam & FiskerSafetyFlags.STOP_ASSIST) == on


class TestHydraAccelParsing(unittest.TestCase):
  def setUp(self):
    CP, CP_SP = make_car()
    self.CS = CarState(CP, CP_SP)
    self.parsers = self.CS.get_can_parsers(CP, CP_SP)
    self.packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])

  def tick(self, *frames):
    msgs = [self.packer.make_can_msg("ADAS_0x121", CANBUS.cam, {
      "ADAS_121_SSecOC_Fresh_Byte0": (wire << 2) | rflag, "ADAS_121_AliveCounter": alive,
      "ADAS_LgtCtrl_AccelVld": vld, "ADAS_LgtCtrl_AccelReq": accel}) for wire, rflag, alive, vld, accel in frames]
    for cp in self.parsers.values():
      cp.update([(0, msgs)])
    self.CS.update(self.parsers)
    return self.CS.hydra_121

  def test_request_and_counters(self):
    h = self.tick((62, 1, 3, 1, 0.75))
    assert (h.full_ctr, h.reset_flag, h.alive, h.valid) == (62, 1, 3, True)
    self.assertAlmostEqual(h.accel, 0.75, places=3)
    h = self.tick((63, 1, 4, 1, 0.7), (0, 1, 5, 1, 0.65))
    assert (h.full_ctr, h.alive) == (64, 5), "a 6-bit wrap within the window"
    h = self.tick((1, 2, 6, 1, 0.6))
    assert h.full_ctr == 1, "the window restarts with the reset flag"

  def test_validity(self):
    assert not self.tick((5, 0, 0, 0, 0.0)).valid
    assert self.tick((6, 0, 1, 1, 0.0)).valid
    for _ in range(4):
      assert self.tick().valid
    assert not self.tick().valid, "stale after 50 ms without a frame"

  def test_not_parsed_without_the_toggle(self):
    CP, CP_SP = make_car(stop_assist=False)
    assert "ADAS_0x121" not in CarState(CP, CP_SP).get_can_parsers(CP, CP_SP)[Bus.cam].vl


class TestStopAssistCandidates(unittest.TestCase):
  def run_controller(self, stop_assist=True, active=True, acc_state=3, gas=False, enabled=True):
    CP, CP_SP = make_car(stop_assist)
    controller = CarController(DBC[CAR.FISKER_OCEAN], CP, CP_SP)
    controller.secoc_key_verified = True
    controller.secoc_key = bytes(16)
    CS = CarState(CP, CP_SP)
    CS.out = structs.CarState(vEgo=50 * KPH, gasPressed=gas)
    CS.out.cruiseState.enabled = True
    CS.acc_state = acc_state
    CS.secoc_reset = 5
    h = CS.hydra_121
    h.accel, h.valid, h.full_ctr, h.alive, h.reset_flag = 0.5, True, 40, 7, 1
    cc_sp = structs.CarControlSP()
    cc_sp.stopAssist.active = active
    cc_sp.stopAssist.accel = -1.0
    _, sends = controller.update(structs.CarControl(enabled=enabled).as_reader(), cc_sp, CS, 0)
    return CS, [(data, bus) for addr, data, bus in sends if addr == ACCEL_CANDIDATE_ADDR]

  def test_candidates_around_the_hydra_counters(self):
    CS, cands = self.run_controller()
    assert CS.stop_assist_state == StopAssistState.slowing
    assert len(cands) == 11
    for k, (data, bus) in zip(range(-1, 10), cands, strict=True):
      assert bus == LAT_CANDIDATE_BUS
      assert data[4] == (((40 + k) & 0x3F) << 2) | 1
      assert data[1] == (1 << 4) | ((7 + k) % 15), "AccelVld Valid + the alive counter"
      assert data[0] == fisker_plain_checksum(0x121, data)
      self.assertAlmostEqual(accel_of(data), 0.5 - DOWN, places=3)

  @parameterized("kw", [{"stop_assist": False}, {"active": False}, {"acc_state": 4}, {"gas": True}, {"enabled": False}])
  def test_no_candidates(self, kw):
    CS, cands = self.run_controller(**kw)
    assert cands == []
    assert CS.stop_assist_state == StopAssistState.off

  def test_alive_patch_matches_packing(self):
    fcan = FiskerCAN(None, CANPacker("fisker_ocean_adas"))
    for accel in (-1.5, -0.3, 0.0, 1.2):
      base = fcan.create_accel_command(accel, True, 0)
      for alive in range(15):
        assert fcan.accel_command_with_alive(base, alive) == fcan.create_accel_command(accel, True, alive)


if __name__ == "__main__":
  unittest.main()
