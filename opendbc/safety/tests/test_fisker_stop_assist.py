#!/usr/bin/env python3
"""
Panda side of the Fisker stop assist: the car's own ACC (the Hydra) drives, and sunnypilot's
0x121 candidates (0x521 on bus 1) may replace the Hydra's 0x121 on its way to the VCU only to
ask for less acceleration, within a floor and a per-frame step, while ACC is Active.
"""
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py

LONG_CONTROL = 1
STOP_ASSIST = 4
FRAME_US = 10_000


def raw(accel):
  return round((accel + 16) / 0.0004882)


def accel_frame(ctr, accel, vld=1, rflag=0, tag=0):
  """An 0x121-shaped payload. byte 1 = AccelVld (bits 5-4) + alive; bytes 2-3 = AccelReq;
  byte 4 = SecOC wire counter + reset flag; byte 5 is a marker to tell whose payload went out."""
  r = raw(accel)
  d = [0] * 8
  d[1] = ((vld & 0x3) << 4) | (ctr % 15)
  d[2], d[3] = (r >> 8) & 0xFF, r & 0xFF
  d[4] = (((ctr & 0x3F) << 2) | (rflag & 3)) & 0xFF
  d[5] = tag & 0xFF
  return bytes(d)


class TestFiskerStopAssist(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self._init(STOP_ASSIST)

  def _init(self, param, acc_state=3):
    self.t = 1_000_000
    self.ctr = 10
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, param)
    self.safety.init_tests()
    self.safety.set_timer(self.t)   # after init_tests, which zeroes it
    dat = bytearray(8)
    dat[4] = acc_state & 0x0F   # ADAS_Sts_ACC_ICC
    self.safety.safety_rx_hook(common.make_msg(2, 0x313, dat=bytes(dat)))

  def _cand(self, accel, ctr=None, **kw):
    ctr = self.ctr if ctr is None else ctr
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x521, 1, accel_frame(ctr, accel, tag=0xC0, **kw)))

  def _hydra(self, accel, **kw):
    """The Hydra's frame for the current counter on its way to the VCU: (emitted, accel the VCU gets, ours)."""
    pkt = libsafety_py.make_CANPacket(0x121, 2, accel_frame(self.ctr, accel, tag=0xAA, **kw))
    emit = self.safety.safety_fwd_modify_hook(pkt)
    got = ((pkt.data[2] << 8) | pkt.data[3]) * 0.0004882 - 16
    return emit, round(got, 2), pkt.data[5] == 0xC0

  def _next(self):
    self.ctr += 1
    self.t += FRAME_US
    self.safety.set_timer(self.t)

  def _step(self, hydra, cand, **kw):
    """One 10 ms cycle: our candidate for this counter (unless None), then the Hydra's frame."""
    if cand is not None:
      self._cand(cand)
    result = self._hydra(hydra, **kw)
    self._next()
    return result

  def _ramp_to(self, hydra, target):
    """The Hydra's frames at a steady request, then ours ramping down from it at 0.05 per frame to target."""
    self._step(hydra, None)
    accel = hydra
    while accel > target:
      accel = max(target, accel - 0.05)
      emit, got, ours = self._step(hydra, accel)
      self.assertTrue(emit and ours)
      self.assertAlmostEqual(got, accel, places=2)

  def test_candidates_never_transmitted(self):
    self.assertFalse(self._cand(-1.0))

  def test_candidates_rejected_without_flag(self):
    for param in (0, LONG_CONTROL, LONG_CONTROL | STOP_ASSIST):
      with self.subTest(param=param):
        self._init(param)
        self._step(1.0, None)
        emit, got, ours = self._step(1.0, 0.95)
        self.assertTrue(emit)
        self.assertFalse(ours)
        self.assertEqual(got, 1.0)

  def test_lowers_hydra_request(self):
    self._ramp_to(1.0, -1.5)
    emit, got, ours = self._step(1.5, -1.5)
    self.assertTrue(emit and ours)
    self.assertEqual(got, -1.5)

  def test_first_frame_never_replaced(self):
    emit, got, ours = self._step(1.0, 0.95)
    self.assertTrue(emit)
    self.assertFalse(ours, "nothing to step from before the Hydra's first frame")

  def test_never_raises(self):
    for hydra, cand in ((0.5, 0.5), (0.5, 0.55), (-1.0, -0.95)):
      with self.subTest(hydra=hydra, cand=cand):
        self._init(STOP_ASSIST)
        self._step(hydra, None)
        emit, got, ours = self._step(hydra, cand)
        self.assertTrue(emit)
        self.assertFalse(ours)
        self.assertEqual(got, hydra)

  def test_floor(self):
    self._ramp_to(-1.9, -1.95)
    emit, got, ours = self._step(-1.9, -2.03)
    self.assertTrue(emit)
    self.assertFalse(ours, "below -2.0 m/s^2 the Hydra's own frame goes through")
    self.assertEqual(got, -1.9)

  def test_step_limit(self):
    self._step(1.0, None)
    self.assertFalse(self._step(1.0, 0.85)[2], "more than 0.1 m/s^2 below the last value in one frame is a jump")
    self.assertTrue(self._step(1.0, 0.92)[2])
    self.assertFalse(self._step(1.0, 0.8)[2])
    self.assertTrue(self._step(1.0, 0.84)[2], "steps on from our last value, not from the Hydra's frame in between")

  def test_holds_while_hydra_rises(self):
    self._ramp_to(0.0, -1.0)
    for hydra in (0.5, 1.0, 1.5):
      emit, got, ours = self._step(hydra, -1.0)
      self.assertTrue(ours)
      self.assertEqual(got, -1.0)

  def test_hydra_braking_harder(self):
    self._ramp_to(0.0, -1.0)
    emit, got, ours = self._step(-1.8, -1.0)
    self.assertFalse(ours)
    self.assertEqual(got, -1.8)
    self.assertTrue(self._step(-0.5, -1.0)[2], "back in under the Hydra once it lets go")

  def test_missed_frames_resume(self):
    self._ramp_to(1.0, -1.0)
    for _ in range(5):
      emit, got, ours = self._step(1.0, None)
      self.assertTrue(emit)
      self.assertEqual(got, 1.0)
    self.assertTrue(self._step(1.0, -1.0)[2], "a short miss resumes at the same level")

  def test_long_miss_must_ramp_again(self):
    self._ramp_to(1.0, -1.0)
    for _ in range(26):
      self._step(1.0, None)
    self.assertFalse(self._step(1.0, -1.0)[2])
    self.assertTrue(self._step(1.0, 0.95)[2])

  def test_only_while_acc_active(self):
    for state in (0, 2, 4, 5, 6, 9, 11):
      with self.subTest(state=state):
        self._init(STOP_ASSIST, acc_state=state)
        self._step(1.0, None)
        self.assertFalse(self._step(1.0, 0.95)[2])

  def test_not_without_controls_allowed(self):
    self._step(1.0, None)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._step(1.0, 0.95)[2])

  def test_both_frames_valid(self):
    self._step(1.0, None)
    self.assertFalse(self._step(1.0, 0.95, vld=0)[2], "the Hydra's request isn't valid")
    self._cand(0.95, vld=0)
    self.assertFalse(self._hydra(1.0)[2], "our candidate isn't valid")

  def test_counters_must_match(self):
    self._step(1.0, None)
    self._cand(0.95, ctr=self.ctr + 1)
    emit, got, ours = self._hydra(1.0)
    self.assertTrue(emit, "no candidate for this counter: the Hydra's frame, never a gap")
    self.assertFalse(ours)
    self._cand(0.95, rflag=1)
    self.assertFalse(self._hydra(1.0)[2])

  def test_stale_candidate(self):
    self._step(1.0, None)
    self._cand(0.95)
    self.safety.set_timer(self.t + 70_000)
    self.assertFalse(self._hydra(1.0)[2])

  def test_steering_substitution_unaffected(self):
    # an 0x121 candidate doesn't arm the 0x1D0 substitution
    self._cand(0.95)
    pkt = libsafety_py.make_CANPacket(0x1D0, 2, accel_frame(self.ctr, 0.0, tag=0xAA))
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    self.assertEqual(pkt.data[5], 0xAA)


if __name__ == "__main__":
  unittest.main()
