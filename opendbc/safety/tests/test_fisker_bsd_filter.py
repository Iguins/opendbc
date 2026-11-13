#!/usr/bin/env python3
"""
Panda-side filter for the blind-spot dropout warnings (ADAS_0x317, bus 2 -> cluster).

A rear corner radar drops out for ~2 s at a time, taking BSD/DOW/LCA down with it (ADAS_0x314).
Each time the ADAS lights the yellow ADAS fault telltale for the dropout and shows "BSM/ELKA
system fault" (text 13) with the fault chime, the text staying ~3 s after it. fisker_fwd_modify
hides those while the dropout is transient, shows them again once it persists, and never touches
ADAS_0x314 or any other warning.
"""

import unittest
from opendbc.can import CANPacker, CANParser
from opendbc.car.structs import CarParams
from opendbc.car.fisker.fiskercan import fisker_plain_checksum
from opendbc.safety.tests.libsafety import libsafety_py

DBC = "fisker_ocean_adas"
BSD_TEXT, LKA_TEXT, ACC_TEXT = 13, 6, 7     # ADAS_IDS_WarnTxtReq
FAULT_CHIME = 3
TRANSIENT_US = 20_000_000
UNTOUCHED = {"ADAS_SysFltTyp": 15, "ADAS_IDS_LeWarnSrc": 1, "ADAS_IDS_RiWarnReq": 2}


class TestFiskerBsdFilter(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()
    self.packer = CANPacker(DBC)
    self.t = 1_000_000
    self.safety.set_timer(self.t)

  def _advance(self, us):
    self.t += us
    self.safety.set_timer(self.t)

  def _status(self, bsd=2, dow=2, lca=0, bus=2):
    """Forward one ADAS_0x314; it must reach the car unchanged."""
    _, dat, _ = self.packer.make_can_msg("ADAS_0x314", 2, {"ADAS_BSDSts": bsd, "ADAS_DOW_Sts": dow, "ADAS_LCA_Sts": lca, "ADAS_LKASts": 3})
    pkt = libsafety_py.make_CANPacket(0x314, bus, dat)
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    self.assertEqual(bytes(pkt.data[0:8]), bytes(dat), "0x314 is only read, never changed")

  def _warn(self, text=0, telltale=0, chime=0, bus=2):
    """Forward one ADAS_0x317 and return (text, telltale, chime) as the cluster gets it."""
    values = {"ADAS_IDS_WarnTxtReq": text, "ADAS_SysFltWarnReq": telltale, "ADAS_ChimeReq": chime,
              "ADAS_317_AliveCounter": 9, **UNTOUCHED}
    _, dat, _ = self.packer.make_can_msg("ADAS_0x317", bus, values)
    dat = bytearray(dat)
    dat[0] = fisker_plain_checksum(0x317, bytes(dat))
    pkt = libsafety_py.make_CANPacket(0x317, bus, bytes(dat))
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    out = bytes(pkt.data[0:8])
    self.assertEqual(out[0], fisker_plain_checksum(0x317, out), "CheckSum must match what's sent")
    cp = CANParser(DBC, [("ADAS_0x317", 0)], 0)
    cp.update([(0, [(0x317, out, 0)])])
    sig = cp.vl["ADAS_0x317"]
    self.assertEqual(sig["ADAS_317_AliveCounter"], 9)
    for k, v in UNTOUCHED.items():
      self.assertEqual(sig[k], v, k)
    if bus != 2:
      self.assertEqual(out, bytes(dat))
    return int(sig["ADAS_IDS_WarnTxtReq"]), int(sig["ADAS_SysFltWarnReq"]), int(sig["ADAS_ChimeReq"])

  def _dropout(self, us, **status):
    """Hold a dropout for `us` with 0x314 at 50 Hz, then recover."""
    for _ in range(us // 20_000):
      self._status(**({"bsd": 4, "dow": 4, "lca": 7} | status))
      self._advance(20_000)
    self._status()

  def test_untouched_without_dropout(self):
    self._status()
    self.assertEqual(self._warn(text=BSD_TEXT, telltale=1, chime=FAULT_CHIME), (BSD_TEXT, 1, FAULT_CHIME))

  def test_dropout_warnings_hidden(self):
    for status in ({"bsd": 4}, {"dow": 4}, {"lca": 7}):
      with self.subTest(**status):
        self.setUp()
        self._status(**status)
        self.assertEqual(self._warn(telltale=1), (0, 0, 0))
        self.assertEqual(self._warn(text=BSD_TEXT, telltale=1, chime=FAULT_CHIME), (0, 0, 0))
        self.assertEqual(self._warn(text=BSD_TEXT, telltale=1), (0, 0, 0))

  def test_text_hidden_until_it_clears(self):
    self._status(bsd=4)
    self.assertEqual(self._warn(text=BSD_TEXT, telltale=1, chime=FAULT_CHIME), (0, 0, 0))
    self._dropout(2_000_000)
    # the text stays up ~3 s after the dropout: hidden all along
    for _ in range(150):
      self.assertEqual(self._warn(text=BSD_TEXT), (0, 0, 0))
      self._advance(20_000)
    self.assertEqual(self._warn(), (0, 0, 0))
    # the telltale goes with the dropout: one lit 3 s after it shows
    self.assertEqual(self._warn(telltale=1), (0, 1, 0))
    # so does a new text after the window
    self._advance(2_000_000)
    self.assertEqual(self._warn(text=BSD_TEXT, chime=FAULT_CHIME), (BSD_TEXT, 0, FAULT_CHIME))

  def test_other_warnings_untouched(self):
    self._status(bsd=4, dow=4, lca=7)
    self.assertEqual(self._warn(text=LKA_TEXT, chime=FAULT_CHIME), (LKA_TEXT, 0, FAULT_CHIME))
    self.assertEqual(self._warn(text=ACC_TEXT), (ACC_TEXT, 0, 0))
    self.assertEqual(self._warn(telltale=2, chime=FAULT_CHIME), (0, 2, FAULT_CHIME))
    # only the fault chime goes with a hidden warning
    self.assertEqual(self._warn(text=BSD_TEXT, chime=1), (0, 0, 1))

  def test_persistent_dropout_shows(self):
    self._status(bsd=4)
    self.assertEqual(self._warn(text=BSD_TEXT, telltale=1, chime=FAULT_CHIME), (0, 0, 0))
    for _ in range(TRANSIENT_US // 20_000):
      self._advance(20_000)
      self._status(bsd=4)
    self.assertEqual(self._warn(text=BSD_TEXT, telltale=1), (BSD_TEXT, 1, 0))
    # still a real dropout after it ends: its trailing text shows
    self._status()
    self._advance(1_000_000)
    self.assertEqual(self._warn(text=BSD_TEXT), (BSD_TEXT, 0, 0))

  def test_new_transient_dropout_hidden_after_persistent_one(self):
    self._dropout(TRANSIENT_US + 100_000, bsd=4)
    self._advance(1_000_000)
    self._status(dow=4)
    self.assertEqual(self._warn(text=BSD_TEXT, telltale=1, chime=FAULT_CHIME), (0, 0, 0))

  def test_reset_on_init(self):
    self._status(bsd=4)
    self.setUp()
    self.assertEqual(self._warn(text=BSD_TEXT, telltale=1), (BSD_TEXT, 1, 0))

  def test_vehicle_side_ignored(self):
    self._status(bsd=4, bus=0)
    self.assertEqual(self._warn(text=BSD_TEXT, telltale=1), (BSD_TEXT, 1, 0))
    self._status(bsd=4)
    self.assertEqual(self._warn(text=BSD_TEXT, telltale=1, bus=0), (BSD_TEXT, 1, 0))


if __name__ == "__main__":
  unittest.main()
