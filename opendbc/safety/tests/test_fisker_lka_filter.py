#!/usr/bin/env python3
"""
Panda-side filter for the amber LKA telltale (ADAS_0x31A, bus 2 -> cluster).

openpilot's ICC_0x52A copy sets ICC_LKASetting to Warning_only so the ADAS never steers. The ADAS
then requests the amber LKA telltale (ADAS_ELKA_TelltaleReq 2). fisker_fwd_modify clears that
request only while the ADAS is getting our Warning_only copy and its LKA reports no error.
"""

import unittest
from opendbc.can import CANPacker, CANParser
from opendbc.car.structs import CarParams
from opendbc.car.fisker.fiskercan import fisker_plain_checksum
from opendbc.safety.tests.libsafety import libsafety_py

DBC = "fisker_ocean_adas"
LKA_OFF, LKA_WARNING_ONLY, LKA_WARNING_AND_STEERING = 0, 1, 2
LKA_STS_INACTIVE, LKA_STS_ERROR = 2, 8
TELLTALE_DEGRADED = 2
OTHER_TELLTALES = {"ADAS_BSM_ELKA_TelltaleReq": 3, "ADAS_ESA_TelltaleReq": 1, "ADAS_FACM_TelltaleReq": 2}


class TestFiskerLkaTelltaleFilter(unittest.TestCase):
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

  def _send_icc_copy(self, lka):
    _, dat, _ = self.packer.make_can_msg("ICC_0x52A", 2, {"ICC_LKASetting": lka, "ICCACCSwt": 1})
    self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x52A, 2, dat)))

  def _lka_status(self, sts):
    _, dat, _ = self.packer.make_can_msg("ADAS_0x314", 2, {"ADAS_LKASts": sts, "ADAS_BSDSts": 2})
    self.assertTrue(self.safety.safety_fwd_modify_hook(libsafety_py.make_CANPacket(0x314, 2, dat)))

  def _telltale(self, value, bus=2):
    """Forward one ADAS_0x31A and return the ELKA telltale the cluster gets."""
    _, dat, _ = self.packer.make_can_msg("ADAS_0x31A", bus, {"ADAS_ELKA_TelltaleReq": value, "ADAS_31A_AliveCounter": 7, **OTHER_TELLTALES})
    dat = bytearray(dat)
    dat[0] = fisker_plain_checksum(0x31A, bytes(dat))
    pkt = libsafety_py.make_CANPacket(0x31A, bus, bytes(dat))
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    out = bytes(pkt.data[0:8])
    self.assertEqual(out[0], fisker_plain_checksum(0x31A, out), "CheckSum must match what's sent")
    cp = CANParser(DBC, [("ADAS_0x31A", 0)], 0)
    cp.update([(0, [(0x31A, out, 0)])])
    sig = cp.vl["ADAS_0x31A"]
    self.assertEqual(sig["ADAS_31A_AliveCounter"], 7)
    for k, v in OTHER_TELLTALES.items():
      self.assertEqual(sig[k], v, k)
    if bus != 2:
      self.assertEqual(out, bytes(dat))
    return int(sig["ADAS_ELKA_TelltaleReq"])

  def test_forced_warning_only_telltale_hidden(self):
    self._send_icc_copy(LKA_WARNING_ONLY)
    self._lka_status(LKA_STS_INACTIVE)
    self.assertEqual(self._telltale(TELLTALE_DEGRADED), 0)

  def test_real_lka_error_shows(self):
    self._send_icc_copy(LKA_WARNING_ONLY)
    self._lka_status(LKA_STS_ERROR)
    self.assertEqual(self._telltale(TELLTALE_DEGRADED), TELLTALE_DEGRADED)

  def test_other_lka_settings_untouched(self):
    for lka in (LKA_OFF, LKA_WARNING_AND_STEERING):
      with self.subTest(lka=lka):
        self.setUp()
        self._send_icc_copy(lka)
        self._lka_status(LKA_STS_INACTIVE)
        self.assertEqual(self._telltale(TELLTALE_DEGRADED), TELLTALE_DEGRADED)

  def test_no_copy_since_init(self):
    self._lka_status(LKA_STS_INACTIVE)
    self.assertEqual(self._telltale(TELLTALE_DEGRADED), TELLTALE_DEGRADED)

  def test_shows_again_once_copies_stop(self):
    self._send_icc_copy(LKA_WARNING_ONLY)
    self._lka_status(LKA_STS_INACTIVE)
    self._advance(400_000)
    self.assertEqual(self._telltale(TELLTALE_DEGRADED), 0)
    self._advance(200_000)
    self.assertEqual(self._telltale(TELLTALE_DEGRADED), TELLTALE_DEGRADED)

  def test_other_telltale_values_untouched(self):
    self._send_icc_copy(LKA_WARNING_ONLY)
    self._lka_status(LKA_STS_INACTIVE)
    for value in (0, 1, 3):
      self.assertEqual(self._telltale(value), value)

  def test_vehicle_side_ignored(self):
    self._send_icc_copy(LKA_WARNING_ONLY)
    self._lka_status(LKA_STS_INACTIVE)
    self.assertEqual(self._telltale(TELLTALE_DEGRADED, bus=0), TELLTALE_DEGRADED)


if __name__ == "__main__":
  unittest.main()
