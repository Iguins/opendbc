import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.fisker.adasis import (POSITION_TIMEOUT, ROUTE_TIMEOUT, TURN_PASSED, Direction, RouteTurnTracker,
                                       signed_distance)
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.values import CANBUS, CAR, DBC
from opendbc.testing import parameterized


def segment(on_route=1):
  return {"ICC_SegPartOfCalcRoute": on_route}


def stub(offset, angle, on_route=1, path=14, sub=6):
  return {"ICC_StubPathIdx": path, "ICC_StubOffset": offset, "ICC_StubStubPathIdx": sub,
          "ICC_StubTurnAngl": angle, "ICC_StubPartOfCalcRout": on_route}


def position(offset, path=14):
  return {"ICC_PosnPathldx": path, "ICC_PosnOffset": offset}


def tracker_at(offset, stubs, on_route=1):
  t = RouteTurnTracker()
  t.update([segment(on_route)], stubs, [position(offset)])
  return t


class TestRouteTurn(unittest.TestCase):
  # route 00000014: the route branch read 90.7 deg at a right turn, the other road 276.3 deg
  @parameterized('angle, direction', [(90.7, Direction.turnRight), (96.4, Direction.turnRight), (270.6, Direction.turnLeft),
                                      (0.0, Direction.none), (11.3, Direction.none), (348.6, Direction.none),
                                      (168.6, Direction.none)])
  def test_direction_from_route_branch(self, angle, direction):
    turn = tracker_at(2160, [stub(2197, angle), stub(2197, 276.3, on_route=0, sub=5)]).next_turn()
    assert turn.direction == direction
    if direction != Direction.none:
      assert turn.distance == 37

  def test_turn_reported_until_just_past(self):
    t = tracker_at(2160, [stub(2197, 90.7), stub(2400, 270.6)])
    for offset, expected in ((2190, (Direction.turnRight, 7)), (2197 + TURN_PASSED, (Direction.turnRight, -TURN_PASSED)),
                             (2197 + TURN_PASSED + 1, (Direction.turnLeft, 400 - 213))):
      t.update([segment()], [], [position(offset)])
      assert (t.next_turn().direction, t.next_turn().distance) == expected

  def test_offsets_wrap(self):
    assert signed_distance(20, 8150) == 62
    assert signed_distance(8150, 20) == -62
    assert tracker_at(8150, [stub(20, 90.7)]).next_turn().distance == 62

  def test_other_paths_and_off_route_branches_ignored(self):
    assert tracker_at(100, [stub(140, 90.7, path=15), stub(150, 270.6, on_route=0)]).next_turn().direction == Direction.none

  def test_nothing_without_route_or_position(self):
    t = tracker_at(100, [stub(140, 90.7)], on_route=0)
    assert t.next_turn().direction == Direction.none

    t = tracker_at(100, [stub(140, 90.7)])
    for _ in range(round(ROUTE_TIMEOUT / 0.01) + 1):
      t.update([segment(0)], [], [position(100)])
    assert t.next_turn().direction == Direction.none  # arrived or route cancelled

    t = tracker_at(100, [stub(140, 90.7)])
    for _ in range(round(POSITION_TIMEOUT / 0.01) + 1):
      t.update([segment()], [], [])
    assert t.next_turn().direction == Direction.none

  def test_passed_stubs_forgotten(self):
    t = tracker_at(100, [stub(140, 90.7)])
    t.update([segment()], [], [position(200)])
    assert t.stubs == {}

  def test_carstate_reports_route_turn(self):
    cp = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
    sp = CarInterface.get_non_essential_params_sp(cp, CAR.FISKER_OCEAN)
    cs = CarState(cp, sp)
    parsers = cs.get_can_parsers(cp, sp)
    packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])
    frames = [
      packer.make_can_msg("ICC_0x250", CANBUS.pt, {"ICC_SegMsgType": 2, "ICC_SegPathIdx": 14, "ICC_SegPartOfCalcRoute": 1}),
      packer.make_can_msg("ICC_0x251", CANBUS.pt, {"ICC_StubMsgType": 3, "ICC_StubPathIdx": 14, "ICC_StubOffset": 2197,
                                                   "ICC_StubStubPathIdx": 6, "ICC_StubTurnAngl": 90.7,
                                                   "ICC_StubPartOfCalcRout": 1}),
      packer.make_can_msg("ICC_0x361", CANBUS.pt, {"ICC_PosnMsgTye": 1, "ICC_PosnPathldx": 14, "ICC_PosnOffset": 2170}),
    ]
    for parser in parsers.values():
      parser.update([(0, frames)])
    _, ret_sp = cs.update(parsers)
    assert ret_sp.routeTurn == structs.CarStateSP.RouteTurn(direction=Direction.turnRight, distance=27.0)
