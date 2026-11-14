"""
The next turn on the head unit's guided route, from the ADASIS v2 horizon the ICC sends the ADAS module.

Through the GW, the ICC describes the road ahead on the ADAS bus, each message at 10 Hz:
  ICC_0x361 position  the path the car is on and its offset along it (m)
  ICC_0x250 segments  the road pieces ahead; ICC_SegPartOfCalcRoute is 1 while a guided route is active
  ICC_0x251 stubs     every branch at each intersection ahead: its offset on the path, its angle, and
                      whether the route takes it (ICC_StubPartOfCalcRout)
Each stub is sent once, as a few seconds of repeats when its intersection enters the ~1 km horizon, so stubs
are kept until the car has passed them. Offsets are 13 bits and wrap every 8192 m. On a guided route the
car's own path is the route, and the turn angle of the stub the route takes gives the direction, clockwise
from straight: about 90 deg is right, 270 left. On route 00000014 (2026-11-14) both guided right turns
read 91 and 96 deg, and the car started turning 10 to 17 m before the stub's offset.
"""
import math
from dataclasses import dataclass

from opendbc.car import DT_CTRL, structs

RouteTurn = structs.CarStateSP.RouteTurn
Direction = RouteTurn.Direction

OFFSET_WRAP = 8192
OFFSET_INVALID = 8191
FIRST_PATH = 8              # path indexes 0-7 are reserved
ROUTE_TIMEOUT = 2.0         # s without a route segment before the route counts as ended
POSITION_TIMEOUT = 1.0      # s without a position before no turn is reported
STUB_MAX_AGE = 900.0        # s a stub is kept: the horizon is ~1 km, minutes in town traffic
STUB_BEHIND = 50.0          # m past a stub before it's forgotten
TURN_PASSED = 15.0          # m past a turn point it's still reported: about 85 deg into a right turn
RIGHT_TURN = (30.0, 150.0)  # deg clockwise from straight
LEFT_TURN = (210.0, 330.0)


def turn_direction(angle: float) -> Direction:
  if RIGHT_TURN[0] <= angle <= RIGHT_TURN[1]:
    return Direction.turnRight
  if LEFT_TURN[0] <= angle <= LEFT_TURN[1]:
    return Direction.turnLeft
  return Direction.none  # straight on, a fork or a U-turn


def signed_distance(target: int, origin: int) -> int:
  """target - origin along a path, through the 8192 m wrap: negative once target is behind."""
  d = (target - origin) % OFFSET_WRAP
  return d - OFFSET_WRAP if d >= OFFSET_WRAP // 2 else d


@dataclass
class _Stub:
  angle: float
  on_route: bool
  seen: float


class RouteTurnTracker:
  def __init__(self):
    self.t = 0.0
    self.route_seen = -math.inf
    self.position: tuple[int, int] | None = None  # (path, offset)
    self.position_seen = -math.inf
    self.stubs: dict[tuple[int, int, int], _Stub] = {}  # (path, offset, sub-path) -> stub

  def update(self, segments: list[dict], stubs: list[dict], positions: list[dict], dt: float = DT_CTRL) -> None:
    """Feed every frame received since the last call (carstate's get_new_frames)."""
    self.t += dt
    if any(int(s["ICC_SegPartOfCalcRoute"]) == 1 for s in segments):
      self.route_seen = self.t
    for s in stubs:
      path, offset = int(s["ICC_StubPathIdx"]), int(s["ICC_StubOffset"])
      if path >= FIRST_PATH and offset < OFFSET_INVALID:
        self.stubs[(path, offset, int(s["ICC_StubStubPathIdx"]))] = _Stub(float(s["ICC_StubTurnAngl"]),
                                                                         int(s["ICC_StubPartOfCalcRout"]) == 1, self.t)
    for p in positions:
      path, offset = int(p["ICC_PosnPathldx"]), int(p["ICC_PosnOffset"])
      if path >= FIRST_PATH and offset < OFFSET_INVALID:
        self.position, self.position_seen = (path, offset), self.t
    if stubs or positions:
      self._prune()

  def _prune(self) -> None:
    def keep(key, stub):
      if self.t - stub.seen > STUB_MAX_AGE:
        return False
      return self.position is None or key[0] != self.position[0] or signed_distance(key[1], self.position[1]) >= -STUB_BEHIND
    self.stubs = {k: s for k, s in self.stubs.items() if keep(k, s)}

  def next_turn(self) -> RouteTurn:
    """The nearest left or right turn the route takes ahead (or just passed), else direction none."""
    if self.position is None or self.t - self.position_seen > POSITION_TIMEOUT or self.t - self.route_seen > ROUTE_TIMEOUT:
      return RouteTurn()
    path, offset = self.position
    best = None
    for (stub_path, stub_offset, _), stub in self.stubs.items():
      if stub_path != path or not stub.on_route:
        continue
      direction = turn_direction(stub.angle)
      distance = signed_distance(stub_offset, offset)
      if direction != Direction.none and distance >= -TURN_PASSED and (best is None or distance < best.distance):
        best = RouteTurn(direction=direction, distance=float(distance))
    return best or RouteTurn()
