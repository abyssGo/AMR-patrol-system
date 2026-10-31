"""
EStop state rule shared by local_safety_supervisor and mission_supervisor (CR 9/8).

`/control/estop` has a single publisher (control Safety Arbiter) that republishes
every target's state once per second (targets: robot1, robot6, all).

- active=true is applied immediately, whatever its sequence (fail-safe).
- active=false is applied only when its sequence is newer than the last frame
  applied for that target, so an old release can never clear a newer stop.
- No frame for STALE_AFTER seconds means the arbiter is unreachable: the wheels
  are blocked until frames arrive again. A stale link alone does not latch;
  the DriveToken lease decides whether the mission may continue.

Releasing the EStop never restarts motion by itself. mission_supervisor also
requires a valid DriveToken and a separate operator resume command.

No ROS dependency, so the rule is unit-testable without a running graph.
"""

from dataclasses import dataclass

ESTOP_TOPIC = '/control/estop'
ESTOP_ALL = 'all'
STALE_AFTER = 3.0   # s, three missed 1 Hz republishes


def estop_qos():
    """
    Return a subscriber QoS compatible with RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1).

    Depth 10 so frames for different targets (all, robot1, robot6) are not
    overwritten before the callback runs.
    """
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=10)


@dataclass(frozen=True)
class EStopFrame:
    """The fields of patrol_interfaces/EStop this rule needs."""

    target_robot_id: str
    active: bool
    reason: int
    sequence: int


class EStopCheck:
    """Track /control/estop frames for one robot."""

    def __init__(self, robot_id: str):
        if not robot_id:
            raise ValueError('robot_id must not be empty')
        self.robot_id = robot_id
        self._state = {robot_id: (False, 0), ESTOP_ALL: (False, 0)}
        self._sequence = {robot_id: None, ESTOP_ALL: None}
        self._last_rx = None

    def observe(self, frame: EStopFrame, now: float) -> bool:
        """Apply one frame received at monotonic time `now`; True if it changed state."""
        if frame.target_robot_id not in self._state:
            # 다른 로봇 대상이어도 Safety Arbiter가 살아 있다는 증거다.
            self._last_rx = now
            return False
        self._last_rx = now
        target = frame.target_robot_id
        last = self._sequence[target]
        if not frame.active and last is not None and frame.sequence <= last:
            return False
        before = self._state[target]
        if last is None or frame.sequence > last:
            self._sequence[target] = frame.sequence
        self._state[target] = (bool(frame.active), int(frame.reason))
        return before != self._state[target]

    def active(self) -> bool:
        """Return True while an EStop for this robot or for all robots is active."""
        return self._state[self.robot_id][0] or self._state[ESTOP_ALL][0]

    def reason(self) -> int:
        for target in (self.robot_id, ESTOP_ALL):
            if self._state[target][0]:
                return self._state[target][1]
        return 0

    def stale(self, now: float) -> bool:
        """Return True when no frame arrived within STALE_AFTER (or none ever)."""
        return self._last_rx is None or now - self._last_rx > STALE_AFTER

    def blocks_motion(self, now: float) -> bool:
        return self.active() or self.stale(now)
