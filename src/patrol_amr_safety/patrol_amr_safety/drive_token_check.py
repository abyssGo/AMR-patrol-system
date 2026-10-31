"""
Shared DriveToken validity rule for mission_supervisor and local_safety_supervisor.

Both nodes must agree on when this robot may drive, so the rule lives in one
place and has no ROS dependency (unit-testable without a running graph).

A frame grants driving only when all hold:
- holder_robot_id is this robot and the token string is not empty
  (an empty token is the explicit revocation defined in patrol_interfaces),
- the sequence moves forward while a grant is live (an out-of-order frame is ignored),
- the lease has not run out, measured from local receive time
  (receive time is used because the control PC and AMR PC clocks are not synced).
"""

from dataclasses import dataclass

# patrol_control publishes DriveToken BEST_EFFORT / VOLATILE / KEEP_LAST(3)
# (patrol_control_node._token_qos); subscribers must not ask for RELIABLE.
TOKEN_QOS_DEPTH = 3


def token_qos():
    """Return a QoS compatible with the control node's DriveToken publisher."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=TOKEN_QOS_DEPTH)


@dataclass(frozen=True)
class TokenFrame:
    """The fields of patrol_interfaces/DriveToken this rule needs."""

    token: str
    holder_robot_id: str
    lease_sec: float
    sequence: int


class DriveTokenCheck:
    """Track the latest DriveToken frame and answer whether driving is allowed."""

    def __init__(self, robot_id: str):
        if not robot_id:
            raise ValueError('robot_id must not be empty')
        self.robot_id = robot_id
        self.token = ''
        self.sequence = 0
        self.accepted_sequence = 0
        self._expires_at = None

    def observe(self, frame: TokenFrame, now: float) -> bool:
        """Apply one frame received at monotonic time `now`; return True if applied."""
        if frame.holder_robot_id != self.robot_id:
            return False
        if frame.sequence <= self.sequence and self.valid(now):
            # Out-of-order frame during a live grant. After the grant lapses a lower
            # sequence is accepted again, so a restarted control node is not locked out.
            return False
        self.sequence = frame.sequence
        if not frame.token or frame.lease_sec <= 0.0:
            self.token = ''
            self._expires_at = None
            return True
        self.token = frame.token
        self.accepted_sequence = frame.sequence
        self._expires_at = now + frame.lease_sec
        return True

    def valid(self, now: float) -> bool:
        """Return True while a non-empty token's lease is still running."""
        return bool(self.token) and self._expires_at is not None and now < self._expires_at

    def revoke(self) -> None:
        """Forget the current grant (used on shutdown or goal end)."""
        self.token = ''
        self._expires_at = None
