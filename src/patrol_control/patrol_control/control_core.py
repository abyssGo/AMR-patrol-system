"""ROS-independent state machine for the basic patrol control flow."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Iterable
from uuid import uuid4


class TaskState(IntEnum):
    """Patrol Feedback task states."""

    WAITING_FOR_TOKEN = 1
    UNDOCKING = 2
    INITIAL_POSE_READY = 3
    PATROLLING = 4
    MOVING_TO_SAFE_ZONE = 5
    DETECTION_PROCESSING = 6
    DETECTION_CONFIRMED = 7
    RESUMING = 8
    DOCKING = 9
    BLOCKED = 10
    WAYPOINT_REACHED = 11


class EventType(IntEnum):
    """Detection event values shared by Patrol and ReportDetection."""

    FIRE = 1
    LEAK = 2
    OBSTACLE = 3


class PatrolCommandType(IntEnum):
    """Control commands that adjust an active Patrol Action."""

    MOVE_TO_SAFE_ZONE = 1
    RESUME_PATROL = 2


@dataclass(frozen=True)
class GoalIntent:
    """Validated Patrol Goal payload."""

    command_id: str
    robot_id: str


@dataclass(frozen=True)
class CommandIntent:
    """PatrolCommand payload produced by a permit transition."""

    command_id: str
    robot_id: str
    command: PatrolCommandType


@dataclass(frozen=True)
class TokenFrame:
    """One robot-specific DriveToken publication."""

    robot_id: str
    token: str
    sequence: int


@dataclass(frozen=True)
class FeedbackEffect:
    """State changes caused by one Patrol Feedback sample."""

    accepted: bool
    token_granted: bool = False
    fire_hold_activated: bool = False


class ResultOutcome(IntEnum):
    """Patrol Result outcome values."""

    SUCCEEDED = 0
    FAILED = 1
    CANCELED = 2


REASON_LOW_BATTERY = 11          # Patrol.Result: returned and docked on low battery
ESTOP_ALL = 'all'
ESTOP_REASONS = range(0, 7)      # EStop.msg: UNKNOWN .. SYSTEM_FAULT

# 출동 배터리 조건: 30% 미만이 되면 80%까지 충전된 뒤에만 다시 출동한다.
DISPATCH_MIN_BATTERY = 0.30
RECHARGED_BATTERY = 0.80


@dataclass(frozen=True)
class EStopFrame:
    """One /control/estop publication."""

    target_robot_id: str
    active: bool
    reason: int
    sequence: int


class ControlStateError(RuntimeError):
    """Raised when an operation violates the basic control state machine."""


class ControlCore:
    """Own exclusive mission, token, permit, fire-hold, rotation, and EStop state."""

    def __init__(
        self,
        robot_ids: Iterable[str],
        *,
        token_factory: Callable[[], str] | None = None,
        initial_robot_id: str | None = None,
    ) -> None:
        robots = tuple(robot_ids)
        if not robots or len(set(robots)) != len(robots):
            raise ValueError('robot_ids must be non-empty and unique')
        if any(not robot_id for robot_id in robots):
            raise ValueError('robot_ids cannot contain an empty value')
        if initial_robot_id is not None and initial_robot_id not in robots:
            raise ValueError('initial_robot_id must be in robot_ids')

        self.robot_ids = robots
        self._token_factory = token_factory or (lambda: str(uuid4()))
        self._goal_sequence = 0
        self._command_sequence = 0
        self._token_sequences = dict.fromkeys(robots, 0)

        self.patrol_allowed: bool | None = None
        self.active_robot_id: str | None = None
        self.goal_accepted = False
        self.drive_granted = False
        self.active_token = ''
        self.fire_hold = False
        self.fire_event_id = ''

        # 자동 교대 (설계도 01: 도킹 확인 → 다음 AMR에 토큰)
        self.rotation_running = False
        self._next_index = robots.index(initial_robot_id) if initial_robot_id else 0
        self.excluded: set[str] = set()
        self.charging_hold: set[str] = set()
        self.battery: dict[str, float | None] = dict.fromkeys(robots, None)
        # 관제가 모르는 임무가 남아 있는 로봇(재시작 전 임무 등): 그 임무가 끝날 때까지 출동 보류
        self.restart_wait: set[str] = set()
        # EStop이 걸렸던 로봇: 해제 뒤 운영자 재개(/control/resume) 전까지 명령 · 출동 금지
        self.estop_pending: set[str] = set()

        # EStop (Safety Arbiter): 대상별 현재 상태
        self.estop: dict[str, tuple[bool, int]] = {
            target: (False, 0) for target in (ESTOP_ALL, *robots)
        }
        self._estop_sequence = 0

    def prepare_goal(self, robot_id: str) -> GoalIntent:
        """Reserve the single active slot and create a Patrol Goal payload."""
        self._require_robot(robot_id)
        if self.active_robot_id is not None:
            raise ControlStateError(
                f'active patrol already exists for {self.active_robot_id}'
            )
        if self.patrol_allowed is not True:
            raise ControlStateError('patrol is not allowed by CCTV permit')
        if self.fire_hold:
            raise ControlStateError('fire hold is active')

        self._goal_sequence += 1
        self.active_robot_id = robot_id
        self.goal_accepted = False
        self.drive_granted = False
        self.active_token = ''
        return GoalIntent(
            command_id=f'cmd-{robot_id}-start-{self._goal_sequence:06d}',
            robot_id=robot_id,
        )

    def mark_goal_accepted(self, robot_id: str) -> None:
        """Record Action server acceptance without granting drive authority."""
        self._require_active(robot_id)
        self.goal_accepted = True

    def finish_goal(
        self,
        robot_id: str,
        outcome: int | None = None,
        reason_code: int = 0,
    ) -> str:
        """
        Release the active slot, revoke authority and apply the handover rule.

        outcome None means the goal never ran (rejected or not sent): the
        robot stays in rotation and the next robot gets the turn. Otherwise the
        result decides whether the robot docked (설계도 01 "도킹하였는가?"):
        SUCCEEDED, or FAILED with REASON_LOW_BATTERY, means docked. Any other
        failure leaves the robot out of its dock, so it is excluded from the
        rotation until the operator starts the patrol again. CANCELED stops the
        rotation because the operator took over. Returns 'docked',
        'excluded', 'canceled' or 'not_run'.
        """
        self._require_active(robot_id)
        self.active_robot_id = None
        self.goal_accepted = False
        self.drive_granted = False
        self.active_token = ''
        self._advance_after(robot_id)
        if outcome is None:
            return 'not_run'
        if outcome == ResultOutcome.CANCELED:
            self.rotation_running = False
            return 'canceled'
        if outcome == ResultOutcome.SUCCEEDED or (
            outcome == ResultOutcome.FAILED and reason_code == REASON_LOW_BATTERY
        ):
            if reason_code == REASON_LOW_BATTERY:
                self.charging_hold.add(robot_id)
            return 'docked'
        self.excluded.add(robot_id)
        return 'excluded'

    # ── 자동 교대 ───────────────────────────────────────────────────────────
    def start_rotation(self) -> None:
        """
        Operator start: run the rotation and return excluded robots to it.

        A robot whose unknown goal has not ended yet stays excluded: it may be
        stopped in the middle of the garage, not on its dock.
        """
        self.rotation_running = True
        self.excluded &= self.restart_wait

    def stop_rotation(self) -> None:
        """Stop dispatching new patrols (operator cancel)."""
        self.rotation_running = False

    def observe_battery(self, robot_id: str, percentage: float | None) -> None:
        """Track BatteryState.percentage (0..1, None when unknown)."""
        self._require_robot(robot_id)
        if percentage is None or not 0.0 <= percentage <= 1.0:
            self.battery[robot_id] = None
            return
        self.battery[robot_id] = percentage
        if percentage < DISPATCH_MIN_BATTERY:
            self.charging_hold.add(robot_id)
        elif percentage >= RECHARGED_BATTERY:
            self.charging_hold.discard(robot_id)

    def dispatch_blocker(self) -> str:
        """Return why no robot may start now, or '' when a robot may start."""
        if not self.rotation_running:
            return 'rotation stopped'
        if self.active_robot_id is not None:
            return 'patrol active'
        if self.restart_wait:
            return f'waiting for unknown goals to end on {sorted(self.restart_wait)}'
        if self.fire_hold:
            return 'fire hold'
        if self.patrol_allowed is not True:
            return 'patrol not allowed'
        if self.estop[ESTOP_ALL][0]:
            return 'estop all'
        return ''

    def robot_ready(self, robot_id: str) -> bool:
        """Return True when this robot may be dispatched."""
        battery = self.battery.get(robot_id)
        return (
            robot_id not in self.excluded
            and robot_id not in self.charging_hold
            and robot_id not in self.estop_pending
            and not self.estop_active_for(robot_id)
            and battery is not None
            and battery >= DISPATCH_MIN_BATTERY
        )

    def select_next_robot(self, server_ready: Callable[[str], bool]) -> str | None:
        """Pick the next robot in turn that can be dispatched, or None."""
        if self.dispatch_blocker():
            return None
        count = len(self.robot_ids)
        for offset in range(count):
            robot_id = self.robot_ids[(self._next_index + offset) % count]
            if self.robot_ready(robot_id) and server_ready(robot_id):
                return robot_id
        return None

    def mark_unknown_goal(self, robot_id: str) -> None:
        """
        Mark a goal that control does not own as running on this robot.

        Such a goal may be left from before a control restart.

        Its outcome is unknown, so the robot leaves the rotation and nothing is
        dispatched until that goal has ended.
        """
        self._require_robot(robot_id)
        self.excluded.add(robot_id)
        self.restart_wait.add(robot_id)

    def clear_restart_wait(self, robot_id: str) -> None:
        """Clear the wait once the unknown goal on this robot has ended."""
        self.restart_wait.discard(robot_id)

    def _advance_after(self, robot_id: str) -> None:
        self._next_index = (self.robot_ids.index(robot_id) + 1) % len(self.robot_ids)

    # ── EStop (Safety Arbiter, CR 9/8) ──────────────────────────────────────
    def set_estop(self, target_robot_id: str, active: bool, reason: int) -> None:
        """Set the EStop state of one target (robot id or 'all')."""
        if target_robot_id not in self.estop:
            raise ValueError(f'unknown EStop target: {target_robot_id}')
        if reason not in ESTOP_REASONS:
            raise ValueError(f'EStop reason must be 0..6: {reason}')
        self.estop[target_robot_id] = (bool(active), int(reason))
        if active:
            # 해제만으로는 다시 움직이지 않는다: 운영자 재개 전까지 이 로봇들은 보류
            affected = self.robot_ids if target_robot_id == ESTOP_ALL else (target_robot_id,)
            self.estop_pending.update(affected)

    def estop_active_for(self, robot_id: str) -> bool:
        return self.estop[ESTOP_ALL][0] or self.estop.get(robot_id, (False, 0))[0]

    @property
    def estop_targets(self) -> tuple[str, ...]:
        return (ESTOP_ALL, *self.robot_ids)

    def estop_frame(self, target: str, sequence_floor: int = 0) -> EStopFrame:
        """
        One frame for one target with a strictly increasing sequence.

        sequence_floor (e.g. wall-clock milliseconds) keeps the sequence growing
        across a control restart, so AMRs never take a release as out of order.
        """
        self._estop_sequence = max(self._estop_sequence + 1, sequence_floor)
        active, reason = self.estop[target]
        return EStopFrame(target, active, reason, self._estop_sequence)

    def resume(self) -> tuple[CommandIntent | None, tuple[str, ...]]:
        """
        Resume robots held by an EStop once the operator calls /control/resume.

        Clears the hold of every robot whose EStop is no longer active and,
        when the active patrol robot was held, returns the command it needs:
        RESUME_PATROL while patrol is allowed, MOVE_TO_SAFE_ZONE while a
        vehicle is moving. Returns (command or None, robots released).
        """
        released = tuple(
            r for r in self.robot_ids
            if r in self.estop_pending and not self.estop_active_for(r)
        )
        self.estop_pending.difference_update(released)
        robot_id = self.active_robot_id
        if robot_id is None or not self.goal_accepted or robot_id not in released:
            return None, released
        command = (
            PatrolCommandType.RESUME_PATROL
            if self.patrol_allowed is True
            else PatrolCommandType.MOVE_TO_SAFE_ZONE
        )
        return self._command(command), released

    # ── 상태 저장 (관제 재시작 대비) ────────────────────────────────────────
    def snapshot(self) -> dict:
        return {
            'fire_hold': self.fire_hold,
            'fire_event_id': self.fire_event_id,
            'rotation_running': self.rotation_running,
            'excluded': sorted(self.excluded),
            'charging_hold': sorted(self.charging_hold),
            'estop_pending': sorted(self.estop_pending),
            'active_robot_id': self.active_robot_id,
            'estop': {target: list(state) for target, state in self.estop.items()},
            'estop_sequence': self._estop_sequence,
            'next_robot_id': self.robot_ids[self._next_index],
        }

    def restore(self, data: dict) -> None:
        """Load a snapshot written before a restart; unknown values are ignored."""
        self.fire_hold = bool(data.get('fire_hold', False))
        self.fire_event_id = str(data.get('fire_event_id', '')) if self.fire_hold else ''
        self.rotation_running = bool(data.get('rotation_running', False))
        self.excluded = {r for r in data.get('excluded', []) if r in self.robot_ids}
        self.charging_hold = {r for r in data.get('charging_hold', []) if r in self.robot_ids}
        self.estop_pending = {r for r in data.get('estop_pending', []) if r in self.robot_ids}
        for target, state in dict(data.get('estop', {})).items():
            if target in self.estop and len(state) == 2 and int(state[1]) in ESTOP_REASONS:
                self.estop[target] = (bool(state[0]), int(state[1]))
        self._estop_sequence = max(0, int(data.get('estop_sequence', 0)))
        next_robot = data.get('next_robot_id')
        if next_robot in self.robot_ids:
            self._next_index = self.robot_ids.index(next_robot)
        previous = data.get('active_robot_id')
        if previous in self.robot_ids:
            # 재시작 전 임무의 결과를 알 수 없다: 그 로봇은 순번에서 빼고,
            # 그 임무가 끝난 것이 확인될 때까지 다른 로봇도 내보내지 않는다.
            self.mark_unknown_goal(previous)
            self._advance_after(previous)
        for target in self.estop_targets:
            if self.estop[target][0]:
                affected = self.robot_ids if target == ESTOP_ALL else (target,)
                self.estop_pending.update(affected)

    def revoke_drive(self, robot_id: str) -> None:
        """Revoke authority while an Action cancellation is in flight."""
        self._require_active(robot_id)
        self.drive_granted = False
        self.active_token = ''

    def observe_feedback(
        self,
        robot_id: str,
        *,
        task_state: int,
        event_type: int,
        event_id: str,
    ) -> FeedbackEffect:
        """Apply Feedback only for the accepted active Patrol Goal."""
        if robot_id != self.active_robot_id or not self.goal_accepted:
            return FeedbackEffect(accepted=False)
        try:
            state = TaskState(task_state)
        except ValueError:
            return FeedbackEffect(accepted=False)

        token_granted = False
        if (
            state is TaskState.WAITING_FOR_TOKEN
            and self.patrol_allowed is True
            and not self.drive_granted
        ):
            token = self._token_factory()
            if not token:
                raise ValueError('token_factory returned an empty token')
            self.active_token = token
            self.drive_granted = True
            token_granted = True

        fire_hold_activated = False
        if (
            state is TaskState.DETECTION_CONFIRMED
            and event_type == EventType.FIRE
            and event_id
            and not self.fire_hold
        ):
            self.fire_hold = True
            self.fire_event_id = event_id
            fire_hold_activated = True

        return FeedbackEffect(
            accepted=True,
            token_granted=token_granted,
            fire_hold_activated=fire_hold_activated,
        )

    def observe_permit(self, allowed: bool) -> tuple[CommandIntent, ...]:
        """Translate a permit edge into at most one active-mission command."""
        if not isinstance(allowed, bool):
            raise ValueError('allowed must be bool')
        if self.patrol_allowed is allowed:
            return ()
        self.patrol_allowed = allowed
        if self.active_robot_id is None or not self.goal_accepted:
            return ()
        if (self.estop_active_for(self.active_robot_id)
                or self.active_robot_id in self.estop_pending):
            # EStop 중이거나 해제 뒤 운영자 재개 전에는 명령을 보내지 않는다.
            # 운영자 재개(resume)가 그때의 permit에 맞는 명령을 보낸다.
            return ()

        command = (
            PatrolCommandType.RESUME_PATROL
            if allowed
            else PatrolCommandType.MOVE_TO_SAFE_ZONE
        )
        return (self._command(command),)

    def _command(self, command: PatrolCommandType) -> CommandIntent:
        self._command_sequence += 1
        name = command.name.lower().replace('_', '-')
        return CommandIntent(
            command_id=(
                f'cmd-{self.active_robot_id}-{name}-'
                f'{self._command_sequence:06d}'
            ),
            robot_id=self.active_robot_id,
            command=command,
        )

    def token_frames(self) -> tuple[TokenFrame, ...]:
        """Produce one monotonically sequenced frame for every robot."""
        frames = []
        for robot_id in self.robot_ids:
            self._token_sequences[robot_id] += 1
            token = (
                self.active_token
                if robot_id == self.active_robot_id and self.drive_granted
                else ''
            )
            frames.append(
                TokenFrame(
                    robot_id=robot_id,
                    token=token,
                    sequence=self._token_sequences[robot_id],
                )
            )
        return tuple(frames)

    def clear_fire_hold(self) -> None:
        """Explicitly clear the operator-owned fire hold."""
        self.fire_hold = False
        self.fire_event_id = ''

    def _require_robot(self, robot_id: str) -> None:
        if robot_id not in self.robot_ids:
            raise ValueError(f'unknown robot_id: {robot_id}')

    def _require_active(self, robot_id: str) -> None:
        self._require_robot(robot_id)
        if robot_id != self.active_robot_id:
            raise ControlStateError(f'{robot_id} is not the active patrol robot')
