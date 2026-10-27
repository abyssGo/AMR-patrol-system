"""
ROS-independent decisions for the Control <-> AMR patrol bridge.

The Control Server speaks ``patrol_interfaces`` (Patrol Action, DriveToken,
PatrolCommand). The final AMR patrol (``genius_patrol``) instead runs a fixed
route behind a ``nav2_msgs/NavigateToPose`` Action and pauses/resumes through
``event_check`` (``patrol_stop`` SetBool service, ``patrol_resume`` Bool topic).
This core decides what the bridge must do for each input so the logic can be
tested without ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from enum import IntEnum


class TaskState(IntEnum):
    """Patrol Feedback task states (patrol_interfaces/action/Patrol)."""

    WAITING_FOR_TOKEN = 1
    PATROLLING = 4
    RESUMING = 8
    BLOCKED = 10


class CommandStatus(IntEnum):
    """Patrol Feedback command status values."""

    NONE = 0
    ACCEPTED = 1
    EXECUTING = 2
    REJECTED = 3


class PatrolCommandType(IntEnum):
    """PatrolCommand values sent by the Control Server."""

    MOVE_TO_SAFE_ZONE = 1
    RESUME_PATROL = 2


class Outcome(IntEnum):
    """Patrol Result outcome values."""

    SUCCEEDED = 0
    FAILED = 1
    CANCELED = 2


class Reason(IntEnum):
    """Patrol Result reason codes used by the bridge."""

    NONE = 0
    CONTROL_CANCELED = 1
    NAVIGATION_FAILED = 5
    INTERNAL_ERROR = 10


class Phase(Enum):
    """Bridge lifecycle for one Patrol Goal."""

    IDLE = 'idle'
    WAITING_FOR_TOKEN = 'waiting_for_token'
    STARTING = 'starting'
    PATROLLING = 'patrolling'
    PAUSED = 'paused'
    FINISHED = 'finished'


class Action(Enum):
    """Side effects the ROS node must perform."""

    SEND_ROUTE_GOAL = 'send_route_goal'
    REQUEST_STOP = 'request_stop'
    PUBLISH_RESUME = 'publish_resume'


@dataclass(frozen=True)
class Feedback:
    """Patrol Feedback fields the bridge fills in."""

    task_state: TaskState
    token_valid: bool
    accepted_token_sequence: int
    last_command_id: str = ''
    command_status: CommandStatus = CommandStatus.NONE


@dataclass(frozen=True)
class Result:
    """Patrol Result fields."""

    outcome: Outcome
    reason_code: Reason
    reason: str


class BridgeCore:
    """Track one Patrol Goal and translate Control inputs to AMR actions."""

    def __init__(self, robot_id: str) -> None:
        if not robot_id:
            raise ValueError('robot_id must not be empty')
        self.robot_id = robot_id
        self.phase = Phase.IDLE
        self.command_id = ''
        self.token = ''
        self.token_sequence = 0
        self.last_command_id = ''
        self.command_status = CommandStatus.NONE
        self.paused_by_command = False
        self.paused_by_token = False

    # -- Patrol Goal lifecycle -------------------------------------------
    def accept_goal(self, command_id: str, robot_id: str) -> bool:
        """Accept a Goal for this robot when no other Goal is active."""
        if robot_id != self.robot_id or not command_id:
            return False
        if self.phase not in (Phase.IDLE, Phase.FINISHED):
            return False
        self.phase = Phase.WAITING_FOR_TOKEN
        self.command_id = command_id
        self.token = ''
        self.token_sequence = 0
        self.last_command_id = ''
        self.command_status = CommandStatus.NONE
        self.paused_by_command = False
        self.paused_by_token = False
        return True

    def feedback(self) -> Feedback:
        """Feedback to publish for the current phase."""
        state = {
            Phase.WAITING_FOR_TOKEN: TaskState.WAITING_FOR_TOKEN,
            Phase.STARTING: TaskState.PATROLLING,
            Phase.PATROLLING: TaskState.PATROLLING,
            Phase.PAUSED: TaskState.BLOCKED,
        }.get(self.phase, TaskState.BLOCKED)
        if self._paused and self.phase is Phase.STARTING:
            state = TaskState.BLOCKED
        return Feedback(
            task_state=state,
            token_valid=bool(self.token),
            accepted_token_sequence=self.token_sequence,
            last_command_id=self.last_command_id,
            command_status=self.command_status,
        )

    # -- DriveToken --------------------------------------------------------
    def on_token(self, holder_robot_id: str, token: str, sequence: int) -> tuple[Action, ...]:
        """Start on the first valid token; pause on revocation; resume on regrant."""
        if holder_robot_id != self.robot_id:
            return ()
        if self.phase in (Phase.IDLE, Phase.FINISHED):
            return ()
        had_token = bool(self.token)
        self.token = token
        if token:
            self.token_sequence = sequence

        if self.phase is Phase.WAITING_FOR_TOKEN and token:
            self.phase = Phase.STARTING
            return (Action.SEND_ROUTE_GOAL,)

        if self.phase in (Phase.STARTING, Phase.PATROLLING, Phase.PAUSED):
            if had_token and not token and not self.paused_by_token:
                self.paused_by_token = True
                return self._pause()
            if token and self.paused_by_token:
                self.paused_by_token = False
                return self._resume_if_clear()
        return ()

    # -- PatrolCommand -----------------------------------------------------
    def on_command(self, robot_id: str, command_id: str, command: int) -> tuple[Action, ...]:
        """
        Map Control's safe-zone/resume commands to stop/resume.

        The final AMR patrol has no safe-zone route when event_check is used,
        so MOVE_TO_SAFE_ZONE stops the robot in place and RESUME_PATROL resumes
        it from the same patrol step.
        """
        if robot_id != self.robot_id:
            return ()
        self.last_command_id = command_id
        if self.phase not in (Phase.STARTING, Phase.PATROLLING, Phase.PAUSED):
            self.command_status = CommandStatus.REJECTED
            return ()
        try:
            kind = PatrolCommandType(command)
        except ValueError:
            self.command_status = CommandStatus.REJECTED
            return ()
        if kind is PatrolCommandType.MOVE_TO_SAFE_ZONE:
            if self.paused_by_command:
                self.command_status = CommandStatus.EXECUTING
                return ()
            self.paused_by_command = True
            self.command_status = CommandStatus.EXECUTING
            return self._pause()
        if not self.paused_by_command:
            self.command_status = CommandStatus.REJECTED
            return ()
        self.paused_by_command = False
        self.command_status = CommandStatus.ACCEPTED
        return self._resume_if_clear()

    # -- AMR route results ---------------------------------------------------
    def on_route_goal_response(self, accepted: bool) -> Result | None:
        """AMR route server accepted or rejected the forwarded goal."""
        if self.phase is not Phase.STARTING:
            return None
        if accepted:
            self.phase = Phase.PAUSED if self._paused else Phase.PATROLLING
            return None
        return self._finish(Outcome.FAILED, Reason.NAVIGATION_FAILED,
                            'AMR patrol route goal rejected')

    def on_route_result(self, succeeded: bool) -> Result | None:
        """AMR finished the fixed route (docked) or failed."""
        if self.phase in (Phase.IDLE, Phase.FINISHED):
            return None
        if succeeded:
            return self._finish(Outcome.SUCCEEDED, Reason.NONE,
                                'patrol route finished and docked')
        return self._finish(Outcome.FAILED, Reason.NAVIGATION_FAILED,
                            'AMR patrol route failed')

    def on_cancel(self) -> tuple[Result, tuple[Action, ...]]:
        """Control canceled the Patrol Goal: stop the robot in place."""
        actions: tuple[Action, ...] = ()
        if self.phase in (Phase.STARTING, Phase.PATROLLING) and not self._paused:
            actions = (Action.REQUEST_STOP,)
        result = self._finish(Outcome.CANCELED, Reason.CONTROL_CANCELED,
                              'canceled by Control; robot stopped in place')
        return result, actions

    def on_internal_error(self, reason: str) -> Result | None:
        """Bridge-side failure such as a missing AMR server."""
        if self.phase in (Phase.IDLE, Phase.FINISHED):
            return None
        return self._finish(Outcome.FAILED, Reason.INTERNAL_ERROR, reason)

    # -- helpers ---------------------------------------------------------------
    @property
    def _paused(self) -> bool:
        return self.paused_by_command or self.paused_by_token

    def _pause(self) -> tuple[Action, ...]:
        if self.phase is Phase.PATROLLING:
            self.phase = Phase.PAUSED
        return (Action.REQUEST_STOP,)

    def _resume_if_clear(self) -> tuple[Action, ...]:
        if self._paused:
            return ()
        if self.phase is Phase.PAUSED:
            self.phase = Phase.PATROLLING
        return (Action.PUBLISH_RESUME,)

    def _finish(self, outcome: Outcome, reason_code: Reason, reason: str) -> Result:
        self.phase = Phase.FINISHED
        self.token = ''
        self.paused_by_command = False
        self.paused_by_token = False
        return Result(outcome=outcome, reason_code=reason_code, reason=reason)
