"""Unit tests for the ROS-independent bridge core."""

from patrol_amr_bridge.bridge_core import Action
from patrol_amr_bridge.bridge_core import BridgeCore
from patrol_amr_bridge.bridge_core import CommandStatus
from patrol_amr_bridge.bridge_core import Outcome
from patrol_amr_bridge.bridge_core import Phase
from patrol_amr_bridge.bridge_core import Reason
from patrol_amr_bridge.bridge_core import TaskState
import pytest

SAFE = 1
RESUME = 2


def running():
    core = BridgeCore('robot1')
    assert core.accept_goal('cmd-1', 'robot1')
    assert core.on_token('robot1', 'tok', 7) == (Action.SEND_ROUTE_GOAL,)
    assert core.on_route_goal_response(True) is None
    assert core.phase is Phase.PATROLLING
    return core


def test_requires_robot_id():
    with pytest.raises(ValueError):
        BridgeCore('')


def test_goal_filtering():
    core = BridgeCore('robot1')
    assert not core.accept_goal('cmd', 'robot6')
    assert not core.accept_goal('', 'robot1')
    assert core.accept_goal('cmd', 'robot1')
    assert not core.accept_goal('cmd-2', 'robot1')


def test_waits_for_token_before_driving():
    core = BridgeCore('robot1')
    core.accept_goal('cmd', 'robot1')
    assert core.feedback().task_state is TaskState.WAITING_FOR_TOKEN
    assert core.on_token('robot6', 'tok', 1) == ()
    assert core.on_token('robot1', '', 1) == ()
    assert core.feedback().token_valid is False
    assert core.on_token('robot1', 'tok', 3) == (Action.SEND_ROUTE_GOAL,)
    feedback = core.feedback()
    assert feedback.token_valid and feedback.accepted_token_sequence == 3


def test_repeated_token_does_not_resend_goal():
    core = running()
    assert core.on_token('robot1', 'tok', 8) == ()
    assert core.feedback().accepted_token_sequence == 8


def test_safe_zone_command_stops_and_resume_resumes():
    core = running()
    assert core.on_command('robot1', 'c1', SAFE) == (Action.REQUEST_STOP,)
    feedback = core.feedback()
    assert feedback.task_state is TaskState.BLOCKED
    assert feedback.command_status is CommandStatus.EXECUTING
    assert feedback.last_command_id == 'c1'
    assert core.on_command('robot1', 'c1b', SAFE) == ()
    assert core.on_command('robot1', 'c2', RESUME) == (Action.PUBLISH_RESUME,)
    assert core.phase is Phase.PATROLLING
    assert core.feedback().command_status is CommandStatus.ACCEPTED


def test_resume_without_stop_is_rejected():
    core = running()
    assert core.on_command('robot1', 'c', RESUME) == ()
    assert core.command_status is CommandStatus.REJECTED


def test_unknown_or_foreign_command():
    core = running()
    assert core.on_command('robot6', 'x', SAFE) == ()
    assert core.on_command('robot1', 'x', 99) == ()
    assert core.command_status is CommandStatus.REJECTED


def test_command_before_goal_is_rejected():
    core = BridgeCore('robot1')
    assert core.on_command('robot1', 'c', SAFE) == ()
    assert core.command_status is CommandStatus.REJECTED


def test_token_revoke_and_regrant():
    core = running()
    assert core.on_token('robot1', '', 9) == (Action.REQUEST_STOP,)
    assert core.phase is Phase.PAUSED
    assert core.on_token('robot1', '', 10) == ()
    assert core.on_token('robot1', 'tok', 11) == (Action.PUBLISH_RESUME,)
    assert core.phase is Phase.PATROLLING


def test_resume_waits_for_both_reasons_to_clear():
    core = running()
    core.on_command('robot1', 'c1', SAFE)
    core.on_token('robot1', '', 9)
    assert core.on_command('robot1', 'c2', RESUME) == ()
    assert core.phase is Phase.PAUSED
    assert core.on_token('robot1', 'tok', 10) == (Action.PUBLISH_RESUME,)


def test_stop_before_route_accepted_keeps_pause():
    core = BridgeCore('robot1')
    core.accept_goal('cmd', 'robot1')
    core.on_token('robot1', 'tok', 1)
    assert core.on_command('robot1', 'c1', SAFE) == (Action.REQUEST_STOP,)
    assert core.feedback().task_state is TaskState.BLOCKED
    core.on_route_goal_response(True)
    assert core.phase is Phase.PAUSED


def test_route_rejected_fails_goal():
    core = BridgeCore('robot1')
    core.accept_goal('cmd', 'robot1')
    core.on_token('robot1', 'tok', 1)
    result = core.on_route_goal_response(False)
    assert result.outcome is Outcome.FAILED
    assert result.reason_code is Reason.NAVIGATION_FAILED


def test_route_results():
    core = running()
    assert core.on_route_result(True).outcome is Outcome.SUCCEEDED
    assert core.on_route_result(True) is None
    core = running()
    assert core.on_route_result(False).reason_code is Reason.NAVIGATION_FAILED


def test_cancel_stops_only_when_moving():
    core = running()
    result, actions = core.on_cancel()
    assert result.outcome is Outcome.CANCELED
    assert result.reason_code is Reason.CONTROL_CANCELED
    assert actions == (Action.REQUEST_STOP,)
    core = running()
    core.on_command('robot1', 'c', SAFE)
    assert core.on_cancel()[1] == ()


def test_new_goal_after_finish():
    core = running()
    core.on_route_result(True)
    assert core.accept_goal('cmd-2', 'robot1')
    assert core.feedback().task_state is TaskState.WAITING_FOR_TOKEN


def test_internal_error():
    core = running()
    assert core.on_internal_error('x').reason_code is Reason.INTERNAL_ERROR
    assert BridgeCore('robot1').on_internal_error('x') is None
