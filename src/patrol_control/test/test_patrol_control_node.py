"""Unit tests for PatrolControlNode callbacks using local test doubles."""

from itertools import count
from time import monotonic
from types import SimpleNamespace

from action_msgs.msg import GoalStatus, GoalStatusArray
from patrol_control import control_state
from patrol_control.control_core import EventType
from patrol_control.control_core import TaskState
from patrol_control.patrol_control_node import PatrolControlNode
from patrol_interfaces.action import Patrol
from patrol_interfaces.msg import PatrolCommand as PatrolCommandMsg
from patrol_interfaces.srv import SetEStop
import pytest
import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

_UUIDS = count(1)


class FakeFuture:
    """Synchronous stand-in for the rclpy futures used by the node."""

    def __init__(self) -> None:
        self._callbacks = []
        self._result = None
        self._exception = None

    def add_done_callback(self, callback) -> None:
        self._callbacks.append(callback)

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def resolve(self, result) -> None:
        self._result = result
        for callback in tuple(self._callbacks):
            callback(self)

    def reject(self, exception: Exception) -> None:
        self._exception = exception
        for callback in tuple(self._callbacks):
            callback(self)


class FakeGoalHandle:
    """Accepted or rejected Patrol Goal handle."""

    def __init__(self, *, accepted: bool = True) -> None:
        self.accepted = accepted
        self.result_future = FakeFuture()
        self.cancel_called = False
        self.goal_id = SimpleNamespace(uuid=next(_UUIDS).to_bytes(16, 'big'))

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_called = True
        return FakeFuture()


class FakeActionClient:
    """Capture Patrol Goals without contacting an Action server."""

    def __init__(self, *, ready: bool = True) -> None:
        self.ready = ready
        self.goal_future = FakeFuture()
        self.goals = []
        self.feedback_callback = None

    def server_is_ready(self) -> bool:
        return self.ready

    def send_goal_async(self, goal, *, feedback_callback):
        self.goals.append(goal)
        self.feedback_callback = feedback_callback
        return self.goal_future


class RecordingPublisher:
    """Collect published ROS messages for assertions."""

    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


@pytest.fixture
def control_node(tmp_path, monkeypatch):
    """Create a real Node without spinning an executor or ROS peers."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    monkeypatch.setattr(control_state, 'STATE_PATH', tmp_path / 'state.json')
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=[])
    node = PatrolControlNode()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def install_recorders(node: PatrolControlNode):
    """Replace command and token outputs with recording publishers."""
    commands = {
        robot_id: RecordingPublisher() for robot_id in node._core.robot_ids
    }
    tokens = {
        robot_id: RecordingPublisher() for robot_id in node._core.robot_ids
    }
    node._command_publishers = commands
    node._token_publishers = tokens
    return commands, tokens


def start_and_accept(node: PatrolControlNode, robot_id: str = 'robot1'):
    """Start a permitted Goal and synchronously accept it."""
    client = FakeActionClient()
    node._action_clients[robot_id] = client
    node._core.observe_permit(True)
    assert node.start_patrol(robot_id)
    goal_handle = FakeGoalHandle()
    client.goal_future.resolve(goal_handle)
    return client, goal_handle


def feed(node, message, robot_id='robot1'):
    node._on_patrol_feedback(message, robot_id=robot_id, generation=node._goal_generation)


def finish(goal_handle, outcome, reason_code=0):
    result = Patrol.Result()
    result.outcome, result.reason_code = outcome, reason_code
    goal_handle.result_future.resolve(SimpleNamespace(result=result))


def feedback_message(*, state: int, event_type: int = 0, event_id: str = ''):
    """Build the Feedback wrapper passed by rclpy ActionClient."""
    feedback = Patrol.Feedback()
    feedback.task_state = state
    feedback.event_type = event_type
    feedback.event_id = event_id
    return SimpleNamespace(feedback=feedback)


def test_start_rejects_unknown_robot_missing_permit_and_unready_server(
    control_node,
) -> None:
    client = FakeActionClient(ready=True)
    control_node._action_clients['robot1'] = client

    assert not control_node.start_patrol('robot9')
    assert not control_node.start_patrol('robot1')
    assert client.goals == []

    control_node._core.observe_permit(True)
    client.ready = False
    assert not control_node.start_patrol('robot1')
    assert client.goals == []


def test_goal_contains_identity_and_acceptance_does_not_grant_token(
    control_node,
) -> None:
    _, tokens = install_recorders(control_node)
    client, _ = start_and_accept(control_node)

    assert len(client.goals) == 1
    assert client.goals[0].robot_id == 'robot1'
    assert client.goals[0].command_id == 'cmd-robot1-start-000001'

    control_node._publish_token_frames()
    assert tokens['robot1'].messages[-1].token == ''
    assert tokens['robot6'].messages[-1].token == ''


def test_waiting_feedback_grants_token_only_to_active_robot(control_node) -> None:
    _, tokens = install_recorders(control_node)
    start_and_accept(control_node)

    feed(control_node, feedback_message(state=TaskState.WAITING_FOR_TOKEN))
    control_node._publish_token_frames()

    robot1 = tokens['robot1'].messages[-1]
    robot6 = tokens['robot6'].messages[-1]
    assert robot1.token
    assert robot1.holder_robot_id == 'robot1'
    assert robot1.lease_duration.sec == 1
    assert robot1.lease_duration.nanosec == 0
    assert robot1.sequence == 1
    assert robot6.token == ''
    assert robot6.holder_robot_id == 'robot6'
    assert robot6.sequence == 1


def test_feedback_from_non_active_robot_does_not_grant_token(
    control_node,
) -> None:
    _, tokens = install_recorders(control_node)
    start_and_accept(control_node)

    feed(control_node, feedback_message(state=TaskState.WAITING_FOR_TOKEN), 'robot6')
    control_node._publish_token_frames()

    assert tokens['robot1'].messages[-1].token == ''
    assert tokens['robot6'].messages[-1].token == ''


def test_permit_edges_publish_one_command_each(control_node) -> None:
    commands, _ = install_recorders(control_node)
    start_and_accept(control_node)

    control_node._on_patrol_allowed(Bool(data=False))
    control_node._on_patrol_allowed(Bool(data=False))
    control_node._on_patrol_allowed(Bool(data=True))

    published = commands['robot1'].messages
    assert len(published) == 2
    assert published[0].command == published[0].MOVE_TO_SAFE_ZONE
    assert published[1].command == published[1].RESUME_PATROL
    assert published[0].robot_id == 'robot1'
    assert published[0].command_id != published[1].command_id
    assert commands['robot6'].messages == []


@pytest.mark.parametrize('failure_mode', ['rejected', 'exception'])
def test_failed_goal_response_releases_active_slot(
    control_node,
    failure_mode,
) -> None:
    client = FakeActionClient()
    control_node._action_clients['robot1'] = client
    control_node._core.observe_permit(True)
    assert control_node.start_patrol('robot1')

    if failure_mode == 'rejected':
        client.goal_future.resolve(FakeGoalHandle(accepted=False))
    else:
        client.goal_future.reject(RuntimeError('send failed'))

    assert control_node._core.active_robot_id is None
    assert not control_node._core.drive_granted


def test_cancel_revokes_token_before_action_result(control_node) -> None:
    _, tokens = install_recorders(control_node)
    _, goal_handle = start_and_accept(control_node)
    feed(control_node, feedback_message(state=TaskState.WAITING_FOR_TOKEN))

    assert control_node.cancel_active_patrol()
    assert goal_handle.cancel_called
    assert tokens['robot1'].messages[-1].token == ''
    assert control_node._core.active_robot_id == 'robot1'

    result = Patrol.Result()
    result.outcome = Patrol.Result.CANCELED
    result.reason_code = Patrol.Result.REASON_CONTROL_CANCELED
    goal_handle.result_future.resolve(SimpleNamespace(result=result))

    assert control_node._core.active_robot_id is None
    assert not control_node._core.drive_granted


def test_cancel_before_acceptance_cancels_right_after_acceptance(control_node) -> None:
    assert not control_node.cancel_active_patrol()           # 활성 임무 없음

    client = FakeActionClient()
    control_node._action_clients['robot1'] = client
    control_node._core.observe_permit(True)
    assert control_node.start_patrol('robot1')
    assert control_node.cancel_active_patrol()               # 응답 전 취소 요청
    goal_handle = FakeGoalHandle()
    client.goal_future.resolve(goal_handle)
    assert goal_handle.cancel_called


def test_action_result_exception_still_releases_active_slot(control_node) -> None:
    _, goal_handle = start_and_accept(control_node)

    goal_handle.result_future.reject(RuntimeError('result failed'))

    assert control_node._core.active_robot_id is None
    assert 'robot1' not in control_node._goal_handles


def test_fire_feedback_sets_hold_and_explicit_clear_removes_it(
    control_node,
) -> None:
    start_and_accept(control_node)

    feed(control_node, feedback_message(
        state=TaskState.DETECTION_CONFIRMED, event_type=EventType.FIRE, event_id='fire-001'))

    assert control_node._core.fire_hold
    assert control_node._core.fire_event_id == 'fire-001'
    control_node.clear_fire_hold()
    assert not control_node._core.fire_hold


# ── 자동 교대 · 배터리 · 도킹 실패 ─────────────────────────────────────────
def ready_rotation(node, battery=0.9):
    node._started_at = monotonic() - 100.0                   # 시작 확인 시간 지남
    clients = {r: FakeActionClient() for r in node._core.robot_ids}
    node._action_clients.update(clients)
    node._core.rotation_running = True
    node._core.observe_permit(True)
    for robot_id in node._core.robot_ids:
        node._core.observe_battery(robot_id, battery)
    return clients


def accept(client):
    handle = FakeGoalHandle()
    client.goal_future.resolve(handle)
    client.goal_future = FakeFuture()
    return handle


def test_docked_robot_hands_over_to_the_next_robot(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    assert len(clients['robot1'].goals) == 1
    handle = accept(clients['robot1'])
    control_node._try_dispatch()
    assert clients['robot6'].goals == []                     # 한 번에 한 대
    finish(handle, Patrol.Result.SUCCEEDED)
    control_node._try_dispatch()
    assert len(clients['robot6'].goals) == 1                 # 도킹 확인 → 다음 AMR


def test_docking_failure_excludes_robot_and_the_other_continues(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    finish(accept(clients['robot1']), Patrol.Result.FAILED,
           Patrol.Result.REASON_DOCKING_FAILED)
    assert 'robot1' in control_node._core.excluded
    control_node._try_dispatch()
    finish(accept(clients['robot6']), Patrol.Result.SUCCEEDED)
    control_node._try_dispatch()
    assert len(clients['robot1'].goals) == 1                 # 제외된 로봇은 건너뜀
    assert len(clients['robot6'].goals) == 2
    response = control_node._srv_start_patrol(Trigger.Request(), Trigger.Response())
    assert response.success and 'robot1' not in control_node._core.excluded


def test_low_battery_return_counts_as_docked_and_waits_for_80(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    finish(accept(clients['robot1']), Patrol.Result.FAILED, Patrol.Result.REASON_LOW_BATTERY)
    assert 'robot1' not in control_node._core.excluded
    assert 'robot1' in control_node._core.charging_hold
    control_node._core.observe_battery('robot6', 0.2)        # 30% 미만
    control_node._try_dispatch()
    assert len(clients['robot6'].goals) == 0
    control_node._core.observe_battery('robot1', 0.5)        # 아직 80% 전
    control_node._try_dispatch()
    assert len(clients['robot1'].goals) == 1
    control_node._core.observe_battery('robot1', 0.8)
    control_node._try_dispatch()
    assert len(clients['robot1'].goals) == 2


def test_fire_hold_blocks_the_next_dispatch(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    handle = accept(clients['robot1'])
    feed(control_node, feedback_message(
        state=TaskState.DETECTION_CONFIRMED, event_type=EventType.FIRE, event_id='f1'))
    finish(handle, Patrol.Result.SUCCEEDED)
    control_node._try_dispatch()
    assert clients['robot6'].goals == []
    control_node._srv_clear_fire_hold(Trigger.Request(), Trigger.Response())
    control_node._try_dispatch()
    assert len(clients['robot6'].goals) == 1


# ── EStop ───────────────────────────────────────────────────────────────────
def set_estop(node, target, active, reason=1):
    request = SetEStop.Request(target_robot_id=target, active=active, reason=reason)
    return node._srv_set_estop(request, SetEStop.Response())


def test_estop_release_needs_operator_resume_and_follows_permit(control_node) -> None:
    commands, _ = install_recorders(control_node)
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    accept(clients['robot1'])
    assert set_estop(control_node, 'robot1', True).success
    assert set_estop(control_node, 'robot1', False).success
    control_node._on_patrol_allowed(Bool(data=False))        # 해제 뒤, 재개 전 차량 진입
    control_node._on_patrol_allowed(Bool(data=True))
    assert commands['robot1'].messages == []                 # 재개 전에는 명령 없음
    control_node._on_patrol_allowed(Bool(data=False))
    response = control_node._srv_resume(Trigger.Request(), Trigger.Response())
    assert response.success
    assert commands['robot1'].messages[-1].command == PatrolCommandMsg.MOVE_TO_SAFE_ZONE


def test_resume_is_refused_while_estop_is_active(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    accept(clients['robot1'])
    set_estop(control_node, 'all', True)
    response = control_node._srv_resume(Trigger.Request(), Trigger.Response())
    assert not response.success


def test_estop_blocks_dispatch_until_resume(control_node) -> None:
    clients = ready_rotation(control_node)
    set_estop(control_node, 'all', True)
    set_estop(control_node, 'all', False)
    control_node._try_dispatch()
    assert clients['robot1'].goals == [] and clients['robot6'].goals == []
    control_node._srv_resume(Trigger.Request(), Trigger.Response())
    control_node._try_dispatch()
    assert len(clients['robot1'].goals) == 1


def test_estop_targets_are_published_one_per_tick(control_node) -> None:
    recorder = RecordingPublisher()
    control_node._estop_publisher = recorder
    for _ in range(3):
        control_node._publish_next_estop()
    targets = [m.target_robot_id for m in recorder.messages]
    assert sorted(targets) == ['all', 'robot1', 'robot6']
    sequences = [m.sequence for m in recorder.messages]
    assert sequences == sorted(sequences) and len(set(sequences)) == 3


def test_bad_estop_request_is_rejected(control_node) -> None:
    assert not set_estop(control_node, 'robot9', True).success
    assert not set_estop(control_node, 'robot1', True, reason=7).success


# ── 관제가 모르는 임무 · 응답 없음 · 서버 상실 ────────────────────────────
def status(*states):
    message = GoalStatusArray()
    for i, state in enumerate(states):
        item = GoalStatus()
        item.status = state
        item.goal_info.goal_id.uuid = [0xF0 + i] * 16
        message.status_list.append(item)
    return message


def test_unknown_goal_blocks_dispatch_until_it_ends(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._on_goal_status(status(GoalStatus.STATUS_EXECUTING), robot_id='robot6')
    assert 'robot6' in control_node._core.restart_wait
    control_node._try_dispatch()
    assert clients['robot1'].goals == []                     # 두 대가 동시에 돌지 않게
    control_node._on_goal_status(status(GoalStatus.STATUS_CANCELED), robot_id='robot6')
    assert not control_node._core.restart_wait
    control_node._try_dispatch()
    assert len(clients['robot1'].goals) == 1
    assert 'robot6' in control_node._core.excluded


def test_own_goal_status_is_not_unknown(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    control_node._on_goal_status(status(GoalStatus.STATUS_ACCEPTED), robot_id='robot1')
    assert not control_node._core.restart_wait               # 응답 전 자기 Goal
    handle = accept(clients['robot1'])
    message = status(GoalStatus.STATUS_EXECUTING)
    message.status_list[0].goal_info.goal_id.uuid = list(handle.goal_id.uuid)
    control_node._on_goal_status(message, robot_id='robot1')
    assert not control_node._core.restart_wait


def test_restored_active_robot_is_treated_as_unknown(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    monkeypatch.setattr(control_state, 'STATE_PATH', tmp_path / 'state.json')
    control_state.save_state({'active_robot_id': 'robot1', 'fire_hold': True,
                              'fire_event_id': 'f9', 'estop': {'robot6': [True, 1]}})
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=[])
    node = PatrolControlNode()
    try:
        assert node._core.fire_hold                           # 재시작해도 화재 보류 유지
        assert 'robot1' in node._core.excluded
        assert 'robot1' in node._core.restart_wait
        assert node._core.estop['robot6'] == (True, 1)
        assert 'robot6' in node._core.estop_pending
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_goal_response_timeout_releases_slot_and_cancels_late_acceptance(control_node):
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    robot_id, generation, _ = control_node._pending_goal
    control_node._pending_goal = (robot_id, generation, monotonic() - 100.0)
    control_node._check_pending_goal(monotonic())
    assert control_node._core.active_robot_id is None
    late = FakeGoalHandle()
    clients['robot1'].goal_future.resolve(late)
    assert late.cancel_called


def test_server_lost_during_patrol_fails_the_goal(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    accept(clients['robot1'])
    clients['robot1'].ready = False
    now = monotonic()
    control_node._check_server_lost(now)
    control_node._check_server_lost(now + 100.0)
    assert control_node._core.active_robot_id is None
    assert 'robot1' in control_node._core.excluded


# ── 재검토에서 찾은 경우들 ───────────────────────────────────────────────────
def test_cancel_all_with_nothing_running_clears_the_wait(control_node) -> None:
    ready_rotation(control_node)
    control_node._core.mark_unknown_goal('robot1')
    control_node._on_cancel_all(
        SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[])),
        robot_id='robot1')
    assert not control_node._core.restart_wait
    assert 'robot1' in control_node._core.excluded


def test_server_without_any_status_clears_the_wait_after_grace(control_node) -> None:
    ready_rotation(control_node)                             # 시작 대기시간 지남
    control_node._core.mark_unknown_goal('robot6')
    control_node._cancel_clients['robot6'] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda req: FakeFuture())
    control_node._check_unknown_goals(monotonic())
    assert not control_node._core.restart_wait


def test_late_accepted_own_goal_is_not_unknown(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    robot_id, generation, _ = control_node._pending_goal
    control_node._pending_goal = (robot_id, generation, monotonic() - 100.0)
    control_node._check_pending_goal(monotonic())
    late = FakeGoalHandle()
    clients['robot1'].goal_future.resolve(late)
    message = status(GoalStatus.STATUS_CANCELING)
    message.status_list[0].goal_info.goal_id.uuid = list(late.goal_id.uuid)
    control_node._on_goal_status(message, robot_id='robot1')
    assert not control_node._core.restart_wait


def test_reconnected_robot_with_goal_still_running_is_canceled(control_node) -> None:
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    handle = accept(clients['robot1'])
    clients['robot1'].ready = False
    now = monotonic()
    control_node._check_server_lost(now)
    control_node._check_server_lost(now + 100.0)
    assert 'robot1' in control_node._core.restart_wait
    message = status(GoalStatus.STATUS_EXECUTING)          # 다시 연결됐더니 아직 실행 중
    message.status_list[0].goal_info.goal_id.uuid = list(handle.goal_id.uuid)
    control_node._on_goal_status(message, robot_id='robot1')
    assert 'robot1' in control_node._core.restart_wait     # 끝날 때까지 출동 보류


def test_early_start_keeps_robot_with_unknown_goal_excluded(control_node) -> None:
    ready_rotation(control_node)
    control_node._core.mark_unknown_goal('robot1')
    control_node._srv_start_patrol(Trigger.Request(), Trigger.Response())
    assert 'robot1' in control_node._core.excluded


def test_resume_is_resent_while_robot_reports_blocked(control_node) -> None:
    commands, _ = install_recorders(control_node)
    clients = ready_rotation(control_node)
    control_node._try_dispatch()
    accept(clients['robot1'])
    set_estop(control_node, 'robot1', True)
    set_estop(control_node, 'robot1', False)
    feed(control_node, feedback_message(state=TaskState.BLOCKED))
    control_node._srv_resume(Trigger.Request(), Trigger.Response())
    control_node._resend_resume(monotonic())
    sent = commands['robot1'].messages
    assert len(sent) == 2 and sent[0].command_id == sent[1].command_id   # 같은 명령 재전송
    feed(control_node, feedback_message(state=TaskState.RESUMING))
    control_node._resend_resume(monotonic())
    assert len(commands['robot1'].messages) == 2                        # BLOCKED가 풀리면 멈춤
