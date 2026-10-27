"""
rclpy integration test: fake Control + fake genius_patrol around the bridge.

The fake AMR side mirrors genius_patrol's real names: a NavigateToPose server
on ``/robot1/patrol_route`` (genius_patrol remapped from ``patrol_action``), a
``/robot1/patrol_stop`` SetBool service and a ``/robot1/patrol_resume`` Bool
topic.  The fake Control side uses the same Patrol Action, DriveToken and
PatrolCommand names and QoS as patrol_control.
"""

import threading
import time

from nav2_msgs.action import NavigateToPose
from patrol_amr_bridge.amr_bridge_node import AmrBridgeNode
from patrol_interfaces.action import Patrol
from patrol_interfaces.msg import DriveToken
from patrol_interfaces.msg import PatrolCommand
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class FakeAmr(Node):
    """Stand-in for genius_patrol + event_check."""

    def __init__(self, accept=True):
        super().__init__('fake_amr')
        group = ReentrantCallbackGroup()
        self.accept = accept
        self.finish = threading.Event()
        self.succeed = True
        self.route_goals = 0
        self.stops = 0
        self.resumes = 0
        ActionServer(
            self, NavigateToPose, '/robot1/patrol_route', self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT,
            callback_group=group)
        self.create_service(SetBool, '/robot1/patrol_stop', self._stop, callback_group=group)
        self.create_subscription(Bool, '/robot1/patrol_resume', self._resume, 2,
                                 callback_group=group)

    def _execute(self, handle):
        self.route_goals += 1
        self.finish.wait(timeout=30.0)
        if self.succeed:
            handle.succeed()
        else:
            handle.abort()
        return NavigateToPose.Result()

    def _stop(self, request, response):
        if not request.data:
            self.stops += 1
        response.success = True
        response.message = 'stopped'
        return response

    def _resume(self, message):
        if message.data:
            self.resumes += 1


class FakeControl(Node):
    """Stand-in for patrol_control."""

    def __init__(self):
        super().__init__('fake_control')
        self.client = ActionClient(self, Patrol, '/robot1/patrol_action')
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.token_pub = self.create_publisher(DriveToken, '/robot1/drive_token', qos)
        self.command_pub = self.create_publisher(PatrolCommand, '/robot1/patrol_command', 10)
        self.feedback = []
        self.sequence = 0

    def on_feedback(self, message):
        self.feedback.append(message.feedback)

    def token(self, value='tok'):
        self.sequence += 1
        message = DriveToken()
        message.token = value
        message.holder_robot_id = 'robot1'
        message.lease_duration.sec = 1
        message.sequence = self.sequence
        self.token_pub.publish(message)

    def command(self, command, command_id):
        message = PatrolCommand()
        message.command_id = command_id
        message.robot_id = 'robot1'
        message.command = command
        self.command_pub.publish(message)


def wait_until(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def system():
    rclpy.init(args=['--ros-args', '-p', 'feedback_hz:=20.0'])
    bridge = AmrBridgeNode()
    amr = FakeAmr()
    control = FakeControl()
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (bridge, amr, control):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert control.client.wait_for_server(timeout_sec=5.0)
    yield bridge, amr, control
    amr.finish.set()
    # Let any running Patrol goal see the route result before stopping spin.
    wait_until(lambda: bridge._core.phase.value in ('idle', 'finished'))
    executor.shutdown(timeout_sec=5.0)
    for node in (bridge, amr, control):
        node.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=5.0)


def send_goal(control, robot_id='robot1'):
    goal = Patrol.Goal(command_id='cmd-1', robot_id=robot_id)
    future = control.client.send_goal_async(goal, feedback_callback=control.on_feedback)
    assert wait_until(future.done)
    return future.result()


def states(control):
    return [f.task_state for f in control.feedback]


def keep_token(control, stop):
    while not stop.is_set():
        control.token()
        time.sleep(0.2)


def test_full_patrol_with_stop_and_resume(system):
    _, amr, control = system
    handle = send_goal(control)
    assert handle.accepted
    assert wait_until(lambda: Patrol.Feedback.WAITING_FOR_TOKEN in states(control))
    assert amr.route_goals == 0

    stop = threading.Event()
    threading.Thread(target=keep_token, args=(control, stop), daemon=True).start()
    assert wait_until(lambda: amr.route_goals == 1)
    assert wait_until(lambda: Patrol.Feedback.PATROLLING in states(control))
    assert control.feedback[-1].token_valid

    control.command(PatrolCommand.MOVE_TO_SAFE_ZONE, 'c-safe')
    assert wait_until(lambda: amr.stops == 1)
    assert wait_until(lambda: control.feedback[-1].task_state == Patrol.Feedback.BLOCKED)
    assert control.feedback[-1].last_command_id == 'c-safe'
    assert control.feedback[-1].command_status == Patrol.Feedback.COMMAND_EXECUTING

    control.command(PatrolCommand.RESUME_PATROL, 'c-resume')
    assert wait_until(lambda: amr.resumes == 1)
    assert wait_until(lambda: Patrol.Feedback.RESUMING in states(control))

    amr.finish.set()
    result_future = handle.get_result_async()
    assert wait_until(result_future.done)
    stop.set()
    result = result_future.result().result
    assert result.outcome == Patrol.Result.SUCCEEDED


def test_token_revocation_stops_robot(system):
    _, amr, control = system
    handle = send_goal(control)
    stop = threading.Event()
    threading.Thread(target=keep_token, args=(control, stop), daemon=True).start()
    assert wait_until(lambda: amr.route_goals == 1)
    stop.set()
    control.token('')  # explicit revoke, as patrol_control does
    assert wait_until(lambda: amr.stops == 1)
    stop.clear()
    threading.Thread(target=keep_token, args=(control, stop), daemon=True).start()
    assert wait_until(lambda: amr.resumes == 1)
    stop.set()
    amr.finish.set()
    assert wait_until(handle.get_result_async().done)


def test_lease_expiry_stops_robot(system):
    _, amr, control = system
    send_goal(control)
    control.token()
    assert wait_until(lambda: amr.route_goals == 1)
    # No more tokens: 1 s lease + 0.5 s grace must expire.
    assert wait_until(lambda: amr.stops == 1, timeout=4.0)


def test_cancel_stops_in_place(system):
    _, amr, control = system
    handle = send_goal(control)
    control.token()
    assert wait_until(lambda: amr.route_goals == 1)
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done)
    result_future = handle.get_result_async()
    assert wait_until(result_future.done)
    result = result_future.result().result
    assert result.outcome == Patrol.Result.CANCELED
    assert result.reason_code == Patrol.Result.REASON_CONTROL_CANCELED
    assert amr.stops >= 1


def test_route_failure_aborts_goal(system):
    _, amr, control = system
    amr.succeed = False
    handle = send_goal(control)
    control.token()
    assert wait_until(lambda: amr.route_goals == 1)
    amr.finish.set()
    result_future = handle.get_result_async()
    assert wait_until(result_future.done)
    result = result_future.result().result
    assert result.outcome == Patrol.Result.FAILED
    assert result.reason_code == Patrol.Result.REASON_NAVIGATION_FAILED


def test_rejects_other_robot_goal(system):
    _, _, control = system
    assert not send_goal(control, robot_id='robot6').accepted


def test_new_goal_after_cancel_resumes_kept_route(system):
    _, amr, control = system
    handle = send_goal(control)
    control.token()
    assert wait_until(lambda: amr.route_goals == 1)
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done)
    assert wait_until(handle.get_result_async().done)
    second = send_goal(control)
    assert second.accepted
    control.token()
    assert wait_until(lambda: amr.resumes == 1)
    assert amr.route_goals == 1
    amr.finish.set()
    result_future = second.get_result_async()
    assert wait_until(result_future.done)
    assert result_future.result().result.outcome == Patrol.Result.SUCCEEDED
