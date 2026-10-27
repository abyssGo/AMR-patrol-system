"""
Bridge the Control Server's Patrol interface to the final AMR patrol.

Control side (patrol_interfaces 2.0):
  * ``/{robot}/patrol_action``   Patrol Action server (this node)
  * ``/{robot}/drive_token``     DriveToken subscription
  * ``/{robot}/patrol_command``  PatrolCommand subscription

AMR side (patrol_amr_safety.genius_patrol + event_check):
  * ``/{robot}/patrol_route``    NavigateToPose Action client.  Run
    genius_patrol with ``--ros-args -r patrol_action:=patrol_route`` so its
    fixed-route server does not collide with this node's Patrol server.
  * ``/{robot}/patrol_stop``     SetBool service, ``data=false`` stops in place
  * ``/{robot}/patrol_resume``   Bool topic, ``true`` resumes
"""

from __future__ import annotations

import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from patrol_interfaces.action import Patrol
from patrol_interfaces.msg import DriveToken
from patrol_interfaces.msg import PatrolCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .bridge_core import Action
from .bridge_core import BridgeCore
from .bridge_core import Outcome
from .bridge_core import Phase
from .bridge_core import Reason
from .bridge_core import Result
from .bridge_core import TaskState


def _reliable_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _token_qos() -> QoSProfile:
    # Matches patrol_control's DriveToken publisher (BEST_EFFORT, depth 3).
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=3,
    )


class AmrBridgeNode(Node):
    """Serve Control's Patrol Action and drive genius_patrol behind it."""

    def __init__(self) -> None:
        super().__init__('patrol_amr_bridge')
        self.robot_id = str(self.declare_parameter('robot_id', 'robot1').value)
        route_action = str(self.declare_parameter('route_action', 'patrol_route').value)
        self._feedback_period = 1.0 / float(
            self.declare_parameter('feedback_hz', 2.0).value)
        self._server_timeout = float(
            self.declare_parameter('server_timeout_sec', 5.0).value)
        self._token_grace = float(
            self.declare_parameter('token_grace_sec', 0.5).value)

        self._core = BridgeCore(self.robot_id)
        self._lock = threading.RLock()
        self._group = ReentrantCallbackGroup()
        self._route_handle: ClientGoalHandle | None = None
        self._final: Result | None = None
        self._token_expires_at = 0.0
        prefix = f'/{self.robot_id}'

        self._patrol_server = ActionServer(
            self, Patrol, f'{prefix}/patrol_action',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self._group,
        )
        self._route_client = ActionClient(
            self, NavigateToPose, f'{prefix}/{route_action}',
            callback_group=self._group)
        self._stop_client = self.create_client(
            SetBool, f'{prefix}/patrol_stop', callback_group=self._group)
        self._resume_pub = self.create_publisher(Bool, f'{prefix}/patrol_resume', 2)
        self.create_subscription(
            DriveToken, f'{prefix}/drive_token', self._on_token, _token_qos(),
            callback_group=self._group)
        self.create_subscription(
            PatrolCommand, f'{prefix}/patrol_command', self._on_command,
            _reliable_qos(10), callback_group=self._group)
        self.create_timer(0.1, self._check_token_lease, callback_group=self._group)
        self.get_logger().info(
            f'Bridge ready: {prefix}/patrol_action -> {prefix}/{route_action}; '
            f'stop={prefix}/patrol_stop resume={prefix}/patrol_resume')

    # -- Control -> bridge ---------------------------------------------------------
    def _on_goal(self, goal: Patrol.Goal) -> GoalResponse:
        with self._lock:
            if not self._core.accept_goal(goal.command_id, goal.robot_id):
                self.get_logger().warning(
                    f'Rejected Patrol goal command_id={goal.command_id!r} '
                    f'robot_id={goal.robot_id!r} phase={self._core.phase.value}')
                return GoalResponse.REJECT
            self._final = None
        self.get_logger().info(f'Accepted Patrol goal {goal.command_id}')
        return GoalResponse.ACCEPT

    def _on_token(self, message: DriveToken) -> None:
        with self._lock:
            if message.holder_robot_id == self.robot_id and message.token:
                lease = Duration.from_msg(message.lease_duration).nanoseconds / 1e9
                self._token_expires_at = time.monotonic() + max(lease, 0.0) + self._token_grace
            actions = self._core.on_token(
                message.holder_robot_id, message.token, int(message.sequence))
        self._run(actions)

    def _check_token_lease(self) -> None:
        with self._lock:
            if not self._core.token or time.monotonic() <= self._token_expires_at:
                return
            self.get_logger().warning('DriveToken lease expired; stopping in place')
            actions = self._core.on_token(self.robot_id, '', 0)
        self._run(actions)

    def _on_command(self, message: PatrolCommand) -> None:
        with self._lock:
            actions = self._core.on_command(
                message.robot_id, message.command_id, int(message.command))
            status = self._core.command_status
        self.get_logger().info(
            f'PatrolCommand {message.command_id} command={message.command} '
            f'-> {status.name} {[a.value for a in actions]}')
        self._run(actions)

    # -- bridge -> AMR -------------------------------------------------------------
    def _run(self, actions) -> None:
        for action in actions:
            if action is Action.SEND_ROUTE_GOAL:
                self._send_route_goal()
            elif action is Action.REQUEST_STOP:
                self._request_stop()
            elif action is Action.PUBLISH_RESUME:
                self._resume_pub.publish(Bool(data=True))
                self.get_logger().info('Published patrol resume')

    def _send_route_goal(self) -> None:
        with self._lock:
            reuse = self._route_handle is not None
            if reuse:
                # genius_patrol keeps its route goal after a Control cancel (it
                # only stopped in place), so a new Patrol goal resumes that route.
                self._set_final(self._core.on_route_goal_response(True))
        if reuse:
            self._resume_pub.publish(Bool(data=True))
            self.get_logger().info('Resuming the AMR route kept from the previous goal')
            return
        if not self._route_client.wait_for_server(timeout_sec=self._server_timeout):
            self._set_final(self._core.on_internal_error(
                'AMR patrol route server unavailable'))
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        future = self._route_client.send_goal_async(goal)
        future.add_done_callback(self._on_route_goal_response)

    def _on_route_goal_response(self, future) -> None:
        handle = future.result()
        accepted = bool(handle is not None and handle.accepted)
        with self._lock:
            self._route_handle = handle if accepted else None
            self._set_final(self._core.on_route_goal_response(accepted))
            paused = self._core.phase is Phase.PAUSED
        if accepted:
            handle.get_result_async().add_done_callback(self._on_route_result)
            if paused:
                # A stop arrived before genius_patrol accepted the route.
                self._request_stop()

    def _on_route_result(self, future) -> None:
        wrapped = future.result()
        succeeded = wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED
        with self._lock:
            self._route_handle = None
            self._set_final(self._core.on_route_result(succeeded))

    def _request_stop(self) -> None:
        if not self._stop_client.wait_for_service(timeout_sec=self._server_timeout):
            self.get_logger().error('patrol_stop service unavailable')
            return
        self.get_logger().info('Requesting patrol stop (data=false)')
        future = self._stop_client.call_async(SetBool.Request(data=False))
        future.add_done_callback(self._on_stop_response)

    def _on_stop_response(self, future) -> None:
        response = future.result()
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            self.get_logger().warning(f'Patrol stop not confirmed: {message}')
        else:
            self.get_logger().info(f'Patrol stop confirmed: {response.message}')

    def _set_final(self, result: Result | None) -> None:
        if result is not None and self._final is None:
            self._final = result

    # -- Patrol execute loop ---------------------------------------------------------
    def _execute(self, goal_handle) -> Patrol.Result:
        resumed_at = 0.0
        last_state = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._lock:
                    result, actions = self._core.on_cancel()
                    route = self._route_handle
                self._run(actions)
                if route is not None:
                    route.cancel_goal_async()  # genius_patrol may reject this
                goal_handle.canceled()
                return self._to_msg(result)
            with self._lock:
                final = self._final
                feedback = self._core.feedback()
            if final is not None:
                if final.outcome is Outcome.SUCCEEDED:
                    goal_handle.succeed()
                elif final.outcome is Outcome.CANCELED:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._to_msg(final)

            state = feedback.task_state
            if last_state is TaskState.BLOCKED and state is TaskState.PATROLLING:
                resumed_at = time.monotonic()
            if state is TaskState.PATROLLING and time.monotonic() - resumed_at < 1.0:
                state = TaskState.RESUMING
            last_state = feedback.task_state

            message = Patrol.Feedback()
            message.task_state = int(state)
            message.token_valid = feedback.token_valid
            message.accepted_token_sequence = feedback.accepted_token_sequence
            message.last_command_id = feedback.last_command_id
            message.command_status = int(feedback.command_status)
            goal_handle.publish_feedback(message)
            time.sleep(self._feedback_period)

        goal_handle.abort()
        return self._to_msg(Result(Outcome.FAILED, Reason.INTERNAL_ERROR, 'shutdown'))

    @staticmethod
    def _to_msg(result: Result) -> Patrol.Result:
        message = Patrol.Result()
        message.outcome = int(result.outcome)
        message.reason_code = int(result.reason_code)
        message.reason = result.reason
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AmrBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
