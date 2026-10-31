"""
ROS 2 Control Server node for the patrol_interfaces 2.1 contract.

설계도 01·02 페이지의 관제 흐름:
- 순찰 요청 → WAITING_FOR_TOKEN 수신 → DriveToken 발행 → 임무 중 갱신
- CCTV permit 전이 → MOVE_TO_SAFE_ZONE / RESUME_PATROL
- FIRE 확정 → fire_hold (운영자 명시적 해제 전까지 새 순찰 금지)
- Patrol Result → Token 회수 → 도킹하였는가? → 다음 AMR에 토큰 (자동 교대)
- Safety Arbiter: /control/estop 단일 발행자 (CR 9/8)

운영자 명령은 터미널 ROS 서비스로 받는다:
  /control/start_patrol     std_srvs/Trigger   교대 순찰 시작, 제외된 로봇 복귀
  /control/cancel_patrol    std_srvs/Trigger   진행 중 순찰 취소, 교대 정지
  /control/clear_fire_hold  std_srvs/Trigger   화재 보류 해제
  /control/set_estop        patrol_interfaces/SetEStop
  /control/resume           std_srvs/Trigger   EStop 해제 뒤 보류된 로봇 재개

관제가 모르는 임무(관제 재시작 전 임무 등)가 AMR에 남아 있으면 그 임무를 취소하고,
끝난 것이 확인될 때까지 아무 로봇도 출동시키지 않는다.
"""

from __future__ import annotations

from functools import partial
import time
from time import monotonic

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from patrol_control import control_state
from patrol_control.control_core import ControlCore
from patrol_control.control_core import ControlStateError
from patrol_control.control_core import REASON_LOW_BATTERY

from patrol_interfaces.action import Patrol
from patrol_interfaces.msg import DriveToken, EStop, PatrolCommand
from patrol_interfaces.srv import SetEStop

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


DEFAULT_ROBOT_IDS = ('robot1', 'robot6')
ESTOP_PUBLISH_PERIOD = 1.0      # s, 대상 하나당 재발행 주기 (대상 사이 간격 = 주기 / 대상 수)
DISPATCH_PERIOD = 1.0           # s, 교대 출동 판단 · 감시 주기
STARTUP_GRACE = 10.0            # s, 시작 직후 AMR 발견 · 남은 임무 확인 시간
GOAL_RESPONSE_TIMEOUT = 10.0    # s, Goal 수락 응답 대기
SERVER_LOST_TIMEOUT = 10.0      # s, 실행 중 AMR Action server가 이만큼 사라지면 실패 처리
RESUME_RESEND_WINDOW = 10.0     # s, 재개 명령을 BLOCKED가 풀릴 때까지 다시 보내는 기간
CANCEL_RETRY_PERIOD = 5.0       # s, 모르는 임무가 남아 있으면 취소 요청을 다시 보낸다
OWN_GOAL_MEMORY = 32            # 최근에 보낸 Goal UUID 기억 개수
ACTIVE_GOAL_STATES = (
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
)


def _reliable_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _token_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=3,
        deadline=Duration(seconds=0.2),
        lifespan=Duration(seconds=0.5),
    )


def estop_qos() -> QoSProfile:
    """CR 9/8: RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _action_status_qos() -> QoSProfile:
    """Match rclpy action servers' status publisher (transient local)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class PatrolControlNode(Node):
    """Coordinate one active Patrol Action, robot handover, tokens and EStop."""

    def __init__(self) -> None:
        super().__init__('patrol_control_node')

        robot_ids = tuple(
            self.declare_parameter('robot_ids', list(DEFAULT_ROBOT_IDS)).value
        )
        self._initial_robot_id = self.declare_parameter(
            'initial_robot_id', robot_ids[0]
        ).value
        self._auto_start = bool(
            self.declare_parameter('auto_start', False).value
        )
        token_publish_hz = float(
            self.declare_parameter('token_publish_hz', 5.0).value
        )
        self._token_lease_seconds = float(
            self.declare_parameter('token_lease_seconds', 1.0).value
        )
        if token_publish_hz <= 0.0:
            raise ValueError('token_publish_hz must be positive')
        if self._token_lease_seconds <= 0.0:
            raise ValueError('token_lease_seconds must be positive')
        if self._initial_robot_id not in robot_ids:
            raise ValueError('initial_robot_id must be in robot_ids')

        self._core = ControlCore(robot_ids, initial_robot_id=self._initial_robot_id)
        saved = control_state.load_state()
        if saved:
            self._core.restore(saved)
            self.get_logger().warning(
                'control state restored: fire_hold=%s excluded=%s unknown_goal=%s '
                'estop_pending=%s'
                % (self._core.fire_hold, sorted(self._core.excluded),
                   sorted(self._core.restart_wait), sorted(self._core.estop_pending))
            )
        if self._auto_start:
            self._core.rotation_running = True

        self._action_clients = {
            robot_id: ActionClient(self, Patrol, f'/{robot_id}/patrol_action')
            for robot_id in robot_ids
        }
        # 관제가 모르는 임무를 전부 취소하는 데 쓴다(goal_id · stamp 0 = 모든 goal).
        self._cancel_clients = {
            robot_id: self.create_client(
                CancelGoal, f'/{robot_id}/patrol_action/_action/cancel_goal')
            for robot_id in robot_ids
        }
        self._command_publishers = {
            robot_id: self.create_publisher(
                PatrolCommand, f'/{robot_id}/patrol_command', _reliable_qos(10))
            for robot_id in robot_ids
        }
        self._token_publishers = {
            robot_id: self.create_publisher(
                DriveToken, f'/{robot_id}/drive_token', _token_qos())
            for robot_id in robot_ids
        }
        self._estop_publisher = self.create_publisher(
            EStop, '/control/estop', estop_qos()
        )
        self._permit_subscription = self.create_subscription(
            Bool, '/vision/cctv/patrol_allowed', self._on_patrol_allowed, _reliable_qos(1),
        )
        for robot_id in robot_ids:
            self.create_subscription(
                BatteryState, f'/{robot_id}/battery_state',
                partial(self._on_battery, robot_id=robot_id), qos_profile_sensor_data,
            )
            self.create_subscription(
                GoalStatusArray, f'/{robot_id}/patrol_action/_action/status',
                partial(self._on_goal_status, robot_id=robot_id), _action_status_qos(),
            )

        self._goal_handles = {}
        self._goal_generation = 0
        self._pending_goal = None          # (robot_id, generation, sent_at)
        self._cancel_after_accept = False  # Goal 응답 전에 취소 요청이 온 경우
        self._server_lost_since = {}
        self._cancel_all_sent = {}         # robot_id -> 마지막 전체 취소 요청 시각
        self._own_goal_ids = []            # 관제가 보낸 Goal UUID (최근 것만)
        self._status_seen = set()          # status를 한 번이라도 받은 로봇
        self._last_state = {}              # robot_id -> 마지막 Feedback task_state
        self._last_permit = None           # 마지막으로 받은 patrol_allowed 값 (바뀔 때만 로그)
        self._resume_resend = None         # (intent, until): EStop 재개 명령 재전송
        self._started_at = monotonic()
        self._last_server_wait_log = 0.0
        self._last_blocker = None
        self._estop_index = 0

        self._token_timer = self.create_timer(1.0 / token_publish_hz, self._publish_token_frames)
        self._estop_timer = self.create_timer(
            ESTOP_PUBLISH_PERIOD / len(self._core.estop_targets), self._publish_next_estop)
        self._dispatch_timer = self.create_timer(DISPATCH_PERIOD, self._on_dispatch_timer)

        self.create_service(Trigger, '/control/start_patrol', self._srv_start_patrol)
        self.create_service(Trigger, '/control/cancel_patrol', self._srv_cancel_patrol)
        self.create_service(Trigger, '/control/clear_fire_hold', self._srv_clear_fire_hold)
        self.create_service(SetEStop, '/control/set_estop', self._srv_set_estop)
        self.create_service(Trigger, '/control/resume', self._srv_resume)

        for target in self._core.estop_targets:
            self._publish_estop(target)
        self._save_state()
        self.get_logger().info(
            'patrol control ready: robots=%s auto_start=%s initial_robot=%s'
            % (','.join(robot_ids), self._auto_start, self._initial_robot_id)
        )

    # ── 순찰 시작 · 취소 ────────────────────────────────────────────────────
    def start_patrol(self, robot_id: str) -> bool:
        """Send a Patrol Goal to one robot (used by the rotation)."""
        client = self._action_clients.get(robot_id)
        if client is None:
            self.get_logger().error(f'unknown robot_id: {robot_id}')
            return False
        if not client.server_is_ready():
            self._log_server_wait(robot_id)
            return False
        try:
            intent = self._core.prepare_goal(robot_id)
        except (ControlStateError, ValueError) as exc:
            self.get_logger().warning(f'patrol start rejected: {exc}')
            return False

        goal = Patrol.Goal()
        goal.command_id = intent.command_id
        goal.robot_id = intent.robot_id
        self._goal_generation += 1
        generation = self._goal_generation
        self._pending_goal = (robot_id, generation, monotonic())
        self._cancel_after_accept = False
        future = client.send_goal_async(
            goal,
            feedback_callback=partial(
                self._on_patrol_feedback, robot_id=robot_id, generation=generation),
        )
        future.add_done_callback(
            partial(self._on_goal_response, robot_id=robot_id, generation=generation)
        )
        self._save_state()
        self.get_logger().info(
            f'Patrol Goal sent: robot={robot_id} command={intent.command_id}'
        )
        return True

    def cancel_active_patrol(self) -> bool:
        """Revoke the Token immediately and cancel the active (or pending) Patrol Action."""
        robot_id = self._core.active_robot_id
        if robot_id is None:
            return False
        self._core.revoke_drive(robot_id)
        self._publish_token_frames()
        goal_handle = self._goal_handles.get(robot_id)
        if goal_handle is None:
            # Goal 응답 전: 수락되는 즉시 취소한다.
            self._cancel_after_accept = True
            self.get_logger().warning(
                f'Patrol cancel requested before acceptance: robot={robot_id}')
            return True
        goal_handle[0].cancel_goal_async()
        self.get_logger().warning(
            f'Patrol cancel requested; DriveToken revoked: robot={robot_id}'
        )
        return True

    def clear_fire_hold(self) -> None:
        """Clear the fire hold on an explicit operator request."""
        self._core.clear_fire_hold()
        self._save_state()
        self.get_logger().warning('fire hold cleared by explicit operator call')

    # ── 주기 감시 · 교대 ────────────────────────────────────────────────────
    def _on_dispatch_timer(self) -> None:
        now = monotonic()
        self._check_unknown_goals(now)
        self._check_pending_goal(now)
        self._check_server_lost(now)
        self._resend_resume(now)
        self._try_dispatch()

    def _resend_resume(self, now: float) -> None:
        """
        Repeat the operator resume (same command_id) while the robot still reports BLOCKED.

        The EStop release and the command travel on different topics; if the
        command is processed before the release frame the AMR rejects it.
        The AMR applies a command_id only once, so repeats are harmless.
        """
        if self._resume_resend is None:
            return
        intent, until = self._resume_resend
        if (now > until or self._core.active_robot_id != intent.robot_id
                or self._last_state.get(intent.robot_id) != Patrol.Feedback.BLOCKED):
            self._resume_resend = None
            return
        if not self._core.estop_active_for(intent.robot_id):
            self._publish_command(intent)

    def _check_unknown_goals(self, now: float) -> None:
        """Cancel goals control does not own; clear the wait once they are gone."""
        for robot_id in sorted(self._core.restart_wait):
            client = self._cancel_clients[robot_id]
            last = self._cancel_all_sent.get(robot_id)
            if (last is None or now - last > CANCEL_RETRY_PERIOD) and client.service_is_ready():
                future = client.call_async(CancelGoal.Request())   # zero goal_info = cancel all
                future.add_done_callback(partial(self._on_cancel_all, robot_id=robot_id))
                self._cancel_all_sent[robot_id] = now
                self.get_logger().error(
                    f'ALERT {robot_id}: goal unknown to control (restart?) canceled; '
                    'robot excluded until /control/start_patrol')
            if now - self._started_at <= STARTUP_GRACE:
                continue
            if (not self._action_clients[robot_id].server_is_ready()
                    and not client.service_is_ready()):
                # AMR 쪽 mission_supervisor가 없으면 실행 중인 임무도 없다.
                self._clear_unknown(robot_id, 'offline: no unknown goal to wait for')
            elif robot_id not in self._status_seen:
                # rclpy Action server는 goal을 받은 적이 없으면 status를 내지 않는다.
                self._clear_unknown(robot_id, 'server up without any goal status')

    def _on_cancel_all(self, future, *, robot_id: str) -> None:
        try:
            response = future.result()
        except Exception as exc:  # rclpy Future transports middleware errors.
            self.get_logger().warning(f'{robot_id} cancel-all failed: {exc}')
            return
        if robot_id in self._core.restart_wait and not response.goals_canceling:
            self._clear_unknown(robot_id, 'no goal was running')

    def _clear_unknown(self, robot_id: str, why: str) -> None:
        if robot_id not in self._core.restart_wait:
            return
        self._core.clear_restart_wait(robot_id)
        self._cancel_all_sent.pop(robot_id, None)
        self._save_state()
        self.get_logger().warning(
            f'{robot_id}: {why}; it stays excluded until /control/start_patrol')

    def _check_pending_goal(self, now: float) -> None:
        if self._pending_goal is None:
            return
        robot_id, _, sent_at = self._pending_goal
        if now - sent_at > GOAL_RESPONSE_TIMEOUT:
            self.get_logger().error(
                f'Patrol Goal response timeout: robot={robot_id}; slot released '
                '(a late acceptance will be canceled)')
            self._pending_goal = None
            self._finish_core_goal(robot_id)

    def _check_server_lost(self, now: float) -> None:
        for robot_id in list(self._goal_handles):
            if self._action_clients[robot_id].server_is_ready():
                self._server_lost_since.pop(robot_id, None)
                continue
            since = self._server_lost_since.setdefault(robot_id, now)
            if now - since > SERVER_LOST_TIMEOUT:
                self.get_logger().error(
                    f'ALERT {robot_id}: Patrol Action server lost during the patrol')
                handle, _ = self._goal_handles.pop(robot_id)
                self._server_lost_since.pop(robot_id, None)
                self._finish_core_goal(
                    robot_id, Patrol.Result.FAILED, Patrol.Result.REASON_INTERNAL_ERROR)
                # 다시 연결됐을 때 그 Goal이 아직 돌고 있으면 모르는 임무로 잡아 취소한다.
                uuid = bytes(handle.goal_id.uuid)
                if uuid in self._own_goal_ids:
                    self._own_goal_ids.remove(uuid)
                self._core.mark_unknown_goal(robot_id)
                self._save_state()

    def _try_dispatch(self) -> None:
        """설계도 01: 다음 토큰 발급이 보류된 상태가 아니면 다음 AMR에 순찰을 맡긴다."""
        if self._pending_goal is not None:
            return
        if monotonic() - self._started_at < STARTUP_GRACE:
            self._log_blocker('startup: checking robots for unknown goals')
            return
        blocker = self._core.dispatch_blocker()
        if blocker:
            self._log_blocker(blocker)
            return
        robot_id = self._core.select_next_robot(self._server_ready)
        if robot_id is None:
            self._log_blocker(
                'no robot ready (excluded=%s charging=%s estop_pending=%s battery=%s)'
                % (sorted(self._core.excluded), sorted(self._core.charging_hold),
                   sorted(self._core.estop_pending), self._core.battery)
            )
            return
        self._last_blocker = None
        self.start_patrol(robot_id)

    def _server_ready(self, robot_id: str) -> bool:
        ready = self._action_clients[robot_id].server_is_ready()
        if not ready:
            self._log_server_wait(robot_id)
        return ready

    # ── 운영자 서비스 ───────────────────────────────────────────────────────
    def _srv_start_patrol(self, request, response):
        self._core.start_rotation()
        self._save_state()
        self.get_logger().warning('operator: patrol rotation started')
        response.success = True
        blocker = self._core.dispatch_blocker()
        response.message = (
            f'rotation started; waiting: {blocker}' if blocker and blocker != 'patrol active'
            else 'rotation started'
        )
        self._try_dispatch()
        return response

    def _srv_cancel_patrol(self, request, response):
        self._core.stop_rotation()
        canceled = self.cancel_active_patrol()
        self._save_state()
        self.get_logger().warning(f'operator: patrol canceled (active goal={canceled})')
        response.success = True
        response.message = 'active patrol canceled' if canceled else 'rotation stopped'
        return response

    def _srv_clear_fire_hold(self, request, response):
        was = self._core.fire_hold
        self.clear_fire_hold()
        response.success = True
        response.message = 'fire hold cleared' if was else 'fire hold was not active'
        return response

    def _srv_set_estop(self, request, response):
        try:
            self._core.set_estop(request.target_robot_id, request.active, request.reason)
        except ValueError as exc:
            response.success, response.message = False, str(exc)
            return response
        self._save_state()
        self._publish_estop(request.target_robot_id)
        text = (f'operator: EStop target={request.target_robot_id} active={request.active} '
                f'reason={request.reason}')
        if request.active:
            self.get_logger().error(text)
        else:
            self.get_logger().warning(text)
        response.success = True
        response.message = (
            'estop active' if request.active
            else 'estop released; call /control/resume to let the robots move again'
        )
        return response

    def _srv_resume(self, request, response):
        intent, released = self._core.resume()
        self._save_state()
        if intent is not None:
            self._publish_command(intent)
            self._resume_resend = (intent, monotonic() + RESUME_RESEND_WINDOW)
        if not released:
            response.success = False
            response.message = 'no robot is waiting for resume (or its EStop is still active)'
            return response
        self.get_logger().warning(f'operator: resume after EStop for {list(released)}')
        response.success = True
        response.message = (
            f'resumed {list(released)}'
            + (f'; {intent.command.name} sent to {intent.robot_id}' if intent else '')
        )
        return response

    # ── 입력 콜백 ───────────────────────────────────────────────────────────
    def _on_patrol_allowed(self, message: Bool) -> None:
        allowed = bool(message.data)
        commands = self._core.observe_permit(allowed)
        if allowed != self._last_permit:
            self._last_permit = allowed
            self.get_logger().info(f'patrol_allowed={allowed}')
        for intent in commands:
            self._publish_command(intent)

    def _on_battery(self, message: BatteryState, *, robot_id: str) -> None:
        percentage = message.percentage
        if percentage != percentage:  # NaN = unknown
            percentage = None
        before = robot_id in self._core.charging_hold
        self._core.observe_battery(robot_id, percentage)
        after = robot_id in self._core.charging_hold
        if before != after:
            self._save_state()
            self.get_logger().warning(
                f'{robot_id} battery {percentage}: '
                + ('charging hold (dispatch after 80%)' if after else 'ready for dispatch')
            )

    def _on_goal_status(self, message: GoalStatusArray, *, robot_id: str) -> None:
        self._status_seen.add(robot_id)
        pending_here = (self._pending_goal is not None and self._pending_goal[0] == robot_id)
        if pending_here:
            return   # 방금 보낸 Goal의 UUID는 응답 전이라 아직 모른다
        active = any(
            s.status in ACTIVE_GOAL_STATES
            and bytes(s.goal_info.goal_id.uuid) not in self._own_goal_ids
            for s in message.status_list
        )
        if active and robot_id not in self._core.restart_wait:
            # 관제가 보내지 않은(또는 잊은) 임무가 실행 중이다.
            self._core.mark_unknown_goal(robot_id)
            self._save_state()
        elif not active:
            self._clear_unknown(robot_id, 'unknown goal ended')

    def _publish_command(self, intent) -> None:
        command = PatrolCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.command_id = intent.command_id
        command.robot_id = intent.robot_id
        command.command = int(intent.command)
        self._command_publishers[intent.robot_id].publish(command)
        self.get_logger().info(
            'PatrolCommand published: robot=%s command=%s id=%s'
            % (intent.robot_id, intent.command.name, intent.command_id)
        )

    def _publish_token_frames(self) -> None:
        """Publish the current grant or explicit revocation for each robot."""
        now = self.get_clock().now()
        total_ns = int(self._token_lease_seconds * 1_000_000_000)
        for frame in self._core.token_frames():
            message = DriveToken()
            message.header.stamp = now.to_msg()
            message.token = frame.token
            message.holder_robot_id = frame.robot_id
            message.lease_duration.sec = total_ns // 1_000_000_000
            message.lease_duration.nanosec = total_ns % 1_000_000_000
            message.sequence = frame.sequence
            self._token_publishers[frame.robot_id].publish(message)

    def _publish_next_estop(self) -> None:
        """Republish one target per tick so subscribers with depth 1 see every target."""
        targets = self._core.estop_targets
        self._publish_estop(targets[self._estop_index % len(targets)])
        self._estop_index += 1

    def _publish_estop(self, target: str) -> None:
        frame = self._core.estop_frame(target, int(time.time() * 1000))
        message = EStop()
        message.header.stamp = self.get_clock().now().to_msg()
        message.target_robot_id = frame.target_robot_id
        message.active = frame.active
        message.reason = frame.reason
        message.sequence = frame.sequence
        self._estop_publisher.publish(message)

    # ── Patrol Action 응답 ──────────────────────────────────────────────────
    def _current(self, robot_id: str, generation: int) -> bool:
        return generation == self._goal_generation and self._core.active_robot_id == robot_id

    def _on_goal_response(self, future, *, robot_id: str, generation: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy Future transports middleware errors.
            goal_handle = None
            self.get_logger().error(
                f'Patrol Goal request failed: robot={robot_id} error={exc}'
            )
        if not self._current(robot_id, generation) or self._pending_goal is None:
            # 응답 timeout으로 이미 포기한 Goal: 늦게 수락됐으면 바로 취소한다.
            if goal_handle is not None and goal_handle.accepted:
                # 자기 Goal이므로 '모르는 임무'로 오인하지 않게 UUID를 기록하고 취소한다.
                self._own_goal_ids.append(bytes(goal_handle.goal_id.uuid))
                del self._own_goal_ids[:-OWN_GOAL_MEMORY]
                goal_handle.cancel_goal_async()
                self.get_logger().error(f'late Patrol Goal acceptance canceled: {robot_id}')
            return
        self._pending_goal = None
        if goal_handle is None or not goal_handle.accepted:
            if goal_handle is not None:
                self.get_logger().warning(f'Patrol Goal rejected: robot={robot_id}')
            self._finish_core_goal(robot_id)
            return

        self._goal_handles[robot_id] = (goal_handle, generation)
        self._own_goal_ids.append(bytes(goal_handle.goal_id.uuid))
        del self._own_goal_ids[:-OWN_GOAL_MEMORY]
        self._core.mark_goal_accepted(robot_id)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._on_patrol_result, robot_id=robot_id, generation=generation)
        )
        self.get_logger().info(
            f'Patrol Goal accepted; waiting for WAITING_FOR_TOKEN: {robot_id}'
        )
        if self._cancel_after_accept:
            self._cancel_after_accept = False
            self.cancel_active_patrol()

    def _on_patrol_feedback(self, feedback_message, *, robot_id: str, generation: int) -> None:
        if not self._current(robot_id, generation):
            return
        feedback = feedback_message.feedback
        self._last_state[robot_id] = feedback.task_state
        effect = self._core.observe_feedback(
            robot_id,
            task_state=feedback.task_state,
            event_type=feedback.event_type,
            event_id=feedback.event_id,
        )
        if not effect.accepted:
            self.get_logger().warning(
                f'ignored Patrol Feedback: robot={robot_id} state={feedback.task_state}'
            )
            return
        if effect.token_granted:
            self.get_logger().info(
                f'DriveToken granted after WAITING_FOR_TOKEN: robot={robot_id}'
            )
        if effect.fire_hold_activated:
            self._save_state()
            self.get_logger().error(
                f'FIRE_HOLD activated: robot={robot_id} event={feedback.event_id}'
            )
        if feedback.task_state == Patrol.Feedback.WAYPOINT_REACHED:
            self.get_logger().info(
                f'waypoint reached: robot={robot_id} waypoint={feedback.current_waypoint_id}'
            )

    def _on_patrol_result(self, future, *, robot_id: str, generation: int) -> None:
        if not self._current(robot_id, generation):
            return   # 이미 서버 상실로 정리한 Goal
        outcome = reason_code = None
        try:
            wrapped = future.result()
            result = wrapped.result
            outcome, reason_code = result.outcome, result.reason_code
            self.get_logger().info(
                'Patrol Result: robot=%s outcome=%d reason_code=%d reason=%s'
                % (robot_id, result.outcome, result.reason_code, result.reason)
            )
        except Exception as exc:  # rclpy Future transports middleware errors.
            self.get_logger().error(
                f'Patrol Result failed: robot={robot_id} error={exc}'
            )
            outcome, reason_code = Patrol.Result.FAILED, Patrol.Result.REASON_INTERNAL_ERROR
        finally:
            self._goal_handles.pop(robot_id, None)
            self._server_lost_since.pop(robot_id, None)
            self._finish_core_goal(robot_id, outcome, reason_code or 0)

    def _finish_core_goal(self, robot_id: str, outcome=None, reason_code: int = 0) -> None:
        if self._core.active_robot_id != robot_id:
            return
        verdict = self._core.finish_goal(robot_id, outcome, reason_code)
        if verdict == 'excluded':
            self.get_logger().error(
                f'ALERT {robot_id} did not dock (reason_code={reason_code}); excluded from '
                'rotation until /control/start_patrol. Remaining robots continue.'
            )
        elif verdict == 'docked' and reason_code == REASON_LOW_BATTERY:
            self.get_logger().warning(f'{robot_id} returned on low battery; charging hold')
        elif verdict == 'canceled':
            self.get_logger().warning(f'{robot_id} patrol canceled; rotation stopped')
        self._save_state()

    # ── 기타 ────────────────────────────────────────────────────────────────
    def _save_state(self) -> None:
        try:
            control_state.save_state(self._core.snapshot())
        except OSError as exc:
            self.get_logger().error(f'control state not saved: {exc}')

    def _log_blocker(self, blocker: str) -> None:
        if blocker != self._last_blocker:
            self.get_logger().info(f'next patrol on hold: {blocker}')
            self._last_blocker = blocker

    def _log_server_wait(self, robot_id: str) -> None:
        now = monotonic()
        if now - self._last_server_wait_log >= 5.0:
            self.get_logger().warning(
                f'Patrol Action server not ready: /{robot_id}/patrol_action'
            )
            self._last_server_wait_log = now


def main(args=None) -> None:
    """Run the patrol Control Server node."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PatrolControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_active_patrol()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
