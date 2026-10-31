#!/usr/bin/env python3
"""
Mission supervisor: AMR Patrol Action Server (A-1_지능1_시스템_설계도 01·02·03 페이지).

관제(PC 3)의 `/{robot}/patrol_action`(patrol_interfaces/Patrol) Goal을 받아 순찰을
수행하고, 진행 상태를 Feedback으로, 끝을 Result로 돌려준다.

  Goal 수락 → WAITING_FOR_TOKEN → (유효한 DriveToken) → UNDOCKING → INITIAL_POSE_READY
  → PATROLLING / WAYPOINT_REACHED … → DOCKING(귀환 · 도킹) → Result

중단 처리 (모든 대기 구간에 같은 규칙을 적용한다):
- EStop(/control/estop, CR 9/8): 활성 프레임을 받는 즉시 래치한다. 진행 중인 주행 ·
  회전 · 도킹 · 감지 동작을 취소하고 BLOCKED. 래치는 EStop 해제 + 유효한 토큰 +
  해제 뒤 관제가 보낸 재개 명령(RESUME_PATROL 또는 MOVE_TO_SAFE_ZONE)이 모두 있을 때만
  풀린다. 그래서 짧게 걸렸다 풀린 EStop도 운영자 재개 없이는 움직이지 않는다.
- DriveToken 회수·만료: 정지 후 BLOCKED. 토큰이 돌아오면 재개,
  TOKEN_TIMEOUT 안에 돌아오지 않으면 FAILED(TOKEN_EXPIRED).
- PatrolCommand MOVE_TO_SAFE_ZONE(1): 정지 → 가장 가까운 안전구역 → 대기 →
  RESUME_PATROL(2) → RESUMING → 멈춘 단계부터 재개. 감지 처리 중이어도 감지를 취소하고 간다.
- 감지 노드의 정지 요청(patrol_stop, SetBool data=false, 순찰 경로 주행 중에만 수락):
  정지 후 응답 → DETECTION_PROCESSING → DetectEvent Goal(정렬 · 검증) → 확정이면
  DETECTION_CONFIRMED(event_id = detection_id) 후 저장 결과(patrol_resume)를 기다려 재개.
- 경유지 이동 실패: 1번 더 시도 → 그 경유지 건너뜀(결과 사유에 기록).
  연속 2개 경유지를 건너뛰면 바로 귀환한다.
- 배터리 15% 미만: 귀환 · 도킹 후 FAILED(LOW_BATTERY). 안전구역 대기 중이면
  차량이 나간 뒤(RESUME_PATROL) 귀환한다.
- 도킹(CR 9/9): dock_status 접속 2초 유지 + 도킹 뒤 받은 battery_state가 충전 중/완충.
  시도 1: 그 자리 도킹, 2: 도크 앞 재접근 후 도킹, 3: 제자리 회전(위치 재추정) 후
  재접근 · 도킹. 모두 실패하면 자기 도크 앞에 정차하고 FAILED(DOCKING_FAILED).
  도크 앞까지 못 가면 가장 가까운 안전구역에 정차한다(그것도 안 되면 그 자리).
- Cancel: 진행 중인 동작을 모두 취소하고 정지 후 CANCELED(CONTROL_CANCELED).

순찰 경로 · 360° 회전 값은 시연 경로와 같고, 정지 확인 · 안전구역은
move_to_safetyzone 구현을 쓴다.
"""

import math
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from nav2_simple_commander.robot_navigator import TaskResult
from patrol_amr_interfaces.action import DetectEvent
from patrol_interfaces.action import Patrol
from patrol_interfaces.msg import DriveToken, EStop, PatrolCommand
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

if __package__:
    from .dock_check import DOCK_ATTEMPT_TIMEOUT, DockCheck
    from .drive_token_check import DriveTokenCheck, TokenFrame, token_qos
    from .estop_check import ESTOP_TOPIC, EStopCheck, EStopFrame, estop_qos
    from .move_to_safetyzone import Evacuation, SPIN, TEST_DEFAULTS
    from .site_config import load_site
else:
    from dock_check import DOCK_ATTEMPT_TIMEOUT, DockCheck
    from drive_token_check import DriveTokenCheck, TokenFrame, token_qos
    from estop_check import ESTOP_TOPIC, EStopCheck, EStopFrame, estop_qos
    from move_to_safetyzone import Evacuation, SPIN, TEST_DEFAULTS
    from site_config import load_site

F = Patrol.Feedback
R = Patrol.Result

# 순찰 경로 · 도크 앞 · 언도킹 직후 초기 위치 · 안전구역 좌표는 config/site.yaml에 있다
# (site_file 파라미터로 다른 파일 지정 가능).
D = TurtleBot4Directions
SPIN_TIME_ALLOWANCE = 20               # s
TOKEN_TIMEOUT = 30.0                   # s, BLOCKED 후 토큰이 돌아오지 않으면 FAILED(TOKEN_EXPIRED)
DETECT_MIN_CONFIDENCE = 0.70           # 감지 노드 확정 기준과 같음
DETECT_TIMEOUT = 60.0                  # s, DetectEvent 결과 대기
DETECT_CANCEL_TIMEOUT = 30.0           # s, 취소 후 감지 노드가 정렬 회전을 멈출 때까지
STORE_TIMEOUT = 60.0                   # s, 확정 후 저장 결과(patrol_resume) 대기
FEEDBACK_PERIOD = 0.5                  # s
STEP_RETRIES = 1                       # 이동 실패 시 다시 시도하는 횟수
MAX_CONSECUTIVE_SKIPS = 2              # 연속으로 건너뛴 경유지가 이만큼이면 바로 귀환
DOCK_ATTEMPTS = 3
LOW_BATTERY = 0.15                     # 순찰 중 이 아래면 귀환 · 도킹
UNDOCK_TIMEOUT = 60.0                  # s
DOCK_STATUS_WAIT = 5.0                 # s, 시작 시 dock_status 첫 수신 대기
DOCK_STATUS_SETTLE = 3.0               # s, Dock 결과 뒤 dock_status가 따라올 때까지
CREATE_CANCEL_WAIT = 5.0               # s, 취소한 Dock/Undock이 실제로 끝날 때까지
STOP_CONFIRM_ATTEMPTS = 3              # 정지 확인을 다시 하는 횟수(넘으면 BLOCKED로 계속 확인)


def build_route(navigator, route):
    """Return [(waypoint_id or None, PoseStamped or SPIN)] for the site route."""
    steps = []
    for step in route:
        if step == SPIN:
            steps.append((None, SPIN))
        else:
            waypoint_id, xy, direction = step
            steps.append((waypoint_id, navigator.getPoseStamped(list(xy), D[direction])))
    return steps


class PatrolAbort(Exception):
    """End the current goal with a Patrol Result outcome and reason code."""

    def __init__(self, outcome, reason_code, reason):
        super().__init__(reason)
        self.outcome, self.reason_code, self.reason = outcome, reason_code, reason


class _SafetyStop(Exception):
    """An EStop was latched during a wait."""


class _TokenLost(Exception):
    """The DriveToken became invalid during a wait."""


class _Evacuate(Exception):
    """MOVE_TO_SAFE_ZONE arrived during a wait."""


class _EvacSettings:
    """Settings object in the shape move_to_safetyzone.Evacuation expects."""

    def __init__(self, robot_id, safe_zones):
        self.odom_topic = f'/{robot_id}/odom'
        self.base_frame = 'base_link'
        self.safe = safe_zones
        for name, value in TEST_DEFAULTS.items():
            setattr(self, name.replace('-', '_'), value)


def _dock_status_qos():
    # CR 9/9: BEST_EFFORT / VOLATILE / KEEP_LAST(5)
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5)


class MissionSupervisor(Node):
    """Patrol Action Server for one AMR namespace."""

    def __init__(self):
        super().__init__('mission_supervisor')
        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            raise RuntimeError('mission_supervisor must run in a robot namespace')
        site_file = self.declare_parameter('site_file', '').value
        self.site = load_site(site_file or None)
        if self.robot_id not in self.site.docks:
            raise RuntimeError(f'no dock position for {self.robot_id} in the site file')

        self._lock = threading.RLock()
        self._group = ReentrantCallbackGroup()
        self.token = DriveTokenCheck(self.robot_id)
        self.estop = EStopCheck(self.robot_id)
        self.dock = DockCheck()
        self.pose = None
        self._active_goal = None
        # 콜백 → 작업 스레드 플래그
        self._estop_latched = False      # EStop 활성을 받은 뒤 운영자 재개 전까지 True
        self._evacuate = False
        self._resume_command = False
        self._last_command_id = ''
        self._applied_command_ids = []   # 관제가 같은 명령을 다시 보내도 한 번만 적용한다
        self._command_status = F.COMMAND_NONE
        self._detect_request = None      # Future completed when the robot has stopped
        self._detect_resume = False
        self._accept_stop = False        # 순찰 경로 주행 중에만 감지 정지 요청을 받는다
        self._low_battery = False
        # 작업 스레드가 취소해야 할 진행 중 goal
        self._create_goal = None         # (Create 3 Dock/Undock goal handle, result future)
        self._detect_goal = None         # (DetectEvent goal handle, result future)
        self._dock_client = self._undock_client = None
        self._fb = Patrol.Feedback()

        self.create_subscription(DriveToken, 'drive_token', self._on_token, token_qos(),
                                 callback_group=self._group)
        self.create_subscription(EStop, ESTOP_TOPIC, self._on_estop, estop_qos(),
                                 callback_group=self._group)
        self.create_subscription(PatrolCommand, 'patrol_command', self._on_command, 10,
                                 callback_group=self._group)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10,
                                 callback_group=self._group)
        self.create_subscription(DockStatus, 'dock_status', self._on_dock_status,
                                 _dock_status_qos(), callback_group=self._group)
        self.create_subscription(BatteryState, 'battery_state', self._on_battery,
                                 qos_profile_sensor_data, callback_group=self._group)
        self.create_service(SetBool, 'patrol_stop', self._on_stop_request,
                            callback_group=self._group)
        self.create_subscription(Bool, 'patrol_resume', self._on_resume, 10,
                                 callback_group=self._group)
        self.detect_client = ActionClient(self, DetectEvent, 'detect_event',
                                          callback_group=self._group)
        self.server = ActionServer(
            self, Patrol, 'patrol_action',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self._group)
        self.get_logger().info(
            f'mission_supervisor ready: /{self.robot_id}/patrol_action')

    # ── 콜백 ────────────────────────────────────────────────────────────────
    def _on_token(self, msg):
        lease = msg.lease_duration.sec + msg.lease_duration.nanosec / 1e9
        with self._lock:
            self.token.observe(TokenFrame(msg.token, msg.holder_robot_id, lease, msg.sequence),
                               time.monotonic())

    def token_valid(self):
        with self._lock:
            return self.token.valid(time.monotonic())

    def _on_estop(self, msg):
        with self._lock:
            changed = self.estop.observe(
                EStopFrame(msg.target_robot_id, msg.active, msg.reason, msg.sequence),
                time.monotonic())
            active = self.estop.active()
            if active and self._active_goal is not None:
                if not self._estop_latched:
                    # EStop 이전에 온 명령은 무효: 해제 뒤에 새로 온 재개 명령만 인정한다.
                    self._resume_command = self._evacuate = False
                self._estop_latched = True
        if changed:
            self.get_logger().warning(
                f'EStop {"ACTIVE" if active else "released"} '
                f'(target={msg.target_robot_id} reason={msg.reason})')

    def _on_dock_status(self, msg):
        with self._lock:
            self.dock.observe_dock(msg.is_docked, time.monotonic())

    def _on_battery(self, msg):
        with self._lock:
            self.dock.observe_battery(msg.present, msg.power_supply_status, msg.percentage,
                                      time.monotonic())
            pct = self.dock.percentage
            if (self._active_goal is not None and not self._low_battery
                    and pct is not None and pct < LOW_BATTERY):
                self._low_battery = True
                self.get_logger().warning(
                    f'battery {pct * 100:.0f}% < {LOW_BATTERY * 100:.0f}%: return to dock')

    def docked(self):
        with self._lock:
            return self.dock.docked(time.monotonic())

    def is_docked(self):
        with self._lock:
            return self.dock.is_docked

    def _on_pose(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.pose = ps

    def _on_goal(self, goal):
        with self._lock:
            if self._active_goal is not None:
                self.get_logger().warning('Patrol Goal rejected: a patrol is already active')
                return GoalResponse.REJECT
            if goal.robot_id != self.robot_id:
                self.get_logger().warning(
                    f'Patrol Goal rejected: robot_id={goal.robot_id!r} != {self.robot_id}')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_command(self, msg):
        with self._lock:
            if msg.robot_id and msg.robot_id != self.robot_id:
                return
            if msg.command_id and msg.command_id in self._applied_command_ids:
                return
            self._last_command_id = msg.command_id
            active = self._active_goal is not None
            if active and self.estop.active():
                # EStop 중 명령은 받지 않는다. 해제 뒤 관제 재개 명령이 다시 온다.
                self._command_status = F.COMMAND_REJECTED
            elif active and msg.command == PatrolCommand.MOVE_TO_SAFE_ZONE:
                self._evacuate, self._resume_command = True, False
                self._command_status = F.COMMAND_ACCEPTED
                self._remember_command(msg.command_id)
            elif active and msg.command == PatrolCommand.RESUME_PATROL:
                self._resume_command = True
                self._evacuate = False
                self._command_status = F.COMMAND_ACCEPTED
                self._remember_command(msg.command_id)
            else:
                self._command_status = F.COMMAND_REJECTED
            self.get_logger().info(
                f'PatrolCommand {msg.command} id={msg.command_id} active={active} '
                f'status={self._command_status}')

    def _remember_command(self, command_id):
        if command_id:
            self._applied_command_ids.append(command_id)
            del self._applied_command_ids[:-32]

    async def _on_stop_request(self, request, response):
        """Stop for the detection node (설계도 01: 정지 요청 → 정지 응답 확인); data=false."""
        if request.data:
            response.success, response.message = False, 'use data=false to stop'
            return response
        with self._lock:
            if self._active_goal is None or not self._accept_stop:
                response.success, response.message = False, 'not driving the patrol route'
                return response
            if self._detect_request is None or self._detect_request.done():
                self._detect_request = Future(executor=self.executor)
                self._detect_resume = False
            future = self._detect_request
        response.success, response.message = await future
        return response

    def _reject_pending_stop(self, reason):
        with self._lock:
            if self._detect_request is not None and not self._detect_request.done():
                self._detect_request.set_result((False, reason))
            self._detect_request = None

    def _on_resume(self, msg):
        with self._lock:
            if msg.data:
                self._detect_resume = True

    # ── Feedback ────────────────────────────────────────────────────────────
    def _publish(self, goal_handle, state=None, **fields):
        with self._lock:
            if state is not None:
                self._fb.task_state = state
            for key, value in fields.items():
                setattr(self._fb, key, value)
            self._fb.last_command_id = self._last_command_id
            self._fb.command_status = self._command_status
            self._fb.token_valid = self.token.valid(time.monotonic())
            self._fb.accepted_token_sequence = self.token.accepted_sequence
            if self.pose is not None:
                self._fb.current_pose = self.pose
            goal_handle.publish_feedback(self._fb)

    # ── Action 실행 (작업 스레드) ───────────────────────────────────────────
    def _execute(self, goal_handle):
        with self._lock:
            self._active_goal = goal_handle
            self._evacuate = self._resume_command = False
            self._estop_latched = self.estop.active()
            self._command_status = F.COMMAND_NONE
            self._low_battery = False
            self._fb = Patrol.Feedback()
        nav = evac = None
        result = R()
        try:
            nav = TurtleBot4Navigator(namespace=self.robot_id)
            self._dock_client = ActionClient(nav, Dock, 'dock')
            self._undock_client = ActionClient(nav, Undock, 'undock')
            zones = list(self.site.safe_zones.values())
            evac = Evacuation(nav, _EvacSettings(self.robot_id, zones))
            result.outcome, result.reason_code, result.reason = self._run(
                goal_handle, nav, evac)
            if result.outcome == R.SUCCEEDED:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except PatrolAbort as end:
            result.outcome, result.reason_code, result.reason = (
                end.outcome, end.reason_code, end.reason)
            self._safe_stop(nav, evac)
            if end.outcome == R.CANCELED and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        except Exception as exc:  # noqa: B902 - any failure must end the goal safely
            self.get_logger().error(f'Patrol internal error: {exc}')
            result.outcome, result.reason_code = R.FAILED, R.REASON_INTERNAL_ERROR
            result.reason = str(exc)
            self._safe_stop(nav, evac)
            goal_handle.abort()
        finally:
            with self._lock:
                self._accept_stop = False
                self._active_goal = None
                self._estop_latched = False
            self._reject_pending_stop('Patrol ended')
            if nav is not None:
                nav.destroy_node()
        text = (f'Patrol Result outcome={result.outcome} reason_code={result.reason_code} '
                f'reason={result.reason}')
        if result.outcome == R.SUCCEEDED:
            self.get_logger().info(text)
        else:
            self.get_logger().error(text)
        return result

    def _safe_stop(self, nav, evac):
        """End of goal: cancel whatever is still running, then confirm the stop."""
        if nav is not None:
            self._cancel_detect(None, nav)
            self._cancel_create_goal(nav)
        if evac is None:
            return
        try:
            evac.stop()
        except Exception as exc:  # noqa: B902
            if nav is not None:
                nav.error(f'STOP NOT CONFIRMED: {exc}')

    def _run(self, gh, nav, evac):
        """Run one patrol; return (outcome, reason_code, reason) once docked."""
        # 1. 주행 권한 대기 (도크 위). EStop이 걸리면 해제 · 재개 명령까지 기다린다.
        self._publish(gh, F.WAITING_FOR_TOKEN)
        while True:
            try:
                self._wait(gh, nav, self.token_valid, state=F.WAITING_FOR_TOKEN,
                           watch=('estop',))
                break
            except _SafetyStop:
                # 도크 위라 움직이는 것은 없다. 토큰은 WAITING_FOR_TOKEN 뒤에 발급되므로
                # 해제 조건에 토큰을 넣지 않는다.
                self._safety_hold(gh, nav, evac, require_token=False)
                self._publish(gh, F.WAITING_FOR_TOKEN)

        # 2. 도킹 해제 · 초기 위치
        self._publish(gh, F.UNDOCKING)
        self._wait(gh, nav, lambda: self.is_docked() is not None, DOCK_STATUS_WAIT,
                   F.UNDOCKING)
        if self.is_docked() is None:
            raise PatrolAbort(R.FAILED, R.REASON_DOCKING_FAILED, 'dock_status not received')
        if not self.is_docked():
            # 도크 앞 대기 중이던 로봇: 위치 추정 전이므로 이동 없이 도킹만 다시 시도한다.
            nav.info('Not docked at start: docking before undocking')
            if not self._dock_attempts(gh, nav, evac, approach=False):
                raise PatrolAbort(R.FAILED, R.REASON_DOCKING_FAILED, 'initial docking failed')
        self._publish(gh, F.UNDOCKING)
        if not self._guarded(gh, nav, evac, lambda: self._undock_once(gh, nav)):
            raise PatrolAbort(R.FAILED, R.REASON_DOCKING_FAILED, 'undocking failed')
        xy, direction = self.site.initial_poses[self.robot_id]
        nav.setInitialPose(nav.getPoseStamped(list(xy), D[direction]))
        nav.waitUntilNav2Active()
        self._publish(gh, F.INITIAL_POSE_READY)

        # 3. 순찰
        skipped = self._patrol(gh, nav, evac)
        note = f'; skipped: {",".join(skipped)}' if skipped else ''

        # 4. 귀환 · 도킹
        self._return_and_dock(gh, nav, evac, note)
        with self._lock:
            low_battery = self._low_battery
        if low_battery:
            return R.FAILED, R.REASON_LOW_BATTERY, f'returned on low battery; docked{note}'
        return R.SUCCEEDED, 0, f'docked{note}'

    # ── 순찰 경로 ───────────────────────────────────────────────────────────
    def _patrol(self, gh, nav, evac):
        route = build_route(nav, self.site.route)
        steps = [s for _, s in route]
        index = failures = consecutive_skips = 0
        skipped = []
        self._publish(gh, F.PATROLLING)
        try:
            while index < len(route):
                with self._lock:
                    low_battery = self._low_battery
                if low_battery:
                    nav.warn('Low battery: ending patrol and returning to dock')
                    break
                waypoint_id, step = route[index]
                with self._lock:
                    self._accept_stop = True
                outcome = self._run_step(gh, nav, step, route_step=True)
                with self._lock:
                    self._accept_stop = False
                if outcome == 'done':
                    if waypoint_id is not None:
                        consecutive_skips = 0
                        self._publish(gh, F.WAYPOINT_REACHED, current_waypoint_id=waypoint_id)
                        self._publish(gh, F.PATROLLING)
                    failures = 0
                    index += 1
                    continue
                if outcome == 'interrupted':
                    self._handle_interrupt(gh, nav, evac, steps, index)
                    self._publish(gh, F.PATROLLING)
                    continue
                # outcome == 'failed'
                failures += 1
                label = waypoint_id or f'{self._waypoint_of(route, index)} spin'
                if failures <= STEP_RETRIES:
                    nav.warn(f'{label} failed; retrying ({failures}/{STEP_RETRIES})')
                    continue
                failures = 0
                skipped.append(label)
                self.get_logger().error(f'{label} failed again: skipped')
                if waypoint_id is not None:
                    consecutive_skips += 1
                    if index + 1 < len(route) and route[index + 1][1] == SPIN:
                        index += 1     # 도달하지 못한 경유지의 360° 회전도 건너뛴다
                    if consecutive_skips >= MAX_CONSECUTIVE_SKIPS:
                        self.get_logger().error(
                            f'{consecutive_skips} waypoints in a row failed: returning to dock')
                        break
                index += 1
        finally:
            with self._lock:
                self._accept_stop = False
            self._reject_pending_stop('patrol route finished')
        return skipped

    @staticmethod
    def _waypoint_of(route, index):
        for i in range(index, -1, -1):
            if route[i][0] is not None:
                return route[i][0]
        return 'start'

    def _interrupt_pending(self, route_step):
        with self._lock:
            now = time.monotonic()
            detect = (route_step and self._detect_request is not None
                      and not self._detect_request.done())
            return (self._estop_latched or self._evacuate or detect
                    or not self.token.valid(now) or (route_step and self._low_battery))

    def _check_cancel(self, gh):
        if gh.is_cancel_requested:
            raise PatrolAbort(R.CANCELED, R.REASON_CONTROL_CANCELED, 'canceled by control')

    def _run_step(self, gh, nav, step, route_step):
        """Run one move/spin: 'done', 'interrupted' (caller handles) or 'failed'."""
        self._check_cancel(gh)
        if self._interrupt_pending(route_step):
            return 'interrupted'
        if step == SPIN:
            accepted = nav.spin(spin_dist=2 * math.pi, time_allowance=SPIN_TIME_ALLOWANCE)
        else:
            step.header.stamp = nav.get_clock().now().to_msg()
            accepted = nav.goToPose(step)
        if not accepted:
            return 'failed'
        last = 0.0
        while not nav.isTaskComplete():
            self._check_cancel(gh)
            if self._interrupt_pending(route_step):
                return 'interrupted'
            if time.monotonic() - last >= FEEDBACK_PERIOD:
                self._publish(gh)
                last = time.monotonic()
            rclpy.spin_once(nav, timeout_sec=0.05)
        if nav.getResult() == TaskResult.SUCCEEDED:
            return 'done'
        if self._interrupt_pending(route_step):
            return 'interrupted'
        return 'failed'

    def _drive_to(self, gh, nav, evac, pose, state, retries=STEP_RETRIES):
        """Drive to one pose outside the patrol route, handling interrupts; True if reached."""
        failures = 0
        while True:
            self._publish(gh, state)
            outcome = self._run_step(gh, nav, pose, route_step=False)
            if outcome == 'done':
                return True
            if outcome == 'interrupted':
                self._handle_interrupt(gh, nav, evac, None, None)
                continue
            failures += 1
            if failures > retries:
                return False
            nav.warn(f'navigation failed; retrying ({failures}/{retries})')

    def _drive_spin(self, gh, nav, evac):
        while True:
            outcome = self._run_step(gh, nav, SPIN, route_step=False)
            if outcome != 'interrupted':
                return outcome == 'done'
            self._handle_interrupt(gh, nav, evac, None, None)

    # ── 귀환 · 도킹 ─────────────────────────────────────────────────────────
    def _home_pose(self, nav):
        xy, direction = self.site.docks[self.robot_id]
        return nav.getPoseStamped(list(xy), D[direction])

    def _return_and_dock(self, gh, nav, evac, note):
        self._publish(gh, F.DOCKING)
        if not self._drive_to(gh, nav, evac, self._home_pose(nav), F.DOCKING):
            self.get_logger().error(f'ALERT {self.robot_id}: dock front unreachable')
            if self._park_in_safe_zone(gh, nav, evac):
                raise PatrolAbort(R.FAILED, R.REASON_DOCKING_FAILED,
                                  f'dock front unreachable; parked in nearest safe zone{note}')
            raise PatrolAbort(R.FAILED, R.REASON_DOCKING_FAILED,
                              f'dock front and nearest safe zone unreachable; '
                              f'stopped in place{note}')
        if self._dock_attempts(gh, nav, evac, approach=True):
            return
        # 자기 도크 앞(다른 로봇 도크와 겹치지 않는 위치)에 정차
        if self.is_docked():
            self._guarded(gh, nav, evac, lambda: self._undock_once(gh, nav))
        self._drive_to(gh, nav, evac, self._home_pose(nav), F.DOCKING)
        raise PatrolAbort(R.FAILED, R.REASON_DOCKING_FAILED,
                          f'docking failed {DOCK_ATTEMPTS} times; waiting at dock front{note}')

    def _park_in_safe_zone(self, gh, nav, evac):
        try:
            x, y = self._current_xy(evac)
        except PatrolAbort as exc:
            nav.error(f'position unknown, cannot pick a safe zone: {exc}')
            return False
        target = min(evac.safe, key=lambda s: (s.pose.position.x - x) ** 2
                     + (s.pose.position.y - y) ** 2)
        return self._drive_to(gh, nav, evac, target, F.DOCKING)

    def _dock_attempts(self, gh, nav, evac, approach):
        """Try docking up to DOCK_ATTEMPTS times; True once docked (CR 9/9 rule)."""
        for attempt in range(1, DOCK_ATTEMPTS + 1):
            self._publish(gh, F.DOCKING)
            if approach and attempt >= 2:
                if self.is_docked():
                    # 접점은 붙었지만 충전이 확인되지 않음: 도크에서 빠진 뒤 다시 접근한다.
                    self._guarded(gh, nav, evac, lambda: self._undock_once(gh, nav))
                if attempt == 3:
                    # 위치를 다시 잡기 위해 제자리 360° 회전(AMCL 갱신) 후 다시 접근
                    self._drive_spin(gh, nav, evac)
                self._drive_to(gh, nav, evac, self._home_pose(nav), F.DOCKING)
            if self._guarded(gh, nav, evac, lambda: self._dock_once(gh, nav)):
                nav.info(f'Docked (attempt {attempt}/{DOCK_ATTEMPTS})')
                return True
            self.get_logger().error(
                f'ALERT {self.robot_id}: docking attempt {attempt}/{DOCK_ATTEMPTS} failed')
        return False

    def _dock_once(self, gh, nav):
        """One Create 3 Dock attempt; True when the CR 9/9 docking rule holds."""
        if self.docked():
            return True
        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            nav.error('Dock action server unavailable')
            return False
        result = self._send_create_goal(gh, nav, self._dock_client, Dock.Goal())
        if result is None:
            return False
        result_done_at = []

        try:
            def finished():
                if self.docked():
                    return True
                if not result.done():
                    return False
                # Dock action 결과가 dock_status보다 먼저 온다(dock_status는 약 0.5초 주기).
                # 결과 뒤 DOCK_STATUS_SETTLE 동안 접속이 없을 때만 이 시도를 실패로 본다.
                if not result_done_at:
                    result_done_at.append(time.monotonic())
                return (self.is_docked() is False
                        and time.monotonic() - result_done_at[0] > DOCK_STATUS_SETTLE)

            self._wait(gh, nav, finished, DOCK_ATTEMPT_TIMEOUT, F.DOCKING,
                       watch=('estop', 'token'))
        finally:
            self._cancel_create_goal(nav)
        return self.docked()

    def _undock_once(self, gh, nav):
        """One Create 3 Undock; True once dock_status reports undocked."""
        if self.is_docked() is False:
            return True
        if not self._undock_client.wait_for_server(timeout_sec=5.0):
            nav.error('Undock action server unavailable')
            return False
        result = self._send_create_goal(gh, nav, self._undock_client, Undock.Goal())
        if result is None:
            return False
        try:
            self._wait(gh, nav, result.done, UNDOCK_TIMEOUT, F.UNDOCKING,
                       watch=('estop', 'token'))
        finally:
            self._cancel_create_goal(nav)
        self._wait(gh, nav, lambda: self.is_docked() is False, DOCK_STATUS_WAIT, F.UNDOCKING)
        return self.is_docked() is False

    def _send_create_goal(self, gh, nav, client, goal):
        send = client.send_goal_async(goal)
        self._wait(gh, nav, send.done, 5.0, None)
        handle = send.result() if send.done() else None
        if handle is None or not handle.accepted:
            nav.error('Create 3 goal not accepted')
            return None
        result = handle.get_result_async()
        with self._lock:
            self._create_goal = (handle, result)
        return result

    def _cancel_create_goal(self, nav):
        with self._lock:
            goal, self._create_goal = self._create_goal, None
        if goal is None:
            return
        handle, result = goal
        if result.done():
            return
        try:
            future = handle.cancel_goal_async()
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(nav, timeout_sec=0.05)
            # Create 3는 이전 동작이 끝나기 전에 온 dock/undock Goal을 거절한다
            # ("A docking behavior is already running"). 다음 시도 전에 종료를 기다린다.
            deadline = time.monotonic() + CREATE_CANCEL_WAIT
            while not result.done() and time.monotonic() < deadline:
                rclpy.spin_once(nav, timeout_sec=0.05)
            if not result.done():
                nav.warn('Create 3 goal did not end after cancel')
        except Exception as exc:  # noqa: B902
            nav.error(f'Create 3 goal cancel failed: {exc}')

    # ── 대기 · 중단 처리 ────────────────────────────────────────────────────
    def _wait(self, gh, nav, done, timeout=None, state=None, on_timeout=None, watch=()):
        """
        Spin the navigator until done() or timeout, publishing feedback.

        Cancel always ends the goal. `watch` names the interrupts that end this
        wait early: 'estop' raises _SafetyStop, 'token' raises _TokenLost,
        'evacuate' raises _Evacuate.
        """
        start = last = time.monotonic()
        while not done():
            self._check_cancel(gh)
            with self._lock:
                latched = self._estop_latched
                token_ok = self.token.valid(time.monotonic())
                evacuate = self._evacuate
            if 'estop' in watch and latched:
                raise _SafetyStop()
            if 'token' in watch and not token_ok:
                raise _TokenLost()
            if 'evacuate' in watch and evacuate:
                raise _Evacuate()
            now = time.monotonic()
            if timeout is not None and now - start > timeout:
                if on_timeout is not None:
                    on_timeout()
                return False
            if now - last >= FEEDBACK_PERIOD:
                self._publish(gh, state)
                last = now
            rclpy.spin_once(nav, timeout_sec=0.05)
        return True

    def _guarded(self, gh, nav, evac, op):
        """Run a Create 3 operation; on EStop or token loss stop, wait, and run it again."""
        while True:
            try:
                return op()
            except _SafetyStop:
                self._safety_hold(gh, nav, evac)
            except _TokenLost:
                self._cancel_create_goal(nav)
                self._blocked(gh, nav, evac)

    def _stop_motion(self, nav, evac, gh=None):
        """
        Cancel the Nav2 task and confirm a real stop (wheels are already gated).

        When odometry cannot confirm the stop (late or missing odometry), the stop
        check runs again. After STOP_CONFIRM_ATTEMPTS failures the mission does not
        end: it reports BLOCKED and keeps checking until the stop is confirmed or
        control cancels the goal. Nav2 is already canceled, so no drive command is
        sent while it waits.
        """
        if evac is None:
            return
        attempt = 0
        while True:
            try:
                evac.stop()
                return
            except RuntimeError as exc:
                with self._lock:
                    gated = self._estop_latched or not self.token.valid(time.monotonic())
                if gated:
                    nav.error(f'stop not confirmed while local safety blocks the wheels: {exc}')
                    return
                attempt += 1
                nav.error(f'stop not confirmed (check {attempt}): {exc}')
                if gh is None:
                    if attempt >= STOP_CONFIRM_ATTEMPTS:
                        raise
                    continue
                self._check_cancel(gh)
                if attempt == STOP_CONFIRM_ATTEMPTS:
                    self.get_logger().error(
                        f'ALERT {self.robot_id}: stop not confirmed {attempt} times; '
                        'BLOCKED until odometry confirms the stop')
                    self._publish(gh, F.BLOCKED)

    def _handle_interrupt(self, gh, nav, evac, steps, index):
        """Handle whatever interrupted a move: EStop > token > safe zone > detection."""
        with self._lock:
            latched = self._estop_latched
        self._stop_motion(nav, evac, gh)
        with self._lock:
            latched = latched or self._estop_latched
            token_ok = self.token.valid(time.monotonic())
            evacuate = self._evacuate
            detect = self._detect_request is not None and not self._detect_request.done()
        if latched:
            self._safety_hold(gh, nav, evac)
            self._publish(gh, F.RESUMING)
        elif not token_ok:
            self._blocked(gh, nav, evac)
            self._publish(gh, F.RESUMING)
        elif evacuate:
            self._safe_zone(gh, nav, evac, steps, index)
        elif detect:
            self._detection(gh, nav, evac)

    def _safety_hold(self, gh, nav, evac, require_token=True):
        """
        Stop now for an EStop (CR 9/8); continue after release, token and resume command.

        Returns 'resume' (RESUME_PATROL) or 'evacuate' (MOVE_TO_SAFE_ZONE; the
        _evacuate flag stays set so the caller's normal safe-zone handling runs).
        """
        self._cancel_detect(gh, nav)
        self._cancel_create_goal(nav)
        self._stop_motion(nav, evac, gh)
        with self._lock:
            reason = self.estop.reason()
        # EStop 이전 명령은 래치할 때(_on_estop) 이미 지웠고, EStop 중 명령은
        # _on_command가 거절한다. 여기서 지우면 해제 직후 온 재개 명령을 잃는다.
        self.get_logger().error(f'EStop: patrol BLOCKED (reason={reason})')
        self._publish(gh, F.BLOCKED)

        def released():
            with self._lock:
                ok = (not self.estop.active()
                      and (not require_token or self.token.valid(time.monotonic()))
                      and (self._resume_command or self._evacuate))
                if ok:
                    self._estop_latched = False
                return ok
        self._wait(gh, nav, released, state=F.BLOCKED)
        with self._lock:
            evacuate = self._evacuate
            self._resume_command = False
            self._command_status = F.COMMAND_EXECUTING
        self.get_logger().warning(
            'EStop released and resume command received: '
            + ('moving to safe zone' if evacuate else 'resuming'))
        return 'evacuate' if evacuate else 'resume'

    def _blocked(self, gh, nav, evac=None):
        """Wait for the DriveToken; returns the EStop hold result if one happened, else None."""
        self.get_logger().warning('DriveToken invalid: patrol BLOCKED')
        self._publish(gh, F.BLOCKED)

        def expired():
            raise PatrolAbort(R.FAILED, R.REASON_TOKEN_EXPIRED,
                              f'DriveToken not restored within {TOKEN_TIMEOUT:.0f}s')
        while True:
            try:
                self._wait(gh, nav, self.token_valid, TOKEN_TIMEOUT, F.BLOCKED, expired,
                           watch=('estop',))
                return None
            except _SafetyStop:
                # 토큰 없는 사이 EStop: EStop 규칙이 우선한다(해제 조건에 토큰 포함).
                return self._safety_hold(gh, nav, evac)

    def _safe_zone(self, gh, nav, evac, steps, index):
        with self._lock:
            self._evacuate = False
            self._command_status = F.COMMAND_EXECUTING
        # 대피가 우선: 남아 있던 감지 정지 요청은 거절한다(안전구역에서 엉뚱하게 처리하지 않게).
        self._reject_pending_stop('evacuating to safe zone')
        self._publish(gh, F.MOVING_TO_SAFE_ZONE)
        x, y = self._current_xy(evac)
        target = min(evac.safe, key=lambda s: (s.pose.position.x - x) ** 2
                     + (s.pose.position.y - y) ** 2)
        self.get_logger().info(
            f'MOVE_TO_SAFE_ZONE from ({x:.2f}, {y:.2f}) to '
            f'({target.pose.position.x:.2f}, {target.pose.position.y:.2f})')

        # 이동: EStop · 토큰 상실 · 중복 명령을 처리하며 안전구역까지 간다.
        # EStop 해제 뒤 관제가 RESUME_PATROL을 보냈다면 차량이 이미 나간 것이다.
        car_left = False
        while True:
            outcome = self._run_step(gh, nav, target, route_step=False)
            if outcome == 'done':
                break
            if outcome == 'failed':
                raise PatrolAbort(R.FAILED, R.REASON_NAVIGATION_FAILED,
                                  'safe zone navigation failed')
            with self._lock:
                latched = self._estop_latched
            self._stop_motion(nav, evac, gh)
            if latched or self._estop_is_latched():
                car_left = self._safety_hold(gh, nav, evac) == 'resume'
            elif not self.token_valid():
                car_left = self._blocked(gh, nav, evac) == 'resume'
            with self._lock:
                self._evacuate = False
                car_left = car_left or self._resume_command
            if car_left:
                break
            self._publish(gh, F.MOVING_TO_SAFE_ZONE)
        self._stop_motion(nav, evac, gh)

        # 대기: RESUME_PATROL(차량 출차)까지. 대기 중 EStop도 같은 규칙을 따른다.
        if not car_left:
            self.get_logger().info('In safe zone: waiting for RESUME_PATROL')
            while True:
                try:
                    self._wait(gh, nav, lambda: self._resume_command,
                               state=F.MOVING_TO_SAFE_ZONE, watch=('estop',))
                    break
                except _SafetyStop:
                    if self._safety_hold(gh, nav, evac) == 'resume':
                        break
                    with self._lock:
                        self._evacuate = False
                    self._publish(gh, F.MOVING_TO_SAFE_ZONE)
        with self._lock:
            self._resume_command = False
            self._command_status = F.COMMAND_EXECUTING
        self._publish(gh, F.RESUMING)
        if steps is not None and steps[index] == SPIN and index > 0:
            # 회전 중이었다면 회전하던 경유지로 돌아가서 360° 회전을 다시 한다.
            self._drive_to(gh, nav, evac, steps[index - 1], F.RESUMING)
        with self._lock:
            self._command_status = F.COMMAND_NONE

    def _current_xy(self, evac):
        """Map position from TF, or the last amcl_pose when TF is stale."""
        try:
            return evac.position()
        except RuntimeError as exc:
            pose = self.pose
            if pose is None:
                raise PatrolAbort(R.FAILED, R.REASON_LOCALIZATION_LOST,
                                  f'no map position for the safe zone: {exc}')
            self.get_logger().warning(f'TF position unavailable ({exc}); using amcl_pose')
            return pose.pose.position.x, pose.pose.position.y

    def _estop_is_latched(self):
        with self._lock:
            return self._estop_latched

    def _detection(self, gh, nav, evac):
        """Run DetectEvent 정렬 · 검증 · 저장 결과 대기. EStop · 토큰 상실 · 대피 명령이면 취소한다."""
        detection_id = str(uuid.uuid4())
        with self._lock:
            request = self._detect_request
            self._detect_resume = False
        request.set_result((True, f'Stopped; detection_id={detection_id}'))
        self._publish(gh, F.DETECTION_PROCESSING)
        if not self.detect_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('DetectEvent server unavailable: resuming patrol')
            return
        goal = DetectEvent.Goal()
        goal.detection_id = detection_id
        goal.min_confidence = DETECT_MIN_CONFIDENCE
        # event_type 0: 감지 노드가 정지를 요청한 후보의 종류로 정렬 · 검증한다.
        watch = ('estop', 'token', 'evacuate')
        send = self.detect_client.send_goal_async(goal)
        try:
            self._wait(gh, nav, send.done, 5.0, F.DETECTION_PROCESSING, watch=watch)
            handle = send.result() if send.done() else None
            if handle is None or not handle.accepted:
                self.get_logger().error('DetectEvent Goal not accepted: resuming patrol')
                return
            result_future = handle.get_result_async()
            with self._lock:
                self._detect_goal = (handle, result_future)
            if not self._wait(gh, nav, result_future.done, DETECT_TIMEOUT,
                              F.DETECTION_PROCESSING, watch=watch):
                self.get_logger().warning('DetectEvent timed out: cancelling before resuming')
                self._cancel_detect(gh, nav)
                return
            with self._lock:
                self._detect_goal = None
            wrapped = result_future.result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.confirmed:
                self.get_logger().info(f'Detection {detection_id} not confirmed: resuming')
                return
            res = wrapped.result
            self.get_logger().info(
                f'DETECTION_CONFIRMED id={detection_id} type={res.event_type} '
                f'avg_conf={res.average_confidence:.2f}')
            self._publish(gh, F.DETECTION_CONFIRMED, event_id=detection_id,
                          event_type=res.event_type)
            # 감지 노드가 ReportDetection 결과를 받은 뒤 patrol_resume(true)을 보낸다.
            if not self._wait(gh, nav, lambda: self._detect_resume, STORE_TIMEOUT,
                              F.DETECTION_CONFIRMED, watch=watch):
                self.get_logger().error(
                    f'No patrol_resume within {STORE_TIMEOUT:.0f}s after confirmation; '
                    'resuming patrol (evidence may not be stored)')
        except _SafetyStop:
            self._safety_hold(gh, nav, evac)      # 감지 취소 포함
            self._publish(gh, F.RESUMING)
        except _TokenLost:
            self._cancel_detect(gh, nav)
            self._blocked(gh, nav, evac)
            self._publish(gh, F.RESUMING)
        except _Evacuate:
            self._cancel_detect(gh, nav)
            self._safe_zone(gh, nav, evac, None, None)
        finally:
            # 순찰 취소(PatrolAbort) 등으로 빠져나가도 감지 노드의 정렬 · 검증을 멈춘다.
            self._cancel_detect(gh, nav)
            self._cancel_when_accepted(send)

    @staticmethod
    def _cancel_when_accepted(send):
        """Goal sent but given up before tracking: cancel it as soon as it is accepted."""
        def cancel(future):
            try:
                handle = future.result()
            except Exception:  # noqa: B902 - nothing to cancel
                return
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
        if not send.done():
            send.add_done_callback(cancel)

    def _cancel_detect(self, gh, nav):
        """Cancel a running DetectEvent and wait until the node has stopped its yaw output."""
        with self._lock:
            pending, self._detect_goal = self._detect_goal, None
        if pending is None:
            return
        handle, result_future = pending
        if result_future.done():
            return
        handle.cancel_goal_async()
        deadline = time.monotonic() + DETECT_CANCEL_TIMEOUT
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.05)
        if not result_future.done():
            self.get_logger().error('DetectEvent did not finish after cancel')


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
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
