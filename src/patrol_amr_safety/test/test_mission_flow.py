"""
mission_supervisor flow with a simulated navigator, Create 3 and clock.

Covers the interrupt rules (EStop latch, token loss, safe zone, detection),
waypoint retry/skip, low-battery return and the docking retry sequence.
"""

import importlib
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

CHARGING, DISCHARGING = 1, 2


class Clock:
    def __init__(self):
        self.t = 1000.0
        self.events = []          # (time, fn)

    def at(self, delay, fn):
        self.events.append((self.t + delay, fn))

    def advance(self, dt):
        self.t += dt
        due = sorted([e for e in self.events if e[0] <= self.t], key=lambda e: e[0])
        self.events = [e for e in self.events if e[0] > self.t]
        for _, fn in due:
            fn()


class Pose(SimpleNamespace):
    pass


def pose(x, y, label):
    return Pose(label=label, header=SimpleNamespace(stamp=None),
                pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))


class FakeNav:
    """Navigator: each task takes `duration` s and ends with plan(label) result."""

    def __init__(self, clock, plan=None):
        self.clock = clock
        self.plan = plan or (lambda label: (1.0, True))
        self.task = None
        self.log = []                 # labels of tasks started
        self.result = None
        self.position = (0.0, 0.0)

    def getPoseStamped(self, xy, direction):
        return pose(xy[0], xy[1], f'{xy[0]:.3f},{xy[1]:.3f}')

    def goToPose(self, p):
        return self._start(p.label, (p.pose.position.x, p.pose.position.y))

    def spin(self, spin_dist, time_allowance):
        return self._start('spin', None)

    def _start(self, label, target):
        duration, ok = self.plan(label)
        self.log.append(label)
        self.task = (self.clock.t + duration, ok, target)
        return True

    def isTaskComplete(self):
        if self.task is None:
            return True
        end, ok, target = self.task
        if self.clock.t >= end:
            self.result = 'ok' if ok else 'fail'
            if ok and target is not None:
                self.position = target
            self.task = None
            return True
        return False

    def cancel(self):
        if self.task is not None:
            self.task = None
            self.result = 'canceled'

    def getResult(self):
        return self.result

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))

    def setInitialPose(self, p):
        self.position = (p.pose.position.x, p.pose.position.y)

    def waitUntilNav2Active(self):
        pass

    def info(self, *_):
        pass

    warn = error = info


class FakeEvac:
    def __init__(self, nav, safe):
        self.nav, self.safe, self.stops = nav, safe, 0
        self.unconfirmed_stops = 0   # 이 횟수만큼 odometry가 정지를 확인하지 못함

    def stop(self):
        self.nav.cancel()
        self.stops += 1
        if self.unconfirmed_stops > 0:
            self.unconfirmed_stops -= 1
            self.nav.clock.advance(10.0)          # 실제로는 stop-timeout만큼 기다린 뒤 실패
            raise RuntimeError('Fresh odometry did not confirm a stop')

    def position(self):
        return self.nav.position


class Done:
    """Already-completed future."""

    def __init__(self, value=None):
        self.value = value
        self.callbacks = []

    def done(self):
        return True

    def result(self):
        return self.value

    def add_done_callback(self, fn):
        fn(self)


class Later:
    """Future completed by the simulated clock."""

    def __init__(self):
        self.value, self._done = None, False

    def done(self):
        return self._done

    def result(self):
        return self.value

    def complete(self, value=None):
        self.value, self._done = value, True

    def add_done_callback(self, fn):
        pass


class Robot:
    """Create 3 dock / undock behaviour driven by a script of dock attempt outcomes."""

    def __init__(self, clock, node, dock_outcomes, status_lag=0.0, cancel_lag=0.0):
        self.clock, self.node = clock, node
        self.status_lag = status_lag      # 실제 Create 3처럼 dock_status가 결과보다 늦게 바뀜
        self.cancel_lag = cancel_lag      # 취소 요청 뒤 동작이 실제로 끝나기까지 걸리는 시간
        # 'ok' | 'contact_no_charge' | 'miss' | 'stuck'(스스로 끝나지 않음)
        self.dock_outcomes = list(dock_outcomes)
        self.dock_goals = self.undock_goals = self.canceled = self.rejected = 0
        self.busy = False                 # Create 3는 동작 중에 새 dock/undock Goal을 거절한다
        self.docked = True
        self.publish_dock(True)

    def publish_dock(self, docked):
        self.docked = docked
        self.node._on_dock_status(SimpleNamespace(is_docked=docked))

    def battery(self, status, pct=0.6):
        self.node._on_battery(SimpleNamespace(present=True, power_supply_status=status,
                                              percentage=pct))

    def client(self, kind):
        robot = self

        class Client:
            def wait_for_server(self, timeout_sec=None):
                return True

            def send_goal_async(self, goal):
                if robot.busy:
                    robot.rejected += 1
                    return Done(SimpleNamespace(accepted=False))
                robot.busy = True
                result = Later()
                state = {'canceled': False}

                def end():
                    robot.busy = False
                    result.complete()

                def cancel():
                    robot.canceled += 1
                    if not result.done() and not state['canceled']:
                        state['canceled'] = True      # Create 3가 동작을 멈춘다
                        robot.clock.at(robot.cancel_lag, end)
                    return Done()
                handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                                         cancel_goal_async=cancel)
                if kind == 'undock':
                    robot.undock_goals += 1

                    def finish():
                        if state['canceled']:
                            return
                        robot.publish_dock(False)
                        end()
                    robot.clock.at(2.0, finish)
                else:
                    robot.dock_goals += 1
                    outcome = robot.dock_outcomes.pop(0) if robot.dock_outcomes else 'ok'

                    def finish():
                        if state['canceled'] or outcome == 'stuck':
                            return
                        if outcome in ('ok', 'contact_no_charge'):
                            if robot.status_lag:
                                robot.clock.at(robot.status_lag,
                                               lambda: robot.publish_dock(True))
                            else:
                                robot.publish_dock(True)
                        end()
                        robot.clock.at(1.0, lambda: robot.battery(
                            CHARGING if outcome == 'ok' else DISCHARGING))
                    robot.clock.at(3.0, finish)
                return Done(handle)
        return Client()


class GoalHandle:
    def __init__(self):
        self.states = []
        self.is_cancel_requested = False

    def publish_feedback(self, fb):
        self.states.append(int(fb.task_state))


@pytest.fixture
def world(monkeypatch):
    module = importlib.import_module('patrol_amr_safety.mission_supervisor')
    clock = Clock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.t)
    monkeypatch.setattr(module, 'TaskResult', SimpleNamespace(SUCCEEDED='ok'))
    monkeypatch.setattr(module.rclpy, 'spin_once',
                        lambda _node, timeout_sec=0.05: clock.advance(timeout_sec))
    monkeypatch.setattr(module, 'build_route', lambda nav, route: [
        ('WP1', pose(1, 0, 'WP1')), (None, module.SPIN),
        ('WP2', pose(2, 0, 'WP2')), (None, module.SPIN),
        ('WP3', pose(3, 0, 'WP3')), (None, module.SPIN),
    ])
    monkeypatch.setattr(module.Node, '__init__', lambda *_a, **_k: None)
    for name in ('create_subscription', 'create_service', 'create_publisher'):
        monkeypatch.setattr(module.MissionSupervisor, name, lambda *_a, **_k: None,
                            raising=False)
    monkeypatch.setattr(module.MissionSupervisor, 'get_namespace', lambda _s: '/robot1')
    monkeypatch.setattr(module.MissionSupervisor, 'declare_parameter',
                        lambda *_a, **_k: SimpleNamespace(value=''), raising=False)
    logs = []
    logger = SimpleNamespace(info=logs.append, warning=logs.append, error=logs.append)
    monkeypatch.setattr(module.MissionSupervisor, 'get_logger', lambda _s: logger)
    monkeypatch.setattr(module, 'ActionClient', lambda *_a, **_k: SimpleNamespace())
    monkeypatch.setattr(module, 'ActionServer', lambda *_a, **_k: SimpleNamespace())

    def build(plan=None, dock_outcomes=(), status_lag=0.0, cancel_lag=0.0):
        node = module.MissionSupervisor()
        token = {'valid': True}
        node.token = SimpleNamespace(valid=lambda now: token['valid'], accepted_sequence=1,
                                     sequence=1)
        nav = FakeNav(clock, plan)
        evac = FakeEvac(nav, [pose(1, 1, 'SAFE_A'), pose(9, 9, 'SAFE_B')])
        robot = Robot(clock, node, dock_outcomes, status_lag, cancel_lag)
        node._dock_client = robot.client('dock')
        node._undock_client = robot.client('undock')
        gh = GoalHandle()
        node._active_goal = gh
        node._on_estop(SimpleNamespace(target_robot_id='all', active=False, reason=0,
                                       sequence=1))
        return SimpleNamespace(m=module, node=node, nav=nav, evac=evac, robot=robot, gh=gh,
                               clock=clock, token=token, logs=logs)
    return build


def states(w):
    return w.gh.states


def estop(w, active, seq):
    w.node._on_estop(SimpleNamespace(target_robot_id='robot1', active=active, reason=1,
                                     sequence=seq))


def command(w, cmd):
    w.node._on_command(SimpleNamespace(robot_id='robot1', command_id=f'c{w.clock.t}',
                                       command=cmd))


# ── 정상 · 경유지 실패 ──────────────────────────────────────────────────────
def test_normal_patrol_undocks_visits_all_waypoints_and_docks(world):
    w = world()
    outcome = w.node._run(w.gh, w.nav, w.evac)
    R = w.m.R
    assert outcome == (R.SUCCEEDED, 0, 'docked')
    assert w.nav.log == ['WP1', 'spin', 'WP2', 'spin', 'WP3', 'spin', '0.000,0.000']
    F = w.m.F
    order = [F.WAITING_FOR_TOKEN, F.UNDOCKING, F.INITIAL_POSE_READY, F.PATROLLING,
             F.WAYPOINT_REACHED, F.DOCKING]
    firsts = [states(w).index(s) for s in order]
    assert firsts == sorted(firsts)
    assert w.robot.undock_goals == 1 and w.robot.dock_goals == 1


def test_waypoint_failure_retries_once_then_skips(world):
    tries = {'WP2': 0}

    def plan(label):
        if label == 'WP2':
            tries['WP2'] += 1
            return 1.0, False
        return 1.0, True
    w = world(plan)
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert 'skipped: WP2' in outcome[2]
    assert tries['WP2'] == 2                               # 1회 재시도
    assert w.nav.log[:6] == ['WP1', 'spin', 'WP2', 'WP2', 'WP3', 'spin']   # WP2 회전도 생략


def test_two_consecutive_skips_return_to_dock(world):
    w = world(lambda label: (1.0, label not in ('WP2', 'WP3')))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert 'WP2' in outcome[2] and 'WP3' in outcome[2]
    assert w.nav.log[-1] == '0.000,0.000'                   # 귀환


# ── 도킹 재시도 ─────────────────────────────────────────────────────────────
def test_docking_retries_with_undock_reapproach_and_spin(world):
    w = world(dock_outcomes=['contact_no_charge', 'miss', 'ok'])
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert w.robot.dock_goals == 3
    assert w.robot.undock_goals == 2                       # 시작 1 + 접점만 붙은 뒤 1
    tail = w.nav.log[-4:]
    assert tail == ['0.000,0.000', '0.000,0.000', 'spin', '0.000,0.000']


def test_three_docking_failures_park_at_dock_front(world):
    w = world(dock_outcomes=['miss', 'miss', 'miss'])
    with pytest.raises(w.m.PatrolAbort) as end:
        w.node._run(w.gh, w.nav, w.evac)
    assert end.value.reason_code == w.m.R.REASON_DOCKING_FAILED
    assert 'waiting at dock front' in end.value.reason
    assert w.nav.position == (0.0, 0.0)


def test_unreachable_dock_front_parks_in_nearest_safe_zone_only(world):
    w = world(lambda label: (1.0, label != '0.000,0.000'))
    with pytest.raises(w.m.PatrolAbort) as end:
        w.node._run(w.gh, w.nav, w.evac)
    assert 'parked in nearest safe zone' in end.value.reason
    assert 'SAFE_B' not in w.nav.log                        # 가장 가까운 한 곳만


# ── 배터리 ──────────────────────────────────────────────────────────────────
def test_low_battery_returns_and_docks(world):
    w = world()
    w.clock.at(5.0, lambda: w.robot.battery(DISCHARGING, 0.10))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[:2] == (w.m.R.FAILED, w.m.R.REASON_LOW_BATTERY)
    assert 'WP3' not in w.nav.log
    assert w.nav.log[-1] == '0.000,0.000'


# ── EStop ───────────────────────────────────────────────────────────────────
def test_short_estop_pulse_still_needs_resume_command(world):
    w = world()
    R = w.m.R
    w.clock.at(8.0, lambda: estop(w, True, 10))
    w.clock.at(8.2, lambda: estop(w, False, 11))            # 0.2초 만에 해제
    resumed = {}

    def resume():
        resumed['t'] = w.clock.t
        command(w, w.m.PatrolCommand.RESUME_PATROL)
    w.clock.at(30.0, resume)
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == R.SUCCEEDED
    assert w.m.F.BLOCKED in states(w)
    assert resumed['t'] > 8.2 + 1000.0


def test_commands_during_active_estop_are_ignored(world):
    w = world()
    w.clock.at(8.0, lambda: estop(w, True, 10))
    w.clock.at(9.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))   # EStop 중
    w.clock.at(12.0, lambda: estop(w, False, 11))
    w.clock.at(40.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
    started_after = []
    original = w.nav._start

    def track(label, target):
        started_after.append((w.clock.t, label))
        return original(label, target)
    w.nav._start = track
    w.node._run(w.gh, w.nav, w.evac)
    moves = [t for t, _ in started_after if 1008.0 < t < 1040.0]
    assert moves == []                                      # 재개 명령 전까지 정지


def test_estop_during_docking_cancels_dock_and_resumes_after_command(world):
    w = world(plan=lambda label: (1.0, True))
    calls = {'n': 0}
    original = w.robot.client('dock').send_goal_async

    class Dock:
        def wait_for_server(self, timeout_sec=None):
            return True

        def send_goal_async(self, goal):
            calls['n'] += 1
            if calls['n'] == 1:
                w.clock.at(1.0, lambda: estop(w, True, 20))
                w.clock.at(3.0, lambda: estop(w, False, 21))
                w.clock.at(5.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
            return original(goal)
    w.node._dock_client = Dock()
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert w.robot.canceled >= 1                            # 도킹 동작 취소
    assert calls['n'] == 2                                  # 재개 뒤 같은 시도를 다시


def test_estop_then_move_to_safe_zone_goes_to_safe_zone(world):
    w = world()
    w.clock.at(8.0, lambda: estop(w, True, 10))
    w.clock.at(9.0, lambda: estop(w, False, 11))
    w.clock.at(10.0, lambda: command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE))
    w.clock.at(30.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert 'SAFE_A' in w.nav.log


# ── 안전구역 · 토큰 ─────────────────────────────────────────────────────────
def test_safe_zone_wait_survives_estop_and_resumes_on_command(world):
    w = world()
    w.clock.at(8.0, lambda: command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE))
    w.clock.at(15.0, lambda: estop(w, True, 10))           # 안전구역 대기 중 EStop
    w.clock.at(16.0, lambda: estop(w, False, 11))
    w.clock.at(20.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))  # 재개 = 차량 출차
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    log = w.nav.log
    assert log.count('SAFE_A') == 1
    assert w.m.F.BLOCKED in states(w)


def test_token_lost_too_long_fails(world):
    w = world()

    def drop():
        w.token['valid'] = False
    w.clock.at(8.0, drop)
    with pytest.raises(w.m.PatrolAbort) as end:
        w.node._run(w.gh, w.nav, w.evac)
    assert end.value.reason_code == w.m.R.REASON_TOKEN_EXPIRED


def test_token_gap_shorter_than_timeout_continues(world):
    w = world()

    def set_token(value):
        w.token['valid'] = value
    w.clock.at(8.0, lambda: set_token(False))
    w.clock.at(12.0, lambda: set_token(True))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert w.m.F.BLOCKED in states(w)


# ── 감지 처리 중 대피 ───────────────────────────────────────────────────────
def test_move_to_safe_zone_during_detection_cancels_detection(world):
    w = world()
    canceled = {'n': 0}
    result = Later()

    def cancel():
        canceled['n'] += 1
        w.clock.at(0.5, lambda: result.complete(SimpleNamespace(status=5, result=None)))
        return Done()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                             cancel_goal_async=cancel)
    w.node.detect_client = SimpleNamespace(wait_for_server=lambda timeout_sec=None: True,
                                           send_goal_async=lambda goal: Done(handle))

    def stop_request():
        from rclpy.task import Future
        w.node._detect_request = Future()
    w.clock.at(5.0, stop_request)
    w.clock.at(9.0, lambda: command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE))
    w.clock.at(30.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert canceled['n'] == 1
    assert 'SAFE_A' in w.nav.log
    assert w.m.F.DETECTION_PROCESSING in states(w)


# ── 재검토에서 찾은 경우들 ───────────────────────────────────────────────────
def detection_world(world, result_after=None):
    """정지 요청 → DetectEvent. result_after 초 뒤 확정 실패 결과, None이면 응답 없음."""
    w = world()
    canceled = {'n': 0}
    result = Later()

    def cancel():
        canceled['n'] += 1
        w.clock.at(0.5, lambda: result.complete(SimpleNamespace(status=5, result=None)))
        return Done()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                             cancel_goal_async=cancel)
    sent = []

    def send(goal):
        sent.append(w.clock.t)
        return Done(handle)
    w.node.detect_client = SimpleNamespace(wait_for_server=lambda timeout_sec=None: True,
                                           send_goal_async=send)

    def stop_request():
        from rclpy.task import Future
        with w.node._lock:
            w.node._detect_request = Future()
    w.stop_request = stop_request
    w.canceled, w.sent = canceled, sent
    return w


def test_patrol_cancel_during_detection_cancels_detect_event(world):
    w = detection_world(world)
    w.clock.at(5.0, w.stop_request)

    def cancel_patrol():
        w.gh.is_cancel_requested = True
    w.clock.at(9.0, cancel_patrol)
    with pytest.raises(w.m.PatrolAbort) as end:
        w.node._run(w.gh, w.nav, w.evac)
    assert end.value.outcome == w.m.R.CANCELED
    assert w.canceled['n'] == 1                             # 감지 노드도 멈춘다


def test_resume_arriving_while_stop_is_being_confirmed_is_kept(world):
    w = world()
    w.clock.at(8.0, lambda: estop(w, True, 10))
    w.clock.at(8.01, lambda: estop(w, False, 11))
    # 정지 확인이 끝나기 전에(같은 0.05초 안에) 재개 명령 도착
    w.clock.at(8.02, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
    original = w.evac.stop

    def slow_stop():
        original()
        w.clock.advance(1.0)                                # 정지 확인에 1초
    w.evac.stop = slow_stop
    outcome = w.node._run(w.gh, w.nav, w.evac)              # 영원히 BLOCKED면 끝나지 않음
    assert outcome[0] == w.m.R.SUCCEEDED


def test_command_before_release_frame_is_taken_on_resend(world):
    w = world()
    w.clock.at(8.0, lambda: estop(w, True, 10))
    cmd = SimpleNamespace(robot_id='robot1', command_id='resume-1',
                          command=w.m.PatrolCommand.RESUME_PATROL)
    w.clock.at(9.0, lambda: w.node._on_command(cmd))       # 해제 프레임보다 먼저 도착 → 거절
    w.clock.at(9.1, lambda: estop(w, False, 11))
    w.clock.at(10.0, lambda: w.node._on_command(cmd))      # 관제 재전송(같은 command_id)
    w.clock.at(11.0, lambda: w.node._on_command(cmd))      # 한 번 더 와도 한 번만 적용
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert not w.node._resume_command                       # 중복 적용으로 남은 플래그 없음


def test_estop_inside_token_loss_inside_safe_zone_keeps_the_resume(world):
    w = world(plan=lambda label: (6.0, True) if label == 'SAFE_A' else (1.0, True))

    def set_token(value):
        w.token['valid'] = value
    w.clock.at(5.0, lambda: command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE))
    w.clock.at(7.0, lambda: set_token(False))              # 안전구역으로 가는 중 토큰 상실
    w.clock.at(8.0, lambda: estop(w, True, 10))            # 그 사이 EStop
    w.clock.at(9.0, lambda: estop(w, False, 11))
    w.clock.at(9.5, lambda: set_token(True))
    w.clock.at(10.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))   # 차량은 이미 출차
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED                    # 안전구역에서 영원히 기다리지 않음


def test_pending_stop_request_is_dropped_when_evacuating(world):
    w = detection_world(world)

    def both():
        w.stop_request()
        command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE)     # 대피 명령이 함께 옴
    w.clock.at(5.0, both)
    w.clock.at(20.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert w.sent == []                                     # 안전구역 뒤 엉뚱한 곳에서 감지 처리 없음


def test_safe_zone_uses_amcl_pose_when_tf_is_stale(world):
    w = world()

    def stale():
        raise RuntimeError('Current map position is stale')
    w.evac.position = stale
    w.node.pose = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=8.0, y=8.0)))
    w.clock.at(5.0, lambda: command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE))
    w.clock.at(20.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert 'SAFE_B' in w.nav.log                            # amcl 위치(8,8) 기준 가장 가까운 곳


def test_dock_result_before_dock_status_is_not_a_failure(world):
    """연결 확인에서 찾은 경우: Dock 결과가 dock_status(0.5초 주기)보다 먼저 온다."""
    w = world(status_lag=0.6)
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert w.robot.dock_goals == 1                          # 1차 시도에서 성공


def test_dock_retry_waits_until_canceled_dock_has_ended(world):
    """
    취소한 Dock이 끝난 뒤에 다음 Dock을 보낸다.

    Gazebo에서 찾은 경우: 끝나기 전에 보내면 Create 3가
    'A docking behavior is already running'으로 거절한다.
    """
    # 로봇이 이미 도크 앞에 있어 재접근 주행이 바로 끝난다(Gazebo에서 0.3초).
    w = world(plan=lambda label: (0.1, True) if label == '0.000,0.000' else (1.0, True),
              dock_outcomes=('stuck', 'ok'), cancel_lag=2.0)
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert w.robot.rejected == 0                            # 2차 Dock이 거절되지 않음
    assert w.robot.dock_goals == 2                          # 2차 시도에서 성공
    assert 'ALERT robot1: docking attempt 1/3 failed' in w.logs


def test_stop_not_confirmed_once_is_checked_again(world):
    """Gazebo에서 찾은 경우: 늦게 들어온 odometry 때문에 정지 확인이 실패해도 순찰을 끝내지 않는다."""
    w = world()
    w.evac.unconfirmed_stops = 1
    w.clock.at(8.0, lambda: command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE))
    w.clock.at(40.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert 'SAFE_A' in w.nav.log


def test_stop_never_confirmed_waits_blocked_then_continues(world):
    w = world()
    w.evac.unconfirmed_stops = 5                      # 확인 3번 실패 → BLOCKED, 그 뒤 확인됨
    w.clock.at(8.0, lambda: command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE))
    w.clock.at(120.0, lambda: command(w, w.m.PatrolCommand.RESUME_PATROL))
    outcome = w.node._run(w.gh, w.nav, w.evac)
    assert outcome[0] == w.m.R.SUCCEEDED
    assert w.m.F.BLOCKED in states(w)
    assert any('stop not confirmed 3 times' in str(line) for line in w.logs)


def test_cancel_while_stop_is_unconfirmed_ends_the_goal(world):
    w = world()
    w.evac.unconfirmed_stops = 50
    w.clock.at(8.0, lambda: command(w, w.m.PatrolCommand.MOVE_TO_SAFE_ZONE))
    w.clock.at(60.0, lambda: setattr(w.gh, 'is_cancel_requested', True))
    with pytest.raises(w.m.PatrolAbort) as end:
        w.node._run(w.gh, w.nav, w.evac)
    assert end.value.outcome == w.m.R.CANCELED
