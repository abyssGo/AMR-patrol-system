"""CR 9/9 yaw alignment in the detection node, with a simulated camera and clock."""

import importlib
import math
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

W = 640
HFOV = math.radians(64.0)


@pytest.fixture
def module(monkeypatch):
    ultralytics = ModuleType('ultralytics')
    ultralytics.YOLO = lambda *_: pytest.fail('no model in this test')
    monkeypatch.setitem(sys.modules, 'ultralytics', ultralytics)
    return importlib.import_module('patrol_amr_safety.vision_node_v2')


# ── horizontal_error / yaw rate ─────────────────────────────────────────────
def test_horizontal_error_sign_and_range(module):
    assert module.horizontal_error((310, 0, 330, 1), W) == pytest.approx(0.0)
    assert module.horizontal_error((0, 0, 0, 1), W) == pytest.approx(-1.0)      # 왼쪽 끝
    assert module.horizontal_error((W, 0, W, 1), W) == pytest.approx(1.0)       # 오른쪽 끝


@pytest.mark.parametrize('error', [0.06, 0.2, 0.5, 1.0])
def test_target_right_turns_clockwise_within_limits(module, error):
    w = module.yaw_rate_for_error(error)
    assert w < 0.0
    assert module.ALIGN_YAW_MIN <= abs(w) <= module.ALIGN_YAW_MAX
    assert module.yaw_rate_for_error(-error) == pytest.approx(-w)              # 왼쪽은 반시계


def test_yaw_rate_is_clamped(module):
    assert abs(module.yaw_rate_for_error(0.06)) == pytest.approx(module.ALIGN_YAW_MIN)
    assert abs(module.yaw_rate_for_error(1.0)) == pytest.approx(module.ALIGN_YAW_MAX)


# ── _align with a simulated robot ───────────────────────────────────────────
class Sim:
    """Robot yaw integrates the published cmd_vel_yaw; the camera sees a fixed target."""

    def __init__(self, module, target_bearing, visible=lambda t: True, frame_period=0.1):
        self.m = module
        self.t = 100.0
        self.yaw = 0.0
        self.w = 0.0
        self.target = target_bearing        # rad, robot frame at yaw 0 (CCW +)
        self.visible = visible
        self.frame_period = frame_period
        self.next_frame = self.t
        self.max_w = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        end = self.t + dt
        while self.t < end - 1e-9:
            step = min(0.01, end - self.t)
            self.yaw += self.w * step
            self.t += step
            if self.t >= self.next_frame:
                self.next_frame += self.frame_period
                self.capture()

    def capture(self):
        rel = self.target - self.yaw
        detection = None
        if self.visible(self.t) and abs(rel) < HFOV / 2:
            cx = W / 2 * (1 - math.tan(rel) / math.tan(HFOV / 2))
            detection = ('fire', 0.9, (int(cx - 10), 100, int(cx + 10), 200))
        frame = SimpleNamespace(shape=(480, W, 3))
        with self.node._latest_lock:
            self.node._latest = (self.t, detection, frame)
        with self.node._odom_lock:
            self.node._odom_motion = (self.t, 0.0, abs(self.w))

    def publish_yaw(self, w):
        self.w = w
        self.max_w = max(self.max_w, abs(w))


def make_node(module, monkeypatch, sim):
    node = module.DetectingNode.__new__(module.DetectingNode)
    node._latest_lock = threading.Lock()
    node._latest = None
    node._odom_lock = threading.Lock()
    node._odom_motion = None
    node._publish_yaw = sim.publish_yaw
    node._detect_feedback = lambda *_a, **_k: None
    sim.node = node
    monkeypatch.setattr(module.time, 'monotonic', sim.monotonic)
    monkeypatch.setattr(module.time, 'sleep', sim.sleep)
    sim.capture()
    return node


def matches(detection):
    return detection is not None and detection[1] >= 0.7 and detection[0] == 'fire'


GOAL = SimpleNamespace(is_cancel_requested=False)


@pytest.mark.parametrize('bearing_deg', [-25.0, -8.0, 8.0, 25.0])
def test_alignment_centres_the_target_and_stops(module, monkeypatch, bearing_deg):
    sim = Sim(module, math.radians(bearing_deg))
    node = make_node(module, monkeypatch, sim)
    assert node._align(GOAL, matches) == 'aligned'
    _, (_, _, bbox), frame = node._latest
    assert abs(module.horizontal_error(bbox, frame.shape[1])) <= module.ALIGN_ERROR_TOLERANCE
    assert sim.w == 0.0                                   # 정렬 끝에 yaw 0
    assert sim.max_w <= module.ALIGN_YAW_MAX + 1e-9


def test_candidate_gap_over_0_6_s_fails(module, monkeypatch):
    start = 100.0
    sim = Sim(module, math.radians(20.0), visible=lambda t: t < start + 1.0)
    node = make_node(module, monkeypatch, sim)
    assert node._align(GOAL, matches) == 'candidate lost'
    assert sim.t - (start + 1.0) <= module.CANDIDATE_GAP_SEC + 0.6


def test_short_gap_is_tolerated(module, monkeypatch):
    sim = Sim(module, math.radians(20.0), visible=lambda t: not (100.5 <= t < 100.9))
    node = make_node(module, monkeypatch, sim)
    assert node._align(GOAL, matches) == 'aligned'


def test_timeout_after_10_s(module, monkeypatch):
    sim = Sim(module, math.radians(20.0))
    node = make_node(module, monkeypatch, sim)
    sim.publish_yaw = lambda w: None                      # 로봇이 돌지 않는다(바퀴 차단 등)
    node._publish_yaw = sim.publish_yaw
    outcome = node._align(GOAL, matches)
    assert outcome.startswith('timeout')
    assert sim.t - 100.0 == pytest.approx(module.ALIGN_TIMEOUT_SEC + module.STOP_HOLD_SEC,
                                          abs=0.2)


def test_alignment_waits_for_a_real_stop_first(module, monkeypatch):
    sim = Sim(module, math.radians(10.0))
    node = make_node(module, monkeypatch, sim)
    node._is_stopped = lambda: False                      # 로봇이 아직 움직이는 중
    assert node._align(GOAL, matches) == 'robot not stopped before alignment'
    assert sim.max_w == 0.0                               # 정지 확인 전에는 회전하지 않는다
