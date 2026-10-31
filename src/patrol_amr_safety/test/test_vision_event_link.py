"""Exercise the actual vision report methods without a camera or YOLO model."""

import importlib
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import numpy as np
from patrol_interfaces.srv import ReportDetection
import pytest
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class Client:
    """
    Service client stub; `responses` pre-programs report answers in call order.

    A response of None leaves the future unanswered (no response from the server);
    an Exception instance fails the call.
    """

    def __init__(self):
        self.calls = []
        self.ready = True
        self.responses = []

    def service_is_ready(self):
        return self.ready

    def wait_for_service(self, timeout_sec=None):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        if self.responses:
            answer = self.responses.pop(0)
            if isinstance(answer, Exception):
                future.set_exception(answer)
            elif answer is not None:
                future.set_result(answer)
        return future


class InlineThread:
    """Run the report worker synchronously so the test sees its effects."""

    def __init__(self, target, args=(), daemon=None):
        self.target, self.args = target, args

    def start(self):
        self.target(*self.args)


@pytest.fixture
def vision(monkeypatch, tmp_path):
    ultralytics = ModuleType('ultralytics')
    ultralytics.YOLO = lambda *_: pytest.fail('A model must not load in this test')
    monkeypatch.setitem(sys.modules, 'ultralytics', ultralytics)
    module = importlib.import_module('patrol_amr_safety.vision_node_v2')
    monkeypatch.setattr(module.threading, 'Thread', InlineThread)
    monkeypatch.setattr(module, 'REPORT_TIMEOUT_SEC', 0.05)
    monkeypatch.setattr(module, 'UNSTORED_REPORT_DIR', tmp_path / 'reports')

    class Harness(module.DetectingNode):
        def __init__(self):
            self.class_name, self.confidence, self.bbox = 'fire', 0.9, (1, 1, 10, 10)
            self.event_type_map = {'fire': ReportDetection.Request.FIRE}
            self.submit_client, self.stop_client = Client(), Client()
            self._pending_futures = set()
            self._frames_received = self._frames_processed = 0
            self._last_image_at = self._last_pose_at = None
            self._last_detection = 'none'
            self._last_boxes_count = 0
            self.published, self.errors, self.recorded = [], [], []
            self.robot_id = 'robot1'
            self.camera_topic = '/robot1/oakd/rgb/image_raw/compressed'
            self._latest_lock = threading.Lock()
            self._latest = None
            self._goal_active = False
            self._candidate = None
            self._odom_lock = threading.Lock()
            self._odom_motion = None
            self._dismissed = []
            self.yaw = []
            self.yaw_pub = SimpleNamespace(
                publish=lambda msg: self.yaw.append(msg.twist.angular.z))
            self.patrol_events = module.VisionPatrolLink(self, self.robot_id)
            self.reports_dir = tmp_path / 'reports'

        def create_client(self, service_type, name):
            assert service_type is SetBool
            assert name == '/robot1/patrol_stop'
            return self.stop_client

        def create_publisher(self, message_type, name, depth):
            assert name == '/robot1/patrol_resume'
            assert depth == 2
            return SimpleNamespace(publish=lambda msg: self.published.append(msg.data))

        def get_logger(self):
            return SimpleNamespace(
                error=self.errors.append,
                info=lambda _, **kwargs: None,
                warning=lambda _, **kwargs: None,
            )

        def get_namespace(self):
            return '/robot1'

        def get_clock(self):
            return SimpleNamespace(now=lambda: Time(seconds=10))

        def get_current_position(self):
            return (1.0, 2.0)

        def compute_theta(self, *_):
            return 0.5

        def record_reported_event(self, *args):
            self.recorded.append(args)

    return Harness()


def test_real_node_initialization_connects_robot1_stop_and_resume(monkeypatch):
    ultralytics = ModuleType('ultralytics')
    ultralytics.YOLO = lambda *_: pytest.fail('A model must not load in this test')
    monkeypatch.setitem(sys.modules, 'ultralytics', ultralytics)
    module = importlib.import_module('patrol_amr_safety.vision_node_v2')
    clients, publishers, model_paths = {}, {}, []

    def fake_model(path):
        model_paths.append(path)
        return SimpleNamespace(names={})

    def create_client(_node, service_type, name, **_kwargs):
        client = Client()
        clients[name] = (service_type, client)
        return client

    def create_publisher(_node, message_type, name, depth):
        publisher = SimpleNamespace(publish=lambda _: None)
        publishers[name] = (message_type, depth, publisher)
        return publisher

    monkeypatch.setattr(module, 'YOLO', fake_model)
    monkeypatch.setattr(module.Node, '__init__', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(module.DetectingNode, 'create_client', create_client)
    monkeypatch.setattr(module.DetectingNode, 'create_publisher', create_publisher)
    monkeypatch.setattr(module.DetectingNode, 'create_subscription',
                        lambda *_args, **_kwargs: SimpleNamespace())
    monkeypatch.setattr(module.DetectingNode, 'create_timer',
                        lambda *_args, **_kwargs: SimpleNamespace())
    monkeypatch.setattr(module.DetectingNode, 'get_namespace', lambda _node: '/robot1')
    monkeypatch.setattr(module, 'ActionServer', lambda *_args, **_kwargs: SimpleNamespace())
    monkeypatch.setattr(module.DetectingNode, 'get_logger', lambda _: SimpleNamespace(
        info=lambda *_args, **_kwargs: None,
        warning=lambda *_args, **_kwargs: None,
        error=lambda *_args, **_kwargs: None,
    ))

    node = module.DetectingNode()

    assert model_paths == [str(module.MODEL_PATH)]
    assert node.pose_topic == '/robot1/amcl_pose'          # 설계도: map 좌표
    assert node.camera_topic == '/robot1/oakd/rgb/image_raw/compressed'
    assert isinstance(node.patrol_events, module.VisionPatrolLink)
    assert clients['/robot1/patrol_stop'] == (SetBool, node.patrol_events.client)
    assert publishers['/robot1/patrol_resume'] == (
        Bool, 2, node.patrol_events.resume_pub)
    assert clients[module.SUBMIT_SERVICE] == (ReportDetection, node.submit_client)
    assert '/robot1/cmd_vel_yaw' in publishers                  # 정렬 회전은 후보로만 낸다
    assert '/robot1/cmd_vel' not in publishers                  # 최종 출력은 local safety만


def stop_confirmed(vision):
    assert vision.patrol_events.request()
    request, pending = vision.stop_client.calls[-1]
    assert request.data is False
    assert vision.submit_client.calls == []
    assert vision.published == []
    pending.set_result(SetBool.Response(success=True, message='Stopped'))
    assert vision.patrol_events.stopped


def report(vision, *responses, event_id='det-1'):
    vision.submit_client.responses = list(responses)
    stop_confirmed(vision)
    vision.submit_event(np.zeros((20, 20, 3), dtype=np.uint8), event_id=event_id)
    return [request for request, _ in vision.submit_client.calls]


def answer(status):
    return ReportDetection.Response(status=status)


def test_stored_report_resumes_and_records(vision):
    requests = report(vision, answer(ReportDetection.Response.STORED))
    assert len(requests) == 1
    assert requests[0].robot_id == 'robot1'
    assert requests[0].event_id == 'det-1'                       # event_id = detection_id
    assert vision.published == [True]
    assert vision.recorded == [('fire', (1.0, 2.0), 0.5)]
    assert not vision.patrol_events.busy


def test_duplicate_counts_as_stored(vision):
    report(vision, answer(ReportDetection.Response.DUPLICATE))
    assert vision.published == [True]
    assert vision.recorded == [('fire', (1.0, 2.0), 0.5)]


def test_rejected_report_resumes_without_recording(vision):
    report(vision, answer(ReportDetection.Response.REJECTED))
    assert vision.published == [True]          # 순찰이 60초 동안 멈춰 있지 않게 재개
    assert not vision.recorded
    assert vision.errors


def test_no_response_retries_with_the_same_request(vision):
    requests = report(vision, None, answer(ReportDetection.Response.STORED))
    assert len(requests) == 2
    assert requests[0] is requests[1]           # 같은 event_id · 같은 내용
    assert vision.published == [True]


def test_transport_error_retries_then_gives_up_after_three(vision):
    requests = report(vision, RuntimeError('down'), None, None)
    assert len(requests) == 3
    assert {r.event_id for r in requests} == {'det-1'}
    assert vision.published == [True]           # 재시도를 다 써도 순찰은 재개
    assert not vision.recorded
    assert vision.errors


def test_report_snapshot_survives_later_camera_updates(vision):
    vision.submit_client.responses = []
    stop_confirmed(vision)
    vision.submit_event(np.zeros((20, 20, 3), dtype=np.uint8), event_id='det-2')
    request = vision.submit_client.calls[0][0]
    vision.class_name, vision.bbox, vision.confidence = 'leak', None, 0.0
    assert request.event_type == ReportDetection.Request.FIRE
    assert (request.position.x, request.position.y) == (1.0, 2.0)


def test_report_uses_the_confirmed_detection_even_if_the_camera_reset(vision):
    vision.submit_client.responses = []
    stop_confirmed(vision)
    vision.class_name, vision.bbox, vision.confidence = None, None, 0.0   # 카메라 콜백이 초기화
    vision.submit_event(np.zeros((20, 20, 3), dtype=np.uint8), event_id='det-3',
                        detection=('fire', 0.91, (2, 2, 8, 8)))
    request = vision.submit_client.calls[0][0]
    assert request.event_id == 'det-3'
    assert request.event_type == ReportDetection.Request.FIRE
    assert request.detected_at.sec == 10
    assert len(request.image) > 0


@pytest.mark.parametrize('transport_error', [False, True])
def test_failed_stop_never_reports_or_resumes_and_backs_off(vision, transport_error):
    assert vision.patrol_events.request()
    pending = vision.stop_client.calls[-1][1]
    if transport_error:
        pending.set_exception(RuntimeError('Stop service failed'))
    else:
        pending.set_result(SetBool.Response(success=False, message='not driving the route'))
    assert not vision.submit_client.calls
    assert not vision.published
    assert not vision.patrol_events.busy
    assert not vision.patrol_events.request()   # 거절 직후에는 프레임마다 다시 보내지 않는다
    assert len(vision.stop_client.calls) == 1


def test_unavailable_stop_service_does_not_start_report(vision):
    vision.stop_client.ready = False
    assert not vision.patrol_events.request()
    assert not vision.submit_client.calls
    assert not vision.published


def test_synchronous_stop_transport_error_allows_later_retry(vision):
    def failed_call(_):
        raise RuntimeError('Transport error')
    vision.stop_client.call_async = failed_call
    with pytest.raises(RuntimeError, match='Transport error'):
        vision.patrol_events.request()
    assert not vision.patrol_events.busy


def test_first_bbox_requests_stop_and_leaves_report_to_detect_event(vision, monkeypatch):
    """설계도: 후보가 보이면 정지만 요청하고, 정렬 · 검증 · 보고는 DetectEvent에서 한다."""
    module = sys.modules['patrol_amr_safety.vision_node_v2']
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    result = SimpleNamespace(plot=lambda: frame, boxes=[])
    vision.model = SimpleNamespace(predict=lambda **_: [result])
    vision.select_detection = lambda _: ('fire', 0.9, (1, 1, 10, 10))
    vision.is_known_event = lambda *_: False
    vision.class_name = vision.last_seen = None
    vision.reported = False
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(module.cv2, 'imdecode', lambda *_: frame)
    image = SimpleNamespace(data=b'')

    vision.image_callback(image)
    assert len(vision.stop_client.calls) == 1          # 첫 bbox에서 바로 정지 요청
    assert vision._candidate == 'fire'                  # 정렬 대상 종류를 기억
    vision.stop_client.calls[0][1].set_result(SetBool.Response(success=True))
    clock.now = 12.0
    vision.image_callback(image)
    assert len(vision.stop_client.calls) == 1          # 같은 사건으로 다시 정지 요청하지 않음
    assert not vision.submit_client.calls              # 보고는 DetectEvent 확정 뒤에만
    assert not vision.published


def test_rejected_report_keeps_evidence_on_the_robot(vision):
    report(vision, answer(ReportDetection.Response.REJECTED))
    saved = sorted(p.name for p in vision.reports_dir.iterdir())
    assert saved == ['det-1.jpg', 'det-1.json']                 # 저장 안 된 사건을 잃지 않는다


def test_unanswered_report_keeps_evidence_on_the_robot(vision):
    report(vision, None, None, None)
    assert (vision.reports_dir / 'det-1.jpg').exists()


def test_large_evidence_is_shrunk_below_the_limit(vision):
    module = sys.modules['patrol_amr_safety.vision_node_v2']
    noisy = np.random.default_rng(0).integers(0, 255, (1440, 2560, 3), dtype=np.uint8)
    data = module.encode_evidence(noisy)
    assert data is not None and len(data) <= module.MAX_IMAGE_BYTES
