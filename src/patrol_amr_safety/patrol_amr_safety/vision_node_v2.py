#!/usr/bin/env python3
"""
AMR local detection node: 화재 · 누수 · 적치물 감지 → 정지 요청 → 정렬 · 검증(DetectEvent) → 보고.

설계도(A-1_지능1_시스템_설계도) 01 · 03 페이지 기준:
- 후보가 보이면 patrol_stop(SetBool data=false)으로 순찰 정지를 요청한다.
- mission_supervisor가 보내는 DetectEvent Goal을 받아 ALIGNING(CR 9/9: 화면 중심 오차로
  /{robot}/cmd_vel_yaw 제자리 회전) → VERIFYING(min_confidence 이상 1초 유지) 후
  확정 여부를 돌려준다. 최종 cmd_vel은 local_safety_supervisor만 낸다.
- 확정이면 detection_id를 event_id로 ReportDetection을 보내고(응답이 없으면 같은
  event_id로 재시도), 저장 결과를 받으면 patrol_resume(true).
- 위치는 amcl_pose(map 좌표)를 쓴다.
"""
import json
import math
from pathlib import Path
import threading
import time
import uuid

import cv2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
from patrol_amr_interfaces.action import DetectEvent
from patrol_interfaces.srv import ReportDetection
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

if __package__:
    from .event_check import VisionPatrolLink
else:
    from event_check import VisionPatrolLink


DEFAULT_ROBOT_ID = 'robot1'
# 카메라: /{robot}/oakd/rgb/image_raw/compressed (카메라 드라이버에서 컴프레스드 발행을 켜야 함)
# 위치: /{robot}/amcl_pose (map 좌표)
SUBMIT_SERVICE = '/system_monitor/report_detection'

# 팀 AMR PC 배치 그대로 ~/patrol/detection_best/detection_best.pt (사용자 홈 기준)
MODEL_PATH = Path.home() / 'patrol' / 'detection_best' / 'detection_best.pt'

# DetectEvent 정렬 (CR-AMR_09-09_10-53 yaw 정렬 Nav2 중재 확정)
ALIGN_ERROR_TOLERANCE = 0.05     # |horizontal_error| 이하이면 정렬 범위
ALIGN_HOLD_SEC = 0.5             # 정렬 범위 연속 유지시간
ALIGN_YAW_MIN = 0.08             # rad/s, yaw 각속도 절댓값 하한
ALIGN_YAW_MAX = 0.25             # rad/s, yaw 각속도 절댓값 상한
ALIGN_YAW_GAIN = 0.5             # rad/s per unit horizontal_error (하한·상한 안에서)
ALIGN_TIMEOUT_SEC = 10.0         # 정렬 시작 후 timeout
CANDIDATE_GAP_SEC = 0.6          # 마지막 유효 후보 이후 이 시간을 넘으면 실패
ALIGN_LOOP_PERIOD_SEC = 0.05
# 실제 정지 확인 (CR 9/9, docs/amr.md 5.2절)
STOP_LINEAR_MAX = 0.05           # m/s
STOP_ANGULAR_MAX = 0.1           # rad/s
STOP_HOLD_SEC = 0.5
STOP_ODOM_MAX_AGE_SEC = 0.5
STOP_CONFIRM_TIMEOUT_SEC = 5.0
VERIFY_TIMEOUT_SEC = 6.0
# ReportDetection 재시도 (scenarios.md 3장: 응답이 없을 때만 같은 event_id로)
REPORT_ATTEMPTS = 3
REPORT_TIMEOUT_SEC = 5.0
DISMISS_COOLDOWN_SEC = 60.0                # 확정 실패한 자리의 같은 종류 후보를 무시하는 시간

YOLO_CONFIDENCE = 0.70
YOLO_IMAGE_SIZE = 704

HOLD_DURATION_SEC = 1.0
MAX_FRAME_GAP_SEC = 0.30
STATUS_LOG_PERIOD_SEC = 5.0  # 진단 출력 주기이며 정지/복구 판단에는 사용하지 않음

MAX_IMAGE_BYTES = 1024 * 1024  # 1 MiB. 인터페이스가 요구하는 상한
JPEG_QUALITY_FALLBACK = 50      # 1차 인코딩이 상한을 넘으면 품질 낮춰서 재시도
EVIDENCE_SCALES = (0.75, 0.5, 0.35)  # 그래도 넘으면 이 비율로 줄여 가며 상한 안에 맞춘다
# 저장되지 못한 보고(거절 · 응답 없음)는 사진과 내용을 여기에 남긴다
UNSTORED_REPORT_DIR = Path.home() / '.ros' / 'patrol_reports'

EVENT_TYPE_NAMES = ('fire', 'leak', 'obstacle')

# ------------------------------------------------------------------
# 중복 식별(같은 로봇 안에서 같은 사건 재보고 방지) — 레이(위치+방향) 교차 기반.
# 로봇 위치만으로는 실제 사건 위치를 알 수 없어(뎁쓰 미사용), 로봇 위치·yaw +
# bbox의 화면 내 위치(화각으로 각도 환산)로 "시선"을 구해 두 시선이 실제로
# 교차하는지로 판정한다. 뎁쓰 카메라는 연산량·레이턴시 문제로 쓰지 않는다.
# CR-비전_09-10_12-01 3절 참고.
# ------------------------------------------------------------------
CAMERA_HORIZONTAL_FOV_DEG = 64.0  # /robot1/oakd/rgb/preview/camera_info 기준 실측

NEAR_ORIGIN_THRESHOLD_M = 0.5      # 로봇이 거의 같은 자리(이 거리 이내)에서 본 것끼리는 각도 직접 비교
BEARING_NEAR_TOLERANCE_RAD = math.radians(15.0)   # 같은 자리 판정 시 각도 허용 오차
MIN_RAY_ANGLE_DIFF_RAD = math.radians(15.0)       # 두 레이 각도차가 이보다 작으면(거의 평행) 삼각측량 신뢰 불가
MAX_SENSOR_RANGE_M = 5.0           # 교차점이 이보다 멀면(비현실적) 무시


def encode_evidence(frame):
    """
    Return JPEG bytes of the evidence frame within MAX_IMAGE_BYTES, or None.

    품질 90 → 50으로 먼저 줄이고, 그래도 1 MiB를 넘으면 해상도를 줄여서 맞춘다.
    상한 때문에 시스템 모니터가 거절(REJECTED)하는 일이 없게 보내기 전에 맞춘다.
    """
    attempts = [(frame, 90), (frame, JPEG_QUALITY_FALLBACK)]
    for scale in EVIDENCE_SCALES:
        small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        attempts.append((small, JPEG_QUALITY_FALLBACK))
    for image, quality in attempts:
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok and 0 < encoded.nbytes <= MAX_IMAGE_BYTES:
            return encoded.tobytes()
    return None


def bearing_from_bbox(bbox, frame_width, hfov_rad):
    """
    Return the robot-frame bearing (rad, CCW +) of a bbox centre for a pinhole camera.

    yaw는 반시계 방향이 +이므로 화면 오른쪽에 있는 대상은 음의 각도다.
    이 각도만큼 회전하면 대상이 화면 가운데로 온다.
    """
    x1, _, x2, _ = bbox
    half = frame_width / 2.0
    ratio = ((x1 + x2) / 2.0 - half) / half          # -1 (left edge) .. +1 (right edge)
    return -math.atan(ratio * math.tan(hfov_rad / 2.0))


def horizontal_error(bbox, frame_width):
    """Return bbox 중심의 화면 가로 오차: 왼쪽 끝 -1 .. 가운데 0 .. 오른쪽 끝 +1."""
    x1, _, x2, _ = bbox
    half = frame_width / 2.0
    return ((x1 + x2) / 2.0 - half) / half


def yaw_rate_for_error(error):
    """
    horizontal_error에 대한 yaw 각속도(rad/s, 반시계 +).

    대상이 오른쪽(+)이면 시계 방향(-)으로 돈다. 절댓값은 0.08~0.25 rad/s.
    """
    magnitude = min(ALIGN_YAW_MAX, max(ALIGN_YAW_MIN, ALIGN_YAW_GAIN * abs(error)))
    return -math.copysign(magnitude, error)


class DetectingNode(Node):
    def __init__(self):
        super().__init__('detecting_node')
        # 로봇 ID는 실행 namespace(/robot1 · /robot6)에서 정한다(설계도: 두 AMR에 같은 노드).
        self.robot_id = self.get_namespace().strip('/') or DEFAULT_ROBOT_ID
        self.camera_topic = f'/{self.robot_id}/oakd/rgb/image_raw/compressed'
        self.pose_topic = f'/{self.robot_id}/amcl_pose'
        self.hfov = math.radians(CAMERA_HORIZONTAL_FOV_DEG)
        self.get_logger().info(
            f'VISION START | robot_id={self.robot_id} camera={self.camera_topic} '
            f'pose={self.pose_topic}'
        )

        best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.get_logger().info(f'YOLO loading | model={MODEL_PATH}')
        try:
            self.model = YOLO(str(MODEL_PATH))
        except Exception as error:
            self.get_logger().error(f'YOLO load failed | model={MODEL_PATH} error={error}')
            raise
        self.get_logger().info(
            f'YOLO ready | classes={self.model.names} '
            f'allowed={EVENT_TYPE_NAMES} confidence={YOLO_CONFIDENCE}'
        )
        self.event_type_map = {
            'fire': ReportDetection.Request.FIRE,
            'leak': ReportDetection.Request.LEAK,
            'obstacle': ReportDetection.Request.OBSTACLE,
        }

        self.class_name = None
        self.bbox = None
        self.confidence = 0.0
        self.hold_start = None
        self.last_seen = None
        self.latest_pose = None
        self.reported = False
        self._pending_futures = set()
        self._frames_received = 0
        self._frames_processed = 0
        self._last_image_at = None
        self._last_odom_at = None
        self._last_detection = 'none'
        self._last_boxes_count = 0

        # 같은 로봇이 보고한 사건들: [{"class_name", "x", "y", "theta"}, ...] (중복 식별용)
        self._reported_events = []
        self._MAX_REPORTED_EVENTS = 500  # 장시간 운용 시 메모리 상한(안전장치, 실제로 도달할 일은 거의 없음)

        # 최신 감지 결과: DetectEvent 실행 스레드가 읽는다. (monotonic time, detection, frame)
        self._latest_lock = threading.Lock()
        self._latest = None
        self._goal_active = False
        self._candidate = None       # class_name of the stop trigger
        # 확정되지 않은 후보: 같은 자리에서 같은 대상을 다시 잡아 회전 → 정지를 반복하지 않게 잠시 무시
        self._dismissed = []         # [{"class_name", "x", "y", "theta", "until"}]
        self._image_group = MutuallyExclusiveCallbackGroup()
        self._io_group = ReentrantCallbackGroup()

        self.image_sub = self.create_subscription(
            CompressedImage, self.camera_topic, self.image_callback, best_effort,
            callback_group=self._image_group
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.amcl_callback, 10,
            callback_group=self._io_group
        )
        # 정렬 전후 실제 정지 확인용 odometry (수신 시각, 선속도, 각속도)
        self._odom_lock = threading.Lock()
        self._odom_motion = None
        self.odom_sub = self.create_subscription(
            Odometry, f'/{self.robot_id}/odom', self._odom_callback, best_effort,
            callback_group=self._io_group
        )
        # 정렬 회전 후보: local_safety_supervisor가 Nav2 후보와 중재해 최종 cmd_vel로 낸다.
        self.yaw_pub = self.create_publisher(TwistStamped, f'/{self.robot_id}/cmd_vel_yaw', 10)
        self.submit_client = self.create_client(
            ReportDetection, SUBMIT_SERVICE, callback_group=self._io_group)
        self.patrol_events = VisionPatrolLink(self, self.robot_id)
        self.detect_server = ActionServer(
            self, DetectEvent, f'/{self.robot_id}/detect_event',
            execute_callback=self.execute_detect,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self._io_group)
        self._status_timer = self.create_timer(
            STATUS_LOG_PERIOD_SEC, self._log_status, callback_group=self._io_group)
        self.get_logger().info(
            f'VISION READY | waiting for camera frames; report_service={SUBMIT_SERVICE} '
            f'status_log_period={STATUS_LOG_PERIOD_SEC}s'
        )

    def _log_status(self):
        now = time.monotonic()
        camera_age = ('never' if self._last_image_at is None
                      else f'{now - self._last_image_at:.1f}s')
        odom_age = 'never' if self._last_odom_at is None else f'{now - self._last_odom_at:.1f}s'
        self.get_logger().info(
            f'VISION STATUS | camera_publishers={self.count_publishers(self.camera_topic)} '
            f'frames={self._frames_received} processed={self._frames_processed} '
            f'camera_age={camera_age} odom_age={odom_age} '
            f'boxes={self._last_boxes_count} detection={self._last_detection} '
            f'stop_busy={self.patrol_events.busy} stopped={self.patrol_events.stopped} '
            f'reported={self.reported} pending_reports={len(self._pending_futures)} '
            f'report_service_ready={self.submit_client.service_is_ready()}'
        )

    def _odom_callback(self, msg):
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        with self._odom_lock:
            self._odom_motion = (time.monotonic(), math.hypot(v.x, v.y, v.z),
                                 math.hypot(w.x, w.y, w.z))

    def amcl_callback(self, msg):
        if self._last_odom_at is None:
            self.get_logger().info(f'POSE first message | topic={self.pose_topic} (map)')
        self._last_odom_at = time.monotonic()
        self.latest_pose = msg.pose.pose

    def get_current_pose_2d(self):
        """(x, y, yaw) 또는 아직 odom을 못 받았으면 None."""
        if self.latest_pose is None:
            return None
        x = self.latest_pose.position.x
        y = self.latest_pose.position.y
        yaw = self._yaw_from_quaternion(self.latest_pose.orientation)
        return (x, y, yaw)

    def get_current_position(self):
        pose = self.get_current_pose_2d()
        if pose is None:
            return None
        return (pose[0], pose[1])

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _bearing_offset(self, bbox, frame_width):
        """Return bbox 중심의 로봇 기준 방향(rad, 반시계 +). 화면 오른쪽 대상은 음수."""
        return bearing_from_bbox(bbox, frame_width, self.hfov)

    def compute_theta(self, bbox, frame_width):
        """지금 로봇 pose + bbox 위치로, 대상을 향한 절대 방향(rad)을 계산. pose 없으면 None."""
        pose = self.get_current_pose_2d()
        if pose is None:
            return None
        _, _, yaw = pose
        return yaw + self._bearing_offset(bbox, frame_width)

    def image_callback(self, image_msg):
        now = time.monotonic()
        self._frames_received += 1
        self._last_image_at = now
        if self._frames_received == 1:
            self.get_logger().info(
                f'CAMERA first frame | topic={self.camera_topic} bytes={len(image_msg.data)}'
            )

        try:
            frame = cv2.imdecode(
                np.frombuffer(image_msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                self.get_logger().warning(
                    f'CAMERA decode failed | bytes={len(image_msg.data)}',
                    throttle_duration_sec=STATUS_LOG_PERIOD_SEC,
                )
                return

            if self._frames_received == 1:
                self.get_logger().info(f'YOLO first inference starting | shape={frame.shape}')
            result = self.model.predict(
                source=frame, conf=YOLO_CONFIDENCE, imgsz=YOLO_IMAGE_SIZE, verbose=False
            )[0]
            self._frames_processed += 1
            self._last_boxes_count = 0 if result.boxes is None else len(result.boxes)

            detection = self.select_detection(result)
            self._last_detection = (
                'none' if detection is None else f'{detection[0]}:{detection[1]:.2f}'
            )
            frame_width = frame.shape[1]
            with self._latest_lock:
                self._latest = (now, detection, frame)
                self._latest_frame_at = now

            if detection is None:
                if self.last_seen is not None and now - self.last_seen > MAX_FRAME_GAP_SEC:
                    self.reset()
            else:
                class_name, confidence, bbox = detection
                self.last_seen = now

                if self.reported and class_name == self.class_name:
                    pass
                else:
                    if self.class_name != class_name:
                        # 새로 잡힌 후보 -> hold 타이머 시작 전에 먼저 중복부터 확인
                        if self.is_known_event(class_name, bbox, frame_width):
                            # 같은 로봇이 이미 보고한 사건과 같음(레이 교차로 확인) -> 후보로 삼지 않고 무시
                            self.get_logger().info(
                                f'DETECTION skipped: known event | class={class_name} bbox={bbox}',
                                throttle_duration_sec=STATUS_LOG_PERIOD_SEC,
                            )
                            return
                        self.class_name = class_name
                        self.hold_start = now
                        self.reported = False
                        if not self._goal_active:
                            # 정렬 대상: 정지를 요청한 후보의 종류
                            self._candidate = class_name
                        self.get_logger().info(
                            f'DETECTION candidate | class={class_name} '
                            f'confidence={confidence:.2f} '
                            f'bbox={bbox}; requesting patrol stop'
                        )
                    self.confidence = confidence
                    self.bbox = bbox

                    if not self.reported and not self._goal_active:
                        # 첫 유효 bbox에서 즉시 정지 요청. 정렬 · 검증 · 보고는
                        # mission_supervisor가 보내는 DetectEvent Goal(execute_detect)에서 한다.
                        self.patrol_events.request()

        except Exception as error:
            self.get_logger().error(f'Vision error: {error}')

    def select_detection(self, result):
        best = None

        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = str(self.model.names[class_id]).strip().lower()

                if class_name not in EVENT_TYPE_NAMES:
                    continue

                confidence = float(box.conf[0])
                if best is None or confidence > best[1]:
                    bbox = tuple(int(v) for v in box.xyxy[0].tolist())
                    best = (class_name, confidence, bbox)

        return best

    def reset(self):
        if self.class_name is not None:
            self.get_logger().info(
                f'DETECTION reset: frame gap exceeded | class={self.class_name} '
                f'stop_busy={self.patrol_events.busy} stopped={self.patrol_events.stopped}'
            )
        self.class_name = None
        self.bbox = None
        self.confidence = 0.0
        self.hold_start = None
        self.last_seen = None
        self.reported = False

    # ------------------------------------------------------------------
    # 중복 식별 — 레이(위치+방향) 교차 기반. CR-비전_09-10_12-01 3절 참고.
    # ------------------------------------------------------------------
    def is_known_event(self, class_name, bbox, frame_width):
        pose = self.get_current_pose_2d()
        if pose is None:
            return False  # 위치를 모르면 판단 불가 -> 중복 아님으로 취급하고 정상 진행

        x, y, yaw = pose
        theta = yaw + self._bearing_offset(bbox, frame_width)

        now = time.monotonic()
        self._dismissed = [d for d in self._dismissed if d['until'] > now]
        for d in self._dismissed:
            # 확정 실패 직후: 같은 자리(0.5 m 안)에서 같은 종류는 방향과 관계없이 잠시 무시
            near = math.hypot(x - d['x'], y - d['y']) <= NEAR_ORIGIN_THRESHOLD_M
            if d['class_name'] == class_name and near:
                return True

        for event in self._reported_events:
            if event['class_name'] != class_name:
                continue

            ex, ey, etheta = event['x'], event['y'], event['theta']
            origin_dist = math.hypot(x - ex, y - ey)

            if origin_dist <= NEAR_ORIGIN_THRESHOLD_M:
                # 거의 같은 자리에서 본 것 -> 각도까지 비슷해야 같은 사건
                if self._angle_diff(theta, etheta) <= BEARING_NEAR_TOLERANCE_RAD:
                    return True
                continue  # 같은 자리인데 각도가 다르면 다른 사건(같은 지점, 다른 방향)

            # 원점이 떨어져 있으면(다른 웨이포인트 등) 실제로 레이를 교차시켜 확인
            if self._intersect_rays(x, y, theta, ex, ey, etheta) is not None:
                return True

        return False

    def record_reported_event(self, class_name, position, theta):
        if position is None or theta is None:
            return

        x, y = position
        self._reported_events.append({'class_name': class_name, 'x': x, 'y': y, 'theta': theta})
        if len(self._reported_events) > self._MAX_REPORTED_EVENTS:
            self._reported_events = self._reported_events[-self._MAX_REPORTED_EVENTS:]

    @staticmethod
    def _angle_diff(a, b):
        d = (a - b + math.pi) % (2 * math.pi) - math.pi
        return abs(d)

    @staticmethod
    def _intersect_rays(x1, y1, th1, x2, y2, th2):
        """
        Return the point where two rays really cross, or None.

        둘 다 '앞쪽'이고 탐지 가능 거리 안이면 좌표를 반환하고,
        아니면(뒤쪽/너무 멀리/거의 평행) None을 반환한다.
        """
        angle_between = DetectingNode._angle_diff(th1, th2)
        if (angle_between < MIN_RAY_ANGLE_DIFF_RAD
                or angle_between > (math.pi - MIN_RAY_ANGLE_DIFF_RAD)):
            return None  # 거의 평행 -> 삼각측량 신뢰 불가

        d1x, d1y = math.cos(th1), math.sin(th1)
        d2x, d2y = math.cos(th2), math.sin(th2)

        cross = d1x * d2y - d1y * d2x
        if abs(cross) < 1e-6:
            return None

        dx = x2 - x1
        dy = y2 - y1
        t1 = (dx * d2y - dy * d2x) / cross
        t2 = (dx * d1y - dy * d1x) / cross

        if t1 <= 0 or t2 <= 0:
            return None  # 둘 중 하나라도 로봇 '뒤쪽'이면 무효
        if t1 > MAX_SENSOR_RANGE_M or t2 > MAX_SENSOR_RANGE_M:
            return None  # 비현실적으로 먼 교차점

        return (x1 + t1 * d1x, y1 + t1 * d1y)

    def submit_event(self, frame, event_id=None):
        """
        Build the ReportDetection request and send it on a worker thread.

        scenarios.md 3장: 응답이 없을 때만 같은 event_id와 같은 내용으로 재시도한다.
        저장 결과(STORED · DUPLICATE · REJECTED)를 받으면, 또는 재시도가 끝나면
        patrol_resume(true)을 보내 mission_supervisor가 순찰을 이어 가게 한다.
        """
        frame_width = frame.shape[1]
        evidence_frame = frame.copy()
        x1, y1, x2, y2 = self.bbox
        cv2.rectangle(evidence_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            evidence_frame,
            f'{self.class_name} {self.confidence:.2f}',
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        encoded = encode_evidence(evidence_frame)
        if encoded is None:
            self.get_logger().error('SUBMIT skipped | evidence JPEG unavailable; resuming patrol')
            self.patrol_events.resume()
            return

        event_id = event_id or str(uuid.uuid4())
        class_name = self.class_name
        # 보고 시점(=지금)의 위치·방향을 스냅샷으로 떠서 그대로 쓴다(응답을 기다리는
        # 동안 로봇이 움직였을 수 있어 나중에 다시 조회하면 엉뚱한 값이 될 수 있음).
        position = self.get_current_position()
        theta = self.compute_theta(self.bbox, frame_width)

        request = ReportDetection.Request()
        request.robot_id = self.robot_id
        request.event_id = event_id
        request.detected_at = self.get_clock().now().to_msg()
        if position is not None:
            request.position.x, request.position.y = position
        request.image = list(encoded)
        request.event_type = self.event_type_map[class_name]

        self.get_logger().info(
            f'SUBMIT request | event_id={event_id} class={class_name} '
            f'image_bytes={len(encoded)} position={position}'
        )
        worker = threading.Thread(
            target=self._report_worker,
            args=(request, class_name, position, theta),
            daemon=True,
        )
        worker.start()

    def _report_worker(self, request, class_name, position, theta):
        event_id = request.event_id
        for attempt in range(1, REPORT_ATTEMPTS + 1):
            if not self.submit_client.wait_for_service(timeout_sec=REPORT_TIMEOUT_SEC):
                self.get_logger().warning(
                    f'SUBMIT attempt {attempt}/{REPORT_ATTEMPTS} | service not ready: '
                    f'{SUBMIT_SERVICE}'
                )
                continue
            future = self.submit_client.call_async(request)
            self._pending_futures.add(future)
            deadline = time.monotonic() + REPORT_TIMEOUT_SEC
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            self._pending_futures.discard(future)
            if not future.done():
                future.cancel()
                self.get_logger().warning(
                    f'SUBMIT attempt {attempt}/{REPORT_ATTEMPTS} | no response; '
                    f'retrying with the same event_id={event_id}'
                )
                continue
            try:
                response = future.result()
            except Exception as error:
                self.get_logger().error(
                    f'SUBMIT attempt {attempt}/{REPORT_ATTEMPTS} | event_id={event_id} '
                    f'error={error}'
                )
                continue
            self.get_logger().info(
                f'SUBMIT 결과 | event_id={event_id} status={response.status} '
                f'detail={response.detail!r}'
            )
            if response.status in (ReportDetection.Response.STORED,
                                   ReportDetection.Response.DUPLICATE):
                self.record_reported_event(class_name, position, theta)
            else:
                saved = self.save_unstored_report(request, f'rejected: {response.detail}')
                self.get_logger().error(
                    f'ALERT SUBMIT rejected | event_id={event_id} detail={response.detail!r}; '
                    f'evidence kept at {saved}; resuming patrol'
                )
            self.patrol_events.resume()
            return
        saved = self.save_unstored_report(request, f'no response after {REPORT_ATTEMPTS} attempts')
        self.get_logger().error(
            f'ALERT SUBMIT failed after {REPORT_ATTEMPTS} attempts | event_id={event_id}; '
            f'evidence kept at {saved}; resuming patrol'
        )
        self.patrol_events.resume()

    def save_unstored_report(self, request, reason):
        """Keep a report the system monitor did not store (image + fields) on this PC."""
        try:
            UNSTORED_REPORT_DIR.mkdir(parents=True, exist_ok=True)
            base = UNSTORED_REPORT_DIR / request.event_id
            base.with_suffix('.jpg').write_bytes(bytes(request.image))
            base.with_suffix('.json').write_text(json.dumps({
                'robot_id': request.robot_id,
                'event_id': request.event_id,
                'event_type': int(request.event_type),
                'detected_at': {'sec': request.detected_at.sec,
                                'nanosec': request.detected_at.nanosec},
                'position': [request.position.x, request.position.y],
                'reason': reason,
            }, ensure_ascii=False, indent=2))
            return str(base.with_suffix('.jpg'))
        except OSError as error:
            self.get_logger().error(f'unstored report could not be saved: {error}')
            return 'not saved'

    # ------------------------------------------------------------------
    # DetectEvent (설계도 03 페이지): mission_supervisor → 감지·정렬 노드, 각 AMR PC 내부.
    # ALIGNING (CR 9/9): 정지 확인 → 화면 중심 오차(horizontal_error)로 cmd_vel_yaw 제자리
    #   회전 → |오차| <= 0.05 가 0.5초 유지 → yaw 0 → 정지 재확인.
    #   후보가 0.6초 넘게 끊기거나 10초가 지나면 확정 실패.
    # VERIFYING: min_confidence 이상이 HOLD_DURATION_SEC(1초) 동안 끊김 없이 이어지면 확정.
    # 확정이면 detection_id로 ReportDetection 보고 → 결과 수신 후 patrol_resume(true).
    # ------------------------------------------------------------------
    def _latest_detection(self, newer_than=None):
        """Return (t, detection, frame) of the newest processed frame, or None."""
        with self._latest_lock:
            latest = self._latest
        if latest is None:
            return None
        if newer_than is not None and latest[0] <= newer_than:
            return None
        return latest

    def _wait_frame(self, goal_handle, newer_than, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return None
            latest = self._latest_detection(newer_than)
            if latest is not None:
                return latest
            time.sleep(0.03)
        return None

    def _publish_yaw(self, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = float(angular_z)
        self.yaw_pub.publish(msg)

    def _is_stopped(self):
        with self._odom_lock:
            odom = self._odom_motion
        if odom is None:
            return False
        rx, linear, angular = odom
        return (time.monotonic() - rx <= STOP_ODOM_MAX_AGE_SEC
                and linear <= STOP_LINEAR_MAX and angular <= STOP_ANGULAR_MAX)

    def _wait_stopped(self, goal_handle, timeout=STOP_CONFIRM_TIMEOUT_SEC):
        """Return True when 선속도·각속도 기준이 STOP_HOLD_SEC 연속 유지(odometry 실제 정지)."""
        deadline = time.monotonic() + timeout
        since = None
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self._is_stopped():
                since = since or time.monotonic()
                if time.monotonic() - since >= STOP_HOLD_SEC:
                    return True
            else:
                since = None
            time.sleep(0.05)
        return False

    def _detect_feedback(self, goal_handle, state, elapsed=0.0, current=0.0, average=0.0):
        fb = DetectEvent.Feedback()
        fb.state = state
        fb.confirm_elapsed = float(elapsed)
        fb.current_confidence = float(current)
        fb.average_confidence = float(average)
        goal_handle.publish_feedback(fb)

    def _finish_detect(self, goal_handle, confirmed, average=0.0, event_type=0):
        self._publish_yaw(0.0)
        result = DetectEvent.Result()
        result.confirmed = bool(confirmed)
        result.robot_id = self.robot_id
        result.event_type = int(event_type)
        result.average_confidence = float(average)
        result.event_time = self.get_clock().now().to_msg()
        if self.latest_pose is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = result.event_time
            pose.pose = self.latest_pose
            result.current_pose = pose
        canceled = goal_handle.is_cancel_requested
        if canceled:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        if not confirmed:
            # 확정이 아니면 mission_supervisor가 스스로 재개한다. 정렬 · 검증까지 했는데도
            # 확정되지 않은 자리는 같은 대상을 바로 다시 잡아 정지를 반복하지 않게 잠시 무시한다.
            # EStop · 대피 · 순찰 취소로 중단된 경우는 판단을 못 한 것이므로 무시하지 않는다.
            pose = self.get_current_pose_2d()
            if not canceled and pose is not None and self._candidate is not None:
                self._dismissed.append({
                    'class_name': self._candidate, 'x': pose[0], 'y': pose[1],
                    'until': time.monotonic() + DISMISS_COOLDOWN_SEC})
            self.patrol_events.release()
            self.reset()
        self._candidate = None
        self._goal_active = False
        return result

    def _align(self, goal_handle, matches):
        """CR 9/9 yaw 정렬. 'aligned' 또는 실패 사유 문자열을 돌려준다."""
        if not self._wait_stopped(goal_handle):
            return 'robot not stopped before alignment'
        start = time.monotonic()
        latest = self._latest_detection()
        last_valid = start
        error = None
        if latest is not None and matches(latest[1]):
            _, (_, _, bbox), frame = latest
            error = horizontal_error(bbox, frame.shape[1])
        seen_t = latest[0] if latest is not None else None
        in_tolerance_since = None
        while True:
            now = time.monotonic()
            if goal_handle.is_cancel_requested:
                return 'cancelled'
            if now - start > ALIGN_TIMEOUT_SEC:
                return f'timeout {ALIGN_TIMEOUT_SEC:.0f}s'
            latest = self._latest_detection(seen_t)
            if latest is not None:
                seen_t = latest[0]
                if matches(latest[1]):
                    _, (_, _, bbox), frame = latest
                    error = horizontal_error(bbox, frame.shape[1])
                    last_valid = latest[0]
            if now - last_valid > CANDIDATE_GAP_SEC:
                return 'candidate lost'
            if error is None:
                # 아직 유효한 후보 프레임이 없다: 회전하지 않고 0.6초까지 기다린다.
                self._publish_yaw(0.0)
                time.sleep(ALIGN_LOOP_PERIOD_SEC)
                continue
            if abs(error) <= ALIGN_ERROR_TOLERANCE:
                self._publish_yaw(0.0)
                in_tolerance_since = in_tolerance_since or now
                if now - in_tolerance_since >= ALIGN_HOLD_SEC:
                    break
            else:
                in_tolerance_since = None
                self._publish_yaw(yaw_rate_for_error(error))
            self._detect_feedback(goal_handle, DetectEvent.Feedback.ALIGNING)
            time.sleep(ALIGN_LOOP_PERIOD_SEC)
        self._publish_yaw(0.0)
        if not self._wait_stopped(goal_handle):
            return 'robot not stopped after alignment'
        return 'aligned'

    def execute_detect(self, goal_handle):
        goal = goal_handle.request
        self._goal_active = True
        min_conf = goal.min_confidence if goal.min_confidence > 0.0 else YOLO_CONFIDENCE
        target_class = self._candidate or self.class_name
        self.get_logger().info(
            f'DETECT goal | id={goal.detection_id} class={target_class} min_conf={min_conf:.2f}'
        )

        def matches(detection):
            return (detection is not None and detection[1] >= min_conf
                    and (target_class is None or detection[0] == target_class))

        # ALIGNING ------------------------------------------------------
        self._detect_feedback(goal_handle, DetectEvent.Feedback.ALIGNING)
        outcome = self._align(goal_handle, matches)
        if outcome != 'aligned':
            self.get_logger().info(f'ALIGN failed | {outcome}')
            return self._finish_detect(goal_handle, False)
        self.get_logger().info('ALIGN complete | robot stopped; verifying')

        # VERIFYING -----------------------------------------------------
        start = time.monotonic()
        hold_from = None
        last_t = None
        confs = []
        while time.monotonic() - start < VERIFY_TIMEOUT_SEC:
            if goal_handle.is_cancel_requested:
                return self._finish_detect(goal_handle, False)
            latest = self._wait_frame(goal_handle, last_t, MAX_FRAME_GAP_SEC * 3)
            if latest is None:
                hold_from, confs = None, []
                continue
            t, detection, frame = latest
            if last_t is not None and t - last_t > MAX_FRAME_GAP_SEC:
                hold_from, confs = None, []
            last_t = t
            if not matches(detection):
                hold_from, confs = None, []
                self._detect_feedback(goal_handle, DetectEvent.Feedback.VERIFYING)
                continue
            if hold_from is None:
                hold_from = t
            confs.append(detection[1])
            elapsed = t - hold_from
            average = sum(confs) / len(confs)
            self._detect_feedback(goal_handle, DetectEvent.Feedback.VERIFYING,
                                  elapsed, detection[1], average)
            if elapsed >= HOLD_DURATION_SEC:
                class_name, confidence, bbox = detection
                self.class_name, self.confidence, self.bbox = class_name, confidence, bbox
                self.get_logger().info(
                    f'DETECT confirmed | id={goal.detection_id} class={class_name} '
                    f'avg={average:.2f} frames={len(confs)}')
                self.submit_event(frame, event_id=goal.detection_id)
                self.reported = True
                return self._finish_detect(goal_handle, True, average=average,
                                           event_type=self.event_type_map[class_name])
        self.get_logger().info(f'DETECT not confirmed within {VERIFY_TIMEOUT_SEC:.1f}s')
        return self._finish_detect(goal_handle, False)


def main(args=None):
    rclpy.init(args=args)
    node = DetectingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.get_logger().info('VISION spin started')
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('VISION shutdown requested')
    finally:
        node.get_logger().info('VISION cleanup')
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
