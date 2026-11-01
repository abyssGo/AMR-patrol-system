# 🙋 내 역할 — 박진용 (기똥차)

**CCTV 차량 판정 · AMR 이상 감지 · 비전 모델 학습 담당**

입구(Gate)와 주차 구역(Center) 웹캠으로 차량 상태를 판정해 순찰 허용 여부(`patrol_allowed`)로 바꾸는 CCTV 비전 노드와, TurtleBot 4의 OAK-D 영상에서 화재 · 누수 · 적치물을 찾아 정렬 · 확인한 뒤 시스템 모니터에 저장하는 AMR 감지 노드를 만들었습니다. 두 노드가 쓰는 차량 · 이상 감지 모델을 학습하고 선정했고, 프로젝트를 마친 뒤에는 최종 코드를 정리하고 Gazebo에서 전체 시스템을 연결해 다시 돌려 보았습니다.

**다룬 기술:** ROS 2 Jazzy(Topic · Service · Action · QoS) · Ultralytics YOLO11n · OpenCV · OAK-D · Python · pytest · Gazebo Harmonic

**담당 코드:** `src/patrol_vision/`(`gate_cam.py` · `center_cam.py` · `cam_master.py` · `cam_common.py`) · `src/patrol_amr_safety/patrol_amr_safety/vision_node_v2.py` · `src/patrol_amr_interfaces/` · `data/`(모델 선정 보고서)

---

## 1. CCTV 차량 상태 판정 — `gate_cam` · `center_cam`

시연장에는 차가 RC 카 1대뿐이라, 매 프레임 판정 영역 안에서 신뢰도가 가장 높은 박스 하나만 차로 봤습니다. YOLO 트래킹 ID는 쓰지 않았습니다. ID가 중간에 바뀌거나 끊기는 문제를 판정에 끌어들이고 싶지 않았습니다.

Gate는 화면 폭 32%와 70% 지점에 판정선을 하나씩 두고, 박스 중심이 어떤 순서로 두 선을 넘는지로 방향을 정합니다. 왼쪽 선 → 오른쪽 선이면 입차(`ENTERING`), 반대면 출차 완료(`EXITED`)입니다. 선을 하나만 두면 차가 선 근처에서 머뭇거릴 때 들어오는 건지 나가는 건지 알 수 없어서 두 개로 나눴습니다.

Center는 주차 칸 ROI 두 곳을 보고, 박스 중심이 5초 머물면 주차(`PARKED`), ROI를 벗어나면 출차 중(`EXITING`)으로 판정합니다.

잘못된 이벤트가 나가지 않도록 몇 가지를 더 걸었습니다. 이동 상태는 0.2초 동안 같은 방향에 있어야 확정하고, 그 사이 차를 놓치면 처음부터 다시 봅니다. 이벤트를 하나 보내면 2초 동안은 새 판정을 시작하지 않습니다. 그리고 이벤트마다 `event_id`(세션 ID + 상태 + 순번)를 붙여서, 받는 쪽이 같은 이벤트를 두 번 반영하지 않게 했습니다.

```python
# 한 프레임에 두 선을 넘으면(검출이 잠깐 끊긴 뒤 등) 이동 방향 순서로 기록한다.
crossed_now = {'L': crossed(prev_cx, cx, left_x), 'R': crossed(prev_cx, cx, right_x)}
for mark in (('L', 'R') if cx > prev_cx else ('R', 'L')):
    if crossed_now[mark] and mark not in self._crossing_history:
        self._crossing_history.append(mark)
...
if seq[-2:] == ['L', 'R']:      # 왼쪽 → 오른쪽 = 입차
    self._pending = {'state': 'ENTERING', 'side': 'right', ...}
elif seq[-2:] == ['R', 'L']:    # 오른쪽 → 왼쪽 = 출차 완료
    self._pending = {'state': 'EXITED', 'side': 'left', ...}
```

## 2. 순찰 허용 변환 — `cam_master`

`cam_master`는 두 카메라의 이벤트를 받아 값 하나로 바꿉니다. 입차나 출차 중이면 `false`, 주차 완료나 출차 완료면 `true`입니다. 관제는 이 값이 `false`로 바뀌면 로봇을 안전구역으로 보내고, `true`로 바뀌면 순찰을 다시 시킵니다.

이벤트를 반영하기 전에 다섯 가지를 확인합니다. 토픽과 카메라가 맞는지(gate_event에는 gate_cam만), 그 카메라가 낼 수 있는 상태인지, 신뢰도가 0 ~ 1 안인지, 이미 받은 `event_id`인지, 이미 반영한 이벤트보다 과거 시각인지입니다. 하나라도 걸리면 버립니다. 마지막 검사는 늦게 도착한 옛 출차 이벤트가 이미 반영된 주차 상태를 되돌리는 걸 막으려고 넣었습니다.

값이 바뀌면 바로 보내고, 바뀌지 않아도 5 Hz로 마지막 값을 계속 보냅니다. 이벤트가 한동안 안 들어와도 값을 임의로 바꾸지는 않습니다.

```python
if msg.camera_id != expected_camera_id: return          # 토픽-카메라 불일치
if msg.state not in allowed_states: return              # 이 카메라가 낼 수 없는 상태
if not (0.0 <= msg.confidence <= 1.0): return           # 신뢰도 범위 이상
if msg.event_id in self._seen_event_ids: return         # 중복
if event_stamp < self._last_accepted_event_stamp: return # 순서 역전
```

## 3. AMR 이상 감지 노드 — `vision_node_v2.py`

순찰 중 첫 후보(신뢰도 0.70 이상)가 보이면 감지 노드는 일단 정지부터 요청합니다(`patrol_stop`). 정렬과 확인, 저장은 그다음 순찰 노드가 보내는 `DetectEvent` Action 안에서 합니다. 이렇게 나눠 두면 비상정지나 대피가 걸렸을 때 순찰 노드가 Action만 취소하면 됩니다.

정렬은 박스 중심의 화면 가로 오차가 ±0.05 안에서 0.5초 유지될 때까지 제자리 회전하는 방식입니다. 회전 속도는 오차에 비례하게 주되 0.08 ~ 0.25 rad/s 사이로 묶었고, 명령은 바퀴로 바로 보내지 않고 최종 속도 노드의 후보 토픽(`cmd_vel_yaw`)으로만 냅니다. 정렬이 끝나면 1초 동안 같은 종류가 0.70 이상으로 계속 보이는지 보고 확정합니다.

같은 사건을 두 번 보고하지 않는 것도 신경 썼습니다. 뎁스 카메라는 연산량과 지연이 커서 쓰지 않았고, 대신 로봇 위치와 방향, 박스의 화면 위치로 "시선"을 만듭니다. 새 후보의 시선이 이미 보고한 사건의 시선과 실제로 교차하면 같은 사건으로 보고 넘어갑니다.

저장 보고는 증거 사진을 1 MiB 안에 들어올 때까지 JPEG 품질과 해상도를 낮춰서 보냅니다. 크기 때문에 거절당하는 일을 없애려는 것입니다. 응답이 없을 때만 같은 `event_id`로 5초 간격, 최대 3번 다시 보내고, 끝내 저장되지 않으면 사진과 내용을 로봇 PC에 따로 남깁니다.

```python
def _intersect_rays(x1, y1, th1, x2, y2, th2):
    angle_between = _angle_diff(th1, th2)
    if angle_between < MIN_RAY_ANGLE_DIFF_RAD or angle_between > math.pi - MIN_RAY_ANGLE_DIFF_RAD:
        return None                      # 거의 평행 -> 교차점을 믿을 수 없음
    ...
    if t1 <= 0 or t2 <= 0: return None   # 로봇 뒤쪽
    if t1 > MAX_SENSOR_RANGE_M or t2 > MAX_SENSOR_RANGE_M: return None
    return (x1 + t1 * d1x, y1 + t1 * d1y)
```

## 4. 비전 모델 학습 · 선정

차량 검출 모델은 yolov8n, yolo11n, yolo26n을 같은 데이터 분할(70 / 20 / 10)과 같은 학습 조건으로 비교했습니다. 차를 놓치면 판정선 통과와 ROI 체류가 그대로 끊기기 때문에 Recall을 가장 먼저 봤고, 그다음 mAP50-95, Precision, 추론 시간 순으로 비교해 yolo11n을 골랐습니다(Test P 0.987 · R 0.988 · mAP50-95 0.696). 시연에 실제로 쓴 `car_best_v2`는 이 모델에서 시작해 시연장 Center 카메라로 찍은 RC 카 이미지를 더해 다시 학습한 것입니다(아래 트러블슈팅 ①).

이상 감지 모델은 yolov8, yolo11, yolo26의 n과 s, 모두 6종을 비교했습니다. 화재나 누수를 놓치면 정렬과 확인이 아예 시작되지 않으니 화재 · 누수 Recall을 먼저 보고 yolo11n을 골랐습니다(Test P 0.967 · R 0.997 · mAP50-95 0.700). 두 모델 모두 Validation으로 고르고, Test는 고른 뒤 최종 확인에만 썼습니다.

## 5. 설계도 · 인터페이스

CCTV 노드와 AMR 감지 노드 설계도를 draw.io로 그렸습니다([`docs/design/cctv_design.drawio`](docs/design/cctv_design.drawio) · [`docs/design/amr_vision_node.drawio`](docs/design/amr_vision_node.drawio)). 인터페이스 쪽에서는 `CameraState`에 `source_session_id`와 `source_sequence`를 더해 이벤트마다 출처와 순번이 붙게 했고, AMR 내부 `DetectEvent` Action(`patrol_amr_interfaces`)을 정의했습니다. 감지 후보와 정렬 상태 인터페이스는 변경 요청서(CR)를 써서 AMR 팀과 맞췄습니다.

## 6. 프로젝트 완료 이후 — 최종 정리 · Gazebo 통합 검증

프로젝트가 끝난 뒤 팀 코드를 최종본으로 정리하고, 테스트(CCTV 4 · 관제 49 · AMR 92 · 시스템 모니터 129개)가 모두 통과하도록 맞췄습니다. 그리고 Gazebo의 TurtleBot 4로 관제, CCTV, 시스템 모니터, AMR 노드를 저장소 코드 그대로 연결해 차량 대응과 비상정지, 감지 저장까지 한 번에 돌려 보았습니다([README 7절](README.md#7-프로젝트-완료-이후--gazebo-통합-검증)).

---

## 7. 트러블슈팅

### ① Center 아래쪽 주차 칸에서 차를 못 잡음 — 시연장 이미지로 추가 학습

Center 카메라를 시험하는데, RC 카가 아래쪽 주차 ROI에 들어가면 YOLO가 차를 안정적으로 잡지 못했습니다. 차가 분명히 주차돼 있는데도 `PARKED`가 나오지 않았습니다. 처음엔 ROI 판정 로직을 의심했지만, 들여다보니 그 전 단계인 차량 검출이 먼저 문제였습니다. 기존 학습 데이터에는 시연장 카메라에서 보이는 차의 위치, 거리, 각도가 충분하지 않았습니다.

그래서 실제 Center 카메라로 찍은 RC 카 이미지 약 250장을 기존 데이터에 더해, `car_best.pt`에서 시작해 다시 학습했습니다(epochs 50, lr0 0.0001). 같은 Test 세트에서 Recall이 0.944에서 1.000으로, mAP50-95가 0.712에서 0.737로 올랐고, 이 모델(`car_best_v2`)을 Gate와 Center에 적용했습니다.

### ② 감지 후보를 토픽으로 주고받다 정렬 · 확인 · 취소 순서가 꼬임 — Action 하나로 묶음

처음에는 감지 노드가 `DetectionCandidate`(후보와 화면 오차)를 보내고, AMR이 정렬을 마친 뒤 `AlignmentStatus`로 알려 주면 그때 감지 노드가 1초 확인을 시작하는 구조로 CR을 냈습니다. 그런데 이렇게 하면 비전과 AMR이 서로의 상태를 따로 들고 있어야 해서, 비상정지나 대피로 중간에 멈출 때 어느 쪽이 무엇을 취소해야 하는지 정리가 잘 안 됐습니다.

결국 순찰 노드가 보내는 `DetectEvent` Action 하나 안에서 정렬, 확인, 저장을 모두 하도록 바꿨습니다. 순찰 노드는 Action을 취소하기만 하면 되고, 감지 노드는 정렬 회전도 최종 속도 노드의 후보 토픽으로만 보내서 바퀴를 직접 움직이지 않습니다.
