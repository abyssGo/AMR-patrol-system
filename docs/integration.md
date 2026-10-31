# 시스템 통합과 시험

> 기준일: 2026-09-13 · 계약: `patrol_interfaces 2.1.0`

통합 순서와 합격 조건을 정한다. 각 팀의 단위시험 통과만으로 장비 통합이 끝났다고
보지 않는다.

## 1. 통합 기준

- 모든 PC는 같은 Git commit의 `patrol_interfaces`, `patrol_amr_interfaces`를 빌드한다.
- `robot_id`와 namespace는 `robot1`, `robot6`이다.
- 외부 순찰은 `Patrol` Action만 사용한다.
- 최종 속도는 각 AMR의 `local_safety_supervisor`만 발행한다.
- 운영자 명령은 PC 3 터미널에서 `/control/*` 서비스로 보낸다.

## 2. 기동 순서

1. PC 3 Offboard Discovery Server와 각 TB4 Onboard Discovery를 확인한다.
2. PC 1·2에서 각 로봇의 localization, Nav2, `amr.launch.py`를 띄운다.
3. PC 3에서 시스템 모니터를 띄운다(`/system_monitor/report_detection`).
4. PC 4에서 `gate_cam`, `center_cam`, `cam_master`를 띄운다.
5. PC 3에서 관제를 띄운다.
6. 토픽·Action·Service의 타입과 송수신 노드를 확인한다.
7. `/control/start_patrol`로 교대 순찰을 시작한다.

## 3. 필수 종단 흐름

### IT-01 순찰 시작

```text
start_patrol → patrol_allowed=true → Patrol Goal → 수락 → WAITING_FOR_TOKEN
→ DriveToken → token_valid=true → UNDOCKING → INITIAL_POSE_READY → PATROLLING
```

합격: `WAITING_FOR_TOKEN` 전에 AMR이 움직이지 않는다. 두 로봇이 동시에 유효한 Token을 갖지 않는다.

### IT-02 경유지, 도킹, 교대

```text
PATROLLING → WAYPOINT_REACHED 반복 → DOCKING → Result SUCCEEDED
→ 빈 DriveToken → 다음 로봇 Patrol Goal
```

합격: 각 로봇이 자기 도크(`robot1` (0, 0), `robot6` (0, −0.7))로 돌아와 도킹하고, 도킹 뒤 다른 로봇이 출발한다.

### IT-03 차량 대피와 재개

```text
patrol_allowed=false → MOVE_TO_SAFE_ZONE → MOVING_TO_SAFE_ZONE
→ patrol_allowed=true → RESUME_PATROL → RESUMING → PATROLLING
```

합격: 같은 permit이 반복돼도 명령이 중복되지 않는다. 대피 중 Token이 유지되고 새 Goal이 생기지 않는다.

### IT-04 감지와 저장

```text
후보 → patrol_stop → DETECTION_PROCESSING → DetectEvent(정렬·검증)
→ DETECTION_CONFIRMED + event_id → ReportDetection → STORED → patrol_resume → PATROLLING
```

합격: Feedback의 `event_id`와 저장된 사건의 `event_id`가 같다. 같은 요청을 다시 보내도 중복 저장되지 않는다. 정렬 회전이 `cmd_vel_yaw` → `local_safety_supervisor` → `cmd_vel`로 나간다.

### IT-05 화재 hold

`DETECTION_CONFIRMED + FIRE` 뒤 현재 임무는 끝까지 진행하고, 임무가 끝나면 다음 로봇이
출발하지 않는다. `/control/clear_fire_hold` 뒤에만 다음 순찰이 시작된다.

### IT-06 EStop

```text
set_estop(active=true) → 즉시 0 속도 → BLOCKED
→ set_estop(active=false) → 계속 정지 → resume → RESUME_PATROL → PATROLLING
```

합격: 해제 뒤 resume 전까지 움직이지 않는다. EStop이 걸린 로봇은 새 순찰에도 출발하지 않는다. `all`은 두 로봇 모두에 적용된다.

### IT-07 Token 끊김

관제를 멈추거나 Token 발행을 끊으면 lease(1초) 안에 0 속도가 되고 `BLOCKED`가 된다.
30초 안에 돌아오면 이어서 진행하고, 아니면 `FAILED(TOKEN_EXPIRED)`로 끝난다.

### IT-08 도킹 재시도와 제외

도킹을 일부러 실패시키면 최대 3번 시도한 뒤 도크 앞에 정차하고 `FAILED(DOCKING_FAILED)`를
돌려준다. 관제는 그 로봇을 순서에서 빼고 다른 로봇으로 순찰을 이어 간다.
`/control/start_patrol` 뒤 다시 순서에 들어간다.

### IT-09 배터리

배터리 15% 미만이면 귀환·도킹 후 `FAILED(LOW_BATTERY)`. 관제는 다음 로봇에 차례를 넘기고,
그 로봇은 80%가 될 때까지 출발시키지 않는다.

### IT-10 취소와 관제 재시작

- `/control/cancel_patrol`: 빈 Token이 Action 종료보다 먼저 나가고, `CANCELED`로 끝나며 교대가 멈춘다.
- 순찰 중 관제를 다시 띄우면 남아 있는 임무를 취소하고, 끝난 것이 확인될 때까지 출발시키지 않는다.
- 통신 복구만으로 새로 출발하지 않는다.

### IT-11 CCTV

- gate 토픽은 `ENTERING`, `EXITED`만, center 토픽은 `PARKED`, `EXITING`만 받는다.
- `cam_master`가 상태에 맞는 `patrol_allowed`를 5 Hz로 발행한다.
- CCTV가 AMR 제어 토픽을 직접 발행하지 않는다.

## 4. 빌드와 자동 시험

```bash
colcon build
python3 scripts/verify_interface_v1.py
bash scripts/check_tracked_paths.sh
(cd src/patrol_control && python3 -m pytest -q test)
(cd src/patrol_amr_safety && python3 -m pytest -q test)
(cd src/patrol_vision && python3 -m pytest -q test)
(cd src/patrol_sysmon && python3 -m pytest -q)
```

`verify_interface_v1.py`는 소스 트리·CMake 등록·필드·상수·manifest SHA-256을 확인한다.

## 5. 완료 판정

다음 단계를 구분해 기록한다.

- 단위시험 통과
- 같은 PC에서 ROS 노드 연결 확인
- 여러 PC 사이 DDS 송수신 확인
- 실제 AMR 주행
- 실제 CCTV와 감지 저장까지 포함한 종단시험

문서나 mock 시험만으로 실제 장비 통합 완료를 선언하지 않는다. 실패하면 생산자·소비자·
QoS·namespace·시간 순서를 함께 기록한다.

현재 상태: 자동 시험은 모두 통과한다. 실제 TurtleBot 4 두 대로 하는 종단 통합시험은
하지 않았다. 시연 영상에 나온 범위(순찰·도킹 복귀, 차량 대피·재개, 감지 정지·기록·재개, CCTV 판정)
밖의 기능(교대 순찰, EStop, 정렬 회전, 도킹 재시도, 배터리 귀환)은 자동 시험으로만 확인했다.
