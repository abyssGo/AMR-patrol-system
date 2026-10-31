# patrol_amr_safety — AMR 순찰 · 안전 · 감지 노드

AMR PC(PC 1 `robot1`, PC 2 `robot6`)에서 실행하는 노드다. 관제(PC 3)와는 공용 계약
`patrol_interfaces 2.1.0`으로, AMR 안의 감지 노드와는 내부 계약
`patrol_amr_interfaces/DetectEvent`로 연결한다. 설계는 [docs/amr.md](../../docs/amr.md)를 따른다.

> 교대 순찰, EStop, 정렬 회전, 도킹 재시도, 배터리 귀환은 자동 시험으로 확인했고, 전체 시스템과의 통합은 Gazebo로 검증했다([README 7절](../../README.md#7-프로젝트-완료-이후--gazebo-통합-검증)).

## 노드

| 노드 | 하는 일 |
|---|---|
| `mission_supervisor` | 관제의 `/{robot}/patrol_action`(`Patrol`) 서버. Goal 수락 → `WAITING_FOR_TOKEN` → 유효한 `DriveToken`을 받으면 언도킹 · 초기 위치 → WP1 ~ WP6 → WP3 → WP7 → WP1(각 360° 회전) → 자기 도크 앞 → 도킹(최대 3번). 진행은 Feedback, 끝은 Result |
| `local_safety_supervisor` | Nav2 후보(`cmd_vel_safety_in`)와 정렬 회전 후보(`cmd_vel_yaw`) 중 0.5초 안에 받은 하나만, `DriveToken`이 유효하고 EStop이 없을 때 `cmd_vel`로 넘긴다. 그 밖에는 0 속도 |
| `vision_node_v2` | OAK-D 영상을 YOLO로 보고 후보가 보이면 `patrol_stop`으로 정지 요청 → `DetectEvent`를 받아 **정렬(제자리 회전으로 대상을 화면 가운데로) → 검증(신뢰도 0.70 이상 1초)** → 확정이면 `ReportDetection`(event_id = detection_id) → 저장 결과를 받으면 `patrol_resume` |

### 실행 중 처리 (`mission_supervisor`)

| 입력 | 동작 | Feedback |
|---|---|---|
| EStop 활성 | 즉시 정지, 진행 중인 동작 취소. 해제 + Token + 관제의 재개 명령이 있어야 다시 움직임 | `BLOCKED` |
| `PatrolCommand` 1 (`MOVE_TO_SAFE_ZONE`) | 정지 → 가장 가까운 안전구역 이동 → 대기 | `MOVING_TO_SAFE_ZONE` |
| `PatrolCommand` 2 (`RESUME_PATROL`) | 멈춘 단계부터 재개 | `RESUMING` → `PATROLLING` |
| `DriveToken` 회수 · 만료 | 정지 후 대기, 돌아오면 재개 / 30초 안에 안 돌아오면 `FAILED(TOKEN_EXPIRED)` | `BLOCKED` |
| 감지 노드 `patrol_stop` | 정지 → `DetectEvent` → 확정이면 보고, 저장 뒤 `patrol_resume`으로 재개 / 아니면 바로 재개 | `DETECTION_PROCESSING` → `DETECTION_CONFIRMED` |
| 경유지 이동 실패 | 1번 더 → 건너뜀, 연속 2개 건너뛰면 귀환 | — |
| 배터리 15% 미만 | 귀환 · 도킹 후 `FAILED(LOW_BATTERY)` | `DOCKING` |
| 도킹 3번 실패 | 자기 도크 앞 정차, `FAILED(DOCKING_FAILED)` | — |
| Goal 취소 | 정지 후 `CANCELED(CONTROL_CANCELED)` | — |

| 로봇 | 도크 앞 | 언도킹 직후 초기 위치 |
|---|---|---|
| `robot1` | (0, 0) | (0, 0), 남쪽 |
| `robot6` | (0, −2.076) | (0, −2.076), 남쪽 |

좌표(경로 · 도크 · 안전구역)는 [`config/site.yaml`](config/site.yaml)에서 읽는다. 현장이 바뀌면 이 파일만 고치면 된다.

## 실행

Nav2 · AMCL은 `turtlebot4_navigation`으로 따로 띄운다. 이 저장소의 `turtlebot4_navigation` 설정은
Nav2 `collision_monitor`와 `docking_server` 출력을 `cmd_vel_safety_in`으로 보내므로,
**`local_safety_supervisor` 없이는 Nav2 주행이 바퀴에 전달되지 않는다.**

```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=robot1 map:=<지도 yaml>
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=robot1
ros2 launch patrol_amr_safety amr.launch.py robot_id:=robot1
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `robot_id` | `robot1` | `robot1` · `robot6` (노드 namespace) |
| `use_sim_time` | `false` | 시뮬레이션 시계 사용 |

- YOLO 모델: `~/patrol/detection_best/detection_best.pt`
- YOLO(`ultralytics`)는 pip로 설치한다.
- 카메라는 `/{robot}/oakd/rgb/image_raw/compressed`(압축 영상 발행을 켜야 함)를 쓴다.

## 시험

```bash
colcon build --packages-up-to patrol_amr_safety && source install/setup.bash
cd src/patrol_amr_safety && python3 -m pytest -q test
```

| 시험 | 내용 |
|---|---|
| `test_mission_flow.py` | 가짜 Nav2 · Create 3 · 시계로 순찰 흐름 전체(EStop, Token 끊김, 안전구역, 감지, 경유지 재시도, 배터리, 도킹 재시도) |
| `test_safety_rules.py` | EStop · 도킹 판정, 후보 선택 |
| `test_drive_token_check.py` | Token 유효 · 만료 · 회수 · 다른 로봇 · 순서 역전 |
| `test_alignment.py` | 화면 중심 오차와 회전 속도 |
| `test_bearing.py` | bbox 중심의 방향각 |
| `test_stop_confirm.py` | odometry 정지 확인 |
| `test_vision_event_link.py` | 정지 요청 · 재개 · 보고 재시도, 큰 사진 축소, 저장 안 된 보고 보관 |
| `test_site_config.py` | 현장 좌표 파일(`config/site.yaml`) 값과 검사 |
| `test_flake8.py` · `test_pep257.py` | 코드 스타일 검사 |
