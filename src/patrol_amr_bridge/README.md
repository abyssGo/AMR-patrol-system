# patrol_amr_bridge — 관제 ↔ AMR 중계

> 프로젝트 종료 후 추가한 패키지입니다. 빌드 · 정적 검사 · 모의 시험까지 검증했고, TurtleBot 4 실기에서는 확인하지 않았습니다.

관제(`patrol_control`)는 `patrol_interfaces/Patrol` Action · `DriveToken` · `PatrolCommand`로 순찰을 지시하지만, 최종 AMR 순찰(`genius_patrol`)은 같은 이름의 서버를 `nav2_msgs/NavigateToPose` 타입으로 열고 정지 · 재개는 `event_check`로 받습니다. 이 노드는 기존 코드를 고치지 않고 둘 사이를 잇습니다.

## 연결

| 방향 | 이름 | 타입 | 비고 |
|---|---|---|---|
| 관제 → 중계 | `/{robot}/patrol_action` | `patrol_interfaces/action/Patrol` | 중계가 서버 |
| 관제 → 중계 | `/{robot}/drive_token` | `DriveToken` | BEST_EFFORT · depth 3 (관제와 같음) |
| 관제 → 중계 | `/{robot}/patrol_command` | `PatrolCommand` | RELIABLE · depth 10 |
| 중계 → AMR | `/{robot}/patrol_route` | `nav2_msgs/action/NavigateToPose` | `genius_patrol`을 `-r patrol_action:=patrol_route`로 실행 |
| 중계 → AMR | `/{robot}/patrol_stop` | `std_srvs/SetBool` | `data=false` → 그 자리 정지 |
| 중계 → AMR | `/{robot}/patrol_resume` | `std_msgs/Bool` | `true` → 멈춘 단계부터 재개 |

## 동작

| 입력 | 중계 동작 | Patrol 피드백 · 결과 |
|---|---|---|
| Patrol Goal(`robot_id` 일치) | 수락, 토큰 대기 | `WAITING_FOR_TOKEN` |
| 이 로봇의 유효한 `DriveToken` | `patrol_route`에 순찰 목표 전달 | `PATROLLING` · `token_valid` · `accepted_token_sequence` |
| `PatrolCommand` 1 (`MOVE_TO_SAFE_ZONE`) | `patrol_stop` 호출(안전구역 이동이 아니라 그 자리 정지) | `BLOCKED` · `COMMAND_EXECUTING` |
| `PatrolCommand` 2 (`RESUME_PATROL`) | `patrol_resume` 발행 | `RESUMING` → `PATROLLING` · `COMMAND_ACCEPTED` |
| 토큰 회수(빈 토큰) · lease 만료 | `patrol_stop` 호출, 다시 부여되면 재개 | `BLOCKED` |
| Goal 취소 | `patrol_stop` 호출 | `CANCELED` · `REASON_CONTROL_CANCELED` |
| 순찰 완료(도킹) / 실패 | — | `SUCCEEDED` / `FAILED` · `REASON_NAVIGATION_FAILED` |

정지 사유(명령 · 토큰)가 겹치면 둘 다 풀려야 재개합니다. `genius_patrol`은 순찰 목표 취소를 받지 않으므로, 취소 후 같은 로봇에 새 Goal이 오면 남아 있는 경로를 재개합니다.

## 실행

```bash
ros2 run patrol_amr_safety genius_patrol --robot-id robot1 --ros-args -r patrol_action:=patrol_route
ros2 launch patrol_amr_bridge amr_bridge.launch.py robot_id:=robot1
ros2 run patrol_control patrol_control_node --ros-args -p auto_start:=true
```

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `robot_id` | `robot1` | 담당 로봇(`robot1` · `robot6`) |
| `route_action` | `patrol_route` | `genius_patrol` 순찰 서버 이름(로봇 namespace 아래) |
| `feedback_hz` | 2.0 | Patrol 피드백 주기 |
| `server_timeout_sec` | 5.0 | AMR 서버 · 서비스 대기 시간 |
| `token_grace_sec` | 0.5 | 토큰 lease에 더하는 여유 시간 |

## 시험

```bash
colcon build --packages-select patrol_interfaces patrol_amr_bridge && source install/setup.bash
cd src/patrol_amr_bridge && python3 -m pytest test -q    # 25 passed
```

| 시험 파일 | 개수 | 확인하는 것 |
|---|---|---|
| `test_bridge_core.py` | 16 | ROS 없이 판단 로직 — 토큰 대기 · 정지 · 재개 · 회수 · 취소 · 실패 |
| `test_bridge_integration.py` | 7 | rclpy로 모의 관제 + 모의 `genius_patrol`/`event_check`(같은 이름)와 실제 통신, 취소 후 경로 재개 |
| `test_flake8.py` · `test_pep257.py` | 2 | 코드 스타일 |

실제 `patrol_control_node` + 중계 + 모의 AMR로 자동 시작 → 토큰 부여 → 차량 진입 정지 → 재개 → `SUCCEEDED`까지 확인했습니다.

## 파일

```text
patrol_amr_bridge/
├── patrol_amr_bridge/bridge_core.py      ROS 없는 판단 로직
├── patrol_amr_bridge/amr_bridge_node.py  ROS 2 노드
├── launch/amr_bridge.launch.py
└── test/
```
