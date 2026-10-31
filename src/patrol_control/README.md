# patrol_control

관제(Control Server) 패키지다(`0.3.0`, `patrol_interfaces 2.1.0`). 두 AMR을 한 번에 한 대씩
교대로 순찰시키고, 주행 권한(DriveToken), 차량 대피·재개 명령, 화재 보류, 비상정지를 관리한다.
설계는 [docs/control_server.md](../../docs/control_server.md)를 따른다.

## 실행

```bash
ros2 run patrol_control patrol_control_node
```

교대 순찰은 운영자가 시작한다. 띄우자마자 시작하려면 `-p auto_start:=true`를 준다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `robot_ids` | `[robot1, robot6]` | 관리할 로봇과 교대 순서 |
| `initial_robot_id` | 첫 번째 로봇 | 처음 출발할 로봇 |
| `auto_start` | `false` | 시작하자마자 교대 순찰을 켬 |
| `token_publish_hz` | `5.0` | DriveToken 발행 주기 |
| `token_lease_seconds` | `1.0` | DriveToken lease |

## 운영 명령

```bash
ros2 service call /control/start_patrol std_srvs/srv/Trigger       # 교대 시작, 제외된 로봇 복귀
ros2 service call /control/cancel_patrol std_srvs/srv/Trigger      # 순찰 취소, 교대 정지
ros2 service call /control/clear_fire_hold std_srvs/srv/Trigger    # 화재 보류 해제
ros2 service call /control/set_estop patrol_interfaces/srv/SetEStop \
  "{target_robot_id: all, active: true, reason: 1}"                 # 비상정지 (robot1 / robot6 / all)
ros2 service call /control/resume std_srvs/srv/Trigger             # EStop 해제 뒤 재개
```

## 동작

- 교대가 켜져 있고, 순찰 중인 로봇이 없고, `fire_hold`·전체 EStop이 없고, CCTV `patrol_allowed=true`일 때 차례인 로봇을 출발시킨다.
- 대상 로봇은 순서에서 빠지지 않았고, 충전 대기가 아니고, EStop·resume 대기가 아니고, 배터리 30% 이상이어야 한다.
- Goal 수락 뒤 `WAITING_FOR_TOKEN`을 받아야 Token을 준다.
- `patrol_allowed` false 전이 → `MOVE_TO_SAFE_ZONE`, true 전이 → `RESUME_PATROL`을 한 번씩 보낸다.
- `DETECTION_CONFIRMED + FIRE` → `fire_hold`. 현재 임무는 마치고, 해제 전까지 다음 순찰을 막는다.
- Result가 오면 Token을 회수하고:
  - `SUCCEEDED` 또는 `FAILED(LOW_BATTERY)` → 도킹한 것으로 보고 다음 로봇 출발(배터리 부족 로봇은 80%까지 충전 대기)
  - 그 밖의 `FAILED` → 그 로봇을 순서에서 빼고 다른 로봇으로 계속
  - `CANCELED` → 교대 정지
- EStop은 관제만 `/control/estop`으로 발행한다. 해제 뒤 `/control/resume`이 있어야 로봇이 다시 움직인다.
- 다시 띄우면 `~/.ros/patrol_control/state.json`에서 상태를 불러오고, AMR에 남은 임무가 있으면 취소하고 끝날 때까지 출발시키지 않는다.

## 파일

| 파일 | 내용 |
|---|---|
| `patrol_control/control_core.py` | ROS 없는 상태 머신(판단 규칙 전부) |
| `patrol_control/patrol_control_node.py` | ROS 노드: Action Client, 발행·구독, 운영 서비스, 감시 타이머 |
| `patrol_control/control_state.py` | 상태 파일 읽기·쓰기 |

## 시험

```bash
cd src/patrol_control && python3 -m pytest -q test
```
