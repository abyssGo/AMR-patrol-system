# AMR 순찰 시스템 개발 문서

> 기준일: 2026-09-13 · 공용 계약: `patrol_interfaces 2.1.0`

두 대의 TurtleBot 4(`robot1`, `robot6`)와 CCTV 두 대로 주차장을 순찰하는 ROS 2
시스템의 설계 문서다. 통신 계약은 [interfaces.md](interfaces.md)가 기준이다.

## 문서 지도

| 문서 | 내용 |
|---|---|
| [architecture.md](architecture.md) | PC 배치, 로봇 식별, Discovery, 실행 환경 |
| [interfaces.md](interfaces.md) | Action·메시지·서비스·토픽 계약, 운영 서비스 |
| [scenarios.md](scenarios.md) | 상황별 시스템 동작 |
| [control_server.md](control_server.md) | 관제 판단과 코드 흐름 |
| [amr.md](amr.md) | AMR 순찰·안전·도킹·로컬 감지 |
| [vision.md](vision.md) | CCTV 차량 상태와 `patrol_allowed` |
| [monitoring_and_data.md](monitoring_and_data.md) | 감지 보고 저장과 읽기 전용 모니터링 |
| [integration.md](integration.md) | 통합 순서, 종단시험, 자동 시험 |
| [change_requests/](change_requests/README.md) | 개발 단위 사이 변경 요청 기록 |
| [decisions/](decisions/) | 설계 결정 기록 |
| [development/](development/) | 브랜치·PR·버전 관리 규칙 |

## 공용 인터페이스

```text
action/Patrol.action
msg/CameraState.msg
msg/DriveToken.msg
msg/EStop.msg
msg/PatrolCommand.msg
srv/ReportDetection.srv
srv/SetEStop.srv
```

- 순찰은 관제가 `Patrol` Action으로 시작하고, 진행·감지·결과를 Feedback·Result로 받는다.
- 실행 중 안전구역 이동과 재개는 `PatrolCommand`를 사용한다.
- 주행 권한은 로봇별 `DriveToken`으로 준다. 한 번에 한 로봇만 갖는다.
- 비상정지는 관제가 `/control/estop`으로 발행하고, 운영자는 `/control/set_estop`·`/control/resume`으로 다룬다.
- CCTV는 `CameraState`와 `patrol_allowed`를 제공한다.
- 확정 사건과 사진은 `ReportDetection`으로 저장한다.
- AMR 안의 `mission_supervisor`와 감지 노드는 `patrol_amr_interfaces/DetectEvent`로 연결한다.

## 책임 경계

- AMR: Patrol Action Server, Nav2·Create 3 연결, 최종 속도 통제, 도킹, 로컬 감지
- 관제: 교대 순찰, Patrol Action Client, `PatrolCommand`, `DriveToken`, EStop, 운영 서비스
- 비전: CCTV 차량 상태와 `patrol_allowed`
- 시스템 모니터: `ReportDetection` 저장과 읽기 전용 표시

관제와 시스템 모니터는 PC 3에서 함께 실행하지만 서로 다른 개발 단위다. 시스템
모니터는 운영 판단이나 주행 명령을 만들지 않는다. 최종 속도는 AMR의
`local_safety_supervisor`만 발행한다.

## 문서 적용 원칙

- 개발과 통합은 현재 `patrol_interfaces` 소스와 이 디렉터리의 설계 문서를 따른다.
- `change_requests/`와 `decisions/`는 당시 상태를 남긴 기록이다. 현재 문서와 다르면 현재 문서를 따른다.
- 모든 PC는 같은 Git commit의 인터페이스 패키지를 빌드한다.

## 환경

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, Fast DDS
- `ROS_DOMAIN_ID=6`
- 로봇: `robot1`, `robot6`
- 좌표계: `map` (지도 `maps/final_project_map.yaml`)
