# patrol_interfaces

AMR, 관제, CCTV 비전, 시스템 모니터가 함께 쓰는 ROS 2 인터페이스 패키지다.

현재 버전은 `2.1.0`이다. 필드 의미와 사용 규칙은 [공용 인터페이스 문서](../../docs/interfaces.md)를 따른다.

```text
patrol_interfaces/
├── action/
│   └── Patrol.action               관제 ↔ AMR 순찰
├── msg/
│   ├── PatrolCommand.msg           안전구역 이동 · 순찰 재개
│   ├── DriveToken.msg              주행 권한
│   ├── EStop.msg                   비상정지 상태 (/control/estop)
│   └── CameraState.msg             CCTV 차량 상태
└── srv/
    ├── ReportDetection.srv         확정 사건 · 사진 저장
    └── SetEStop.srv                운영자 비상정지 요청 (/control/set_estop)
```

## 2.1.0 변경

2.0.0에서 바뀐 점:

- 추가 `srv/SetEStop.srv`: 운영자가 관제에 EStop을 걸고 푸는 서비스
- 추가 `Patrol.action` Result `REASON_LOW_BATTERY=11`: 배터리 부족으로 귀환·도킹을 마침
- 제거 `msg/DetectionEvidence.msg`: 증거 사진을 별도 토픽으로 보내려던 타입. 사진은 `ReportDetection.image`로 보낸다
- 제거 `msg/MissionExecutionEvent.msg`: AMR 명령 게이트웨이와 임무 실행부를 나눌 때 쓰려던 타입. `mission_supervisor`가 Patrol Action을 직접 받으므로 필요 없다

남은 타입의 필드는 2.0.0과 같다.

`EStop.msg`는 주석만 보강했다(QoS, 적용 규칙, reason 표).

## 사용

- 각 PC는 같은 Git commit의 패키지를 빌드한다.
- AMR 안의 `mission_supervisor` ↔ 감지 노드 연결은 이 패키지가 아니라 `patrol_amr_interfaces/DetectEvent`를 쓴다.
- 소스와 설치 타입은 `python3 scripts/verify_interface_v1.py`로 확인한다. 스크립트 파일명은 기존 실행 경로와 맞추려고 유지했으며 검사 대상은 2.1.0이다.
