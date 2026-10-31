# 전체 시스템 구성

> 기준일: 2026-09-13 · 계약: `patrol_interfaces 2.1.0` · 관련: [인터페이스](interfaces.md), [통합](integration.md)

## 1. 개발 단위와 실행 위치

| 개발 단위 | 통합 실행 위치 | 주요 책임 |
|---|---|---|
| AMR | PC 1 | AMR1 제어: 센서, 임무, Nav2·AMCL, 로컬 안전, 배터리, 도킹, Detection·증적 |
| AMR | PC 2 | AMR2 제어: PC 1과 공통 기능, AMR2 LiDAR 위치 검증 관련 역할 |
| 관제 | PC 3 | Control Server: 교대 순찰, Patrol Action·명령·주행 권한·EStop, 운영 서비스 |
| 시스템 모니터 | PC 3 | 판단 결과 토픽 수신·표시, DB, 읽기 전용 Dashboard |
| 비전 | PC 4 | gate_cam·center_cam, cam_master, CameraState·patrol_allowed |

PC 1·2의 역할과 TB4 내부 컴퓨터의 실제 프로세스 배치를 동일한 것으로 단정하지 않는다. 장치 드라이버·Create 3·republisher 등의 실제 배치는 TBD-ARCH-001에서 관리한다. 위 표는 기능 책임과 통합 실행 위치를 나타내며 개발자의 작업 PC를 지정하지 않는다.

관제와 시스템 모니터는 별도 개발 단위이며 각각 [control_server.md](control_server.md)와 [monitoring_and_data.md](monitoring_and_data.md)에서 다룬다. 실제 시스템 실행 시 시스템 모니터는 관제 코드와 함께 관제 PC(PC 3)에서 실행한다. AMR은 PC 1·2, 비전은 PC 4의 역할을 유지한다. 시스템 모니터의 실행 위치를 PC 3으로 지정해도 두 팀의 개발 책임은 분리한다.

## 2. 식별 체계

| 대상 | 담당 PC | robot_id | namespace | 화면 표시명 | Onboard 서버 ID |
|---|---|---|---|---|---|
| 첫 번째 AMR | PC 1 | robot1 | /robot1 | AMR1 | 1 |
| 두 번째 AMR | PC 2 | robot6 | /robot6 | AMR2 | 6 |

- 공통 관제 namespace는 /control, CCTV 비전 namespace는 /vision이다.
- robot_id와 holder_robot_id는 robot1 또는 robot6을 사용한다. 화면 표시명을 메시지 식별자로 쓰지 않는다.
- 관제는 `robot_ids` 파라미터로, AMR 노드는 실행 namespace로 robot_id를 정한다.
- ROS_DOMAIN_ID=6과 두 번째 로봇의 Onboard 서버 ID=6은 별개 설정이다.

## 3. 기능 연결

~~~mermaid
flowchart LR
    V[PC4: CCTV / cam_master] -->|patrol_allowed| C[PC3: Control Server]
    OP[운영자 터미널] -->|/control/* 서비스| C
    C -->|Patrol Action / PatrolCommand| M[PC1·2: mission_supervisor]
    C -->|DriveToken / EStop| S[AMR: local_safety_supervisor]
    M -->|Nav2 · Create 3 Action| N[Nav2 / Create 3]
    N -->|cmd_vel_safety_in| S
    DV[AMR: vision_node_v2] -->|cmd_vel_yaw| S
    M -->|DetectEvent| DV
    S -->|최종 cmd_vel| B[로봇 구동부]
    M -->|Patrol Feedback / Result| C
    DV -->|ReportDetection| O[PC3: 시스템 모니터]
    C -.->|Action 상태 · EStop 구독| O
    O --> D[읽기 전용 Dashboard]
~~~

Nav2와 로컬 감지에서 생기는 모든 주행 후보는 AMR 내부의 `local_safety_supervisor`를 통과한다. 최종 `/{robot}/cmd_vel`은 이 노드만 발행하며, 유효한 DriveToken이 없거나 만료되면, 또는 EStop이 걸리면 0 속도를 낸다.

AMR 로컬 감지는 OAK-D 입력에서 후보·정렬·확정을 처리하며 PC 4의 CCTV 차량 상태 파이프라인과 별개다. `mission_supervisor`와 감지 노드 사이는 AMR 내부 Action `patrol_amr_interfaces/DetectEvent`로 연결한다. 관제는 `Patrol` Feedback으로 감지 진행·확정을 받고, 시스템 모니터는 `ReportDetection` Service로 확정 사건과 증거 사진을 저장한다.

두 로봇은 같은 지도(`final_project_map`, 원점은 `robot1` 도크)를 쓴다. `robot1` 도크는 (0, 0), `robot6` 도크는 (0, −0.7)이다.

## 4. Discovery 구성

각 TB4는 두 Discovery Server를 사용하는 의도적인 구성이다. Onboard 설정을 유지하며 중복으로 판단해 제거하지 않는다.

| 서버 | 식별·접속 | 역할 |
|---|---|---|
| PC 3 Offboard | ID 0, UDP 11811, <PC3_IP>:11811 | 전체 로봇·PC의 Discovery |
| robot1 Onboard | ID 1, UDP 11811 | Create 3를 포함한 로봇 내부 Discovery |
| robot6 Onboard | ID 6, UDP 11811 | Create 3를 포함한 로봇 내부 Discovery |

PC 1·2·4의 ROS 참여자는 PC 3 서버의 Client다. PC 3에서는 서버 프로세스를 실행하고, 자체 관제·모니터 ROS 참여자도 Client로 동작한다. 각 TB4의 Offboard 설정은 PC 3의 ID 0 서버를 가리키고, Enabled는 True다.

Onboard는 Create 3 내부 통신을 위한 Discovery 역할이며 안전 제어 알고리즘 자체가 아니다. Create 3가 Offboard에 직접 참여하거나 서버끼리 직접 연결된다고 추정하지 않는다. 생성된 실제 참여자별 설정 확인은 TBD-ARCH-001이다. 공식 TB4 설정 이력도 Create 3 로컬 서버와 로봇 측 Offboard 연결을 구분한다. [TB4 설정 이력](https://docs.ros.org/en/humble/p/turtlebot4_setup/__CHANGELOG.html)

Discovery는 참여자 발견을 위한 기능이다. 발견 후 사용자 데이터가 모두 PC 3 서버를 경유하는 구조가 아니다. 토픽 데이터의 직접 송수신 경로도 통합시험에서 확인한다. [Fast DDS Discovery 설명](https://fast-dds.docs.eprosima.com/en/3.x/fastdds/discovery/discovery_server.html)

실제 접속 설정에는 서버의 주소 또는 해석 가능한 호스트명이 필요하다. 모든 Client PC의 고정 IP 목록을 설계 문서 작성의 선행 조건으로 요구하지 않는다. 여러 서버의 환경변수 설정은 실제 Fast DDS 버전의 ID 매핑 규칙에 따라 작성하며 단순히 주소를 나열해 ID가 맞는다고 가정하지 않는다.

## 5. 환경 기준

공유 설계의 환경 기준은 Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.3, Fast DDS, ROS_DOMAIN_ID=6, 네트워크 turtle09, 지도 frame map이다. PC 3 주소의 설계 기록값은 192.168.109.12이며, 배포 시 실제 주소와 일치하는지 확인한다. TB4/Create 3의 펌웨어까지 같은 OS·ROS 버전이라고 단정하지 않는다.

시간 동기화는 기존 systemd-timesyncd 및 네트워크 제공/기본 NTP 정책을 유지한다. 토큰의 로컬 lease 경과 측정은 monotonic clock을 사용하며 송신 timestamp 기반 message age의 검증 방식은 별도 계약이다.

## 6. 기동·운영 책임

기동 순서와 확인 항목은 [integration.md](integration.md)에, 노드별 실행 명령은 [amr.md](amr.md)와 [control_server.md](control_server.md)에 둔다. 운영자 명령(순찰 시작·취소, 화재 보류 해제, EStop, 재개)은 PC 3 터미널에서 관제 ROS 서비스로 보낸다.

## TBD

| ID | 미정 사항 | 담당·협의 | 완료 조건 |
|---|---|---|---|
| TBD-ARCH-001 | 실제 노드·프로세스 배치, TB4 생성 설정, 참여자별 서버 목록 | AMR·관제·시스템 모니터·비전 | 배포 표와 실제 설정 대조, 내부·외부 토픽 송수신 확인 |
| TBD-ARCH-002 | 장비별 버전·서버 주소·NIC 및 실행 서비스 확인 | 각 단위 | 기존 환경을 조사한 배포 정보 확정 |

구조 및 로봇 식별자는 이 문서의 설계 기준이다. 위 TBD는 실배포 세부 사항이며 구조 설계 자체의 미확정을 뜻하지 않는다.
