# 관제 Safety Arbiter -> AMR local_safety_supervisor · mission_supervisor, System monitor.
# 토픽: /control/estop  QoS: RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)
# 발행자는 관제 하나이며 대상마다 현재 상태를 1초 주기로 다시 발행한다.
# - active=true: 대상 AMR은 즉시 정지하고 진행 중인 동작을 취소한다(sequence와 무관하게 적용).
# - active=false: sequence가 직전보다 클 때만 적용한다. 해제만으로는 다시 움직이지 않는다.
# target_robot_id: robot1 / robot6 / all
# reason: 0 UNKNOWN, 1 OPERATOR, 2 COMMUNICATION, 3 TOKEN, 4 OBSTACLE,
#         5 KEEPOUT_FAILURE, 6 SYSTEM_FAULT
std_msgs/Header header
string target_robot_id
bool active
uint8 reason
uint64 sequence
