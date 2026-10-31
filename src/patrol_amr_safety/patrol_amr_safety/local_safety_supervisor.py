#!/usr/bin/env python3
"""
local_safety_supervisor: the only publisher of the final `cmd_vel`.

Design (A-1_지능1_시스템_설계도 01 "로컬 주행 안전 · 최종 주행 속도 통제",
CR 9/8 E-stop, CR 9/9 yaw 정렬 Nav2 중재):

Two velocity candidates reach this node:
- `cmd_vel_safety_in`: Nav2's last stage (collision_monitor output, see
  turtlebot4_navigation/config/nav2.yaml),
- `cmd_vel_yaw`: in-place yaw from the detection/alignment node.

A candidate is fresh for CANDIDATE_MAX_AGE after it was received. When exactly
one candidate is fresh it is forwarded; when both are fresh nothing tells which
one to trust, so the output is zero.

The forwarded command passes the safety gate only while
- this robot holds a valid DriveToken, and
- no EStop is active for this robot or for all robots, and
- EStop frames from the control Safety Arbiter are still arriving.
Otherwise zero velocity is published.

Docking and undocking are Create 3 actions that drive the base directly; they
are not gated here. mission_supervisor starts them only with a valid token and
cancels them on EStop.
"""

import time

from geometry_msgs.msg import TwistStamped
from patrol_interfaces.msg import DriveToken, EStop
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

if __package__:
    from .drive_token_check import DriveTokenCheck, TokenFrame, token_qos
    from .estop_check import ESTOP_TOPIC, EStopCheck, EStopFrame, estop_qos
else:
    from drive_token_check import DriveTokenCheck, TokenFrame, token_qos
    from estop_check import ESTOP_TOPIC, EStopCheck, EStopFrame, estop_qos

CANDIDATE_MAX_AGE = 0.5   # s (CR 9/9 후보 신선도)
NAV2, YAW = 'nav2', 'yaw'


def select_candidate(last_rx: dict, now: float):
    """Return the single fresh source name, or None (none fresh / both fresh)."""
    fresh = [name for name, t in last_rx.items()
             if t is not None and now - t <= CANDIDATE_MAX_AGE]
    return fresh[0] if len(fresh) == 1 else None


class LocalSafetySupervisor(Node):
    """Arbitrate Nav2 / yaw candidates and gate them by DriveToken and EStop."""

    def __init__(self):
        super().__init__('local_safety_supervisor')
        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            raise RuntimeError('local_safety_supervisor must run in a robot namespace')
        self.token = DriveTokenCheck(self.robot_id)
        self.estop = EStopCheck(self.robot_id)
        self._last_rx = {NAV2: None, YAW: None}
        self._was_allowed = False
        self._last_block = ''
        self.out = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(DriveToken, 'drive_token', self.on_token, token_qos())
        self.create_subscription(EStop, ESTOP_TOPIC, self.on_estop, estop_qos())
        self.create_subscription(
            TwistStamped, 'cmd_vel_safety_in',
            lambda msg: self.on_cmd(NAV2, msg), qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, 'cmd_vel_yaw',
            lambda msg: self.on_cmd(YAW, msg), qos_profile_sensor_data)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f'local_safety_supervisor ready: robot={self.robot_id} '
            'in=cmd_vel_safety_in,cmd_vel_yaw out=cmd_vel')

    def block_reason(self, now) -> str:
        if not self.token.valid(now):
            return 'no valid DriveToken'
        if self.estop.active():
            return f'EStop active (reason={self.estop.reason()})'
        if self.estop.stale(now):
            return 'EStop link lost'
        return ''

    def on_token(self, msg: DriveToken) -> None:
        lease = msg.lease_duration.sec + msg.lease_duration.nanosec / 1e9
        self.token.observe(
            TokenFrame(msg.token, msg.holder_robot_id, lease, msg.sequence),
            time.monotonic())

    def on_estop(self, msg: EStop) -> None:
        changed = self.estop.observe(
            EStopFrame(msg.target_robot_id, msg.active, msg.reason, msg.sequence),
            time.monotonic())
        if changed and self.estop.active():
            # 활성 즉시 정지: 타이머를 기다리지 않는다.
            self._publish_zero()

    def on_cmd(self, source: str, msg: TwistStamped) -> None:
        now = time.monotonic()
        self._last_rx[source] = now
        if self.block_reason(now):
            self._publish_zero()
            return
        selected = select_candidate(self._last_rx, now)
        if selected == source:
            self.out.publish(msg)
        else:
            # 두 후보가 동시에 신선하면 어느 쪽도 믿지 않는다.
            self._publish_zero()

    def on_timer(self) -> None:
        now = time.monotonic()
        block = self.block_reason(now)
        allowed = not block
        if allowed != self._was_allowed or block != self._last_block:
            self.get_logger().info(
                f'drive output {"ENABLED" if allowed else "BLOCKED: " + block} '
                f'(token_sequence={self.token.sequence})')
            if not allowed:
                self._publish_zero()
            self._was_allowed, self._last_block = allowed, block
        if allowed and select_candidate(self._last_rx, now) is None and all(
                t is not None and now - t <= CANDIDATE_MAX_AGE
                for t in self._last_rx.values()):
            self._publish_zero()

    def _publish_zero(self) -> None:
        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        zero.header.frame_id = 'base_link'
        self.out.publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = LocalSafetySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
