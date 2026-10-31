#!/usr/bin/env python3
"""
감지 노드 → mission_supervisor 정지 · 재개 연결.

설계도 01: patrol_stop(SetBool, data=false) · patrol_resume(Bool).
"""

import time

from std_msgs.msg import Bool
from std_srvs.srv import SetBool


STOP_RETRY_BACKOFF = 2.0   # s


class VisionPatrolLink:
    """One vision event: stop, report, then resume after report success."""

    def __init__(self, node, robot_id):
        self.node = node
        self.stop_service_name = f'/{robot_id}/patrol_stop'
        self.resume_topic_name = f'/{robot_id}/patrol_resume'
        self.client = node.create_client(SetBool, self.stop_service_name)
        self.resume_pub = node.create_publisher(Bool, self.resume_topic_name, 2)
        self.pending = None
        self.busy = False
        self._retry_after = 0.0   # 정지 요청이 거절되면 잠시 다시 보내지 않는다
        self.stopped = False
        self._last_wait_log = {}
        self.node.get_logger().info(
            f'Vision patrol link ready: stop service={self.stop_service_name}; '
            f'resume topic={self.resume_topic_name}')

    def _log_wait(self, reason, message):
        now = time.monotonic()
        last_logged = self._last_wait_log.get(reason)
        if last_logged is None or now - last_logged >= 5.0:
            self._last_wait_log[reason] = now
            self.node.get_logger().warning(message)

    def request(self):
        if time.monotonic() < self._retry_after:
            return False
        if self.busy:
            self._log_wait(
                'busy', f'Patrol stop request skipped: previous event still in progress; '
                f'stop confirmed={self.stopped}; service={self.stop_service_name}')
            return False
        if not self.client.service_is_ready():
            self._log_wait(
                'unavailable', f'Patrol stop service unavailable: {self.stop_service_name}; '
                'stop request was not sent')
            return False
        self.busy = True
        self.node.get_logger().info(
            f'Sending patrol stop request: service={self.stop_service_name}; data=false')
        try:
            self.pending = self.client.call_async(SetBool.Request(data=False))
        except Exception as exc:
            self.busy = False
            self.node.get_logger().error(f'Patrol stop request dispatch failed: {exc}')
            raise
        self.pending.add_done_callback(self._done)
        return True

    def _done(self, future):
        try:
            response = future.result()
            self.node.get_logger().info(
                f'Patrol stop response received: service={self.stop_service_name}; '
                f'success={response.success}; message={response.message}')
            if not response.success:
                raise RuntimeError(response.message)
        except Exception as exc:
            self.busy = False
            # 도킹 · 안전구역 이동 중처럼 정지 요청을 받지 않는 구간: 프레임마다 다시 보내지 않는다.
            self._retry_after = time.monotonic() + STOP_RETRY_BACKOFF
            self.node.get_logger().warning(f'Patrol stop not accepted: {exc}')
            return
        self.stopped = True

    def release(self):
        """End this event without publishing resume (the patrol owner resumes itself)."""
        self.node.get_logger().info('Patrol event released without resume publish')
        self.busy = False
        self.stopped = False

    def resume(self):
        self.resume_pub.publish(Bool(data=True))
        self.node.get_logger().info(
            f'Patrol resume published: topic={self.resume_topic_name}; data=true')
        self.busy = False
        self.stopped = False
