#!/usr/bin/env python3
"""
Stop confirmation and map position used by mission_supervisor.

From the team's move_to_safetyzone module: the actual-stop check (Nav2 task
cancelled, then fresh odometry below the speed limits for stop_hold seconds)
and the current map position from TF. Safe-zone coordinates are in
config/site.yaml.
"""

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

STOP_CHECK_DEFAULTS = {  # 정지 확인 기준값
    'linear-epsilon': 0.01,   # m/s
    'angular-epsilon': 0.02,  # rad/s
    'stop-hold': 0.5,         # s
    'stop-timeout': 10.0,     # s
    'data-max-age': 1.0,      # s
}


class Evacuation:
    """Stop check and map position on the shared patrol navigator."""

    def __init__(self, nav, args):
        self.nav, self.args = nav, args
        self.odom = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, nav)
        self.odom_sub = nav.create_subscription(
            Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.safe = [nav.getPoseStamped([x, y], yaw) for x, y, yaw in args.safe]

    def on_odom(self, msg):
        self.odom = msg

    def tick(self):
        if not rclpy.ok():
            raise RuntimeError('ROS shutdown')
        rclpy.spin_once(self.nav, timeout_sec=0.1)

    def fresh(self, stamp):
        age = (self.nav.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return 0 <= age <= self.args.data_max_age

    def position(self):
        transform = self.tf.lookup_transform('map', self.args.base_frame, Time())
        if not self.fresh(transform.header.stamp):
            raise RuntimeError('Current map position is stale')
        p = transform.transform.translation
        if not all(math.isfinite(v) for v in (p.x, p.y)):
            raise RuntimeError('Invalid current position')
        return p.x, p.y

    def stop(self):
        deadline = time.monotonic() + self.args.stop_timeout
        # Bound cancellation acknowledgement too; cancelTask() itself waits forever.
        if self.nav.result_future is not None and not self.nav.result_future.done():
            cancel = self.nav.goal_handle.cancel_goal_async()
            while not cancel.done():
                if time.monotonic() >= deadline:
                    raise RuntimeError('Action cancellation acknowledgement timeout')
                self.tick()
            cancel.result()  # propagate transport failure; still verify action termination
        self.nav.info('[SAFETY_STOP] cancellation checked; waiting for action termination')
        while not self.nav.isTaskComplete():
            if time.monotonic() >= deadline:
                raise RuntimeError('Action termination timeout')
            self.tick()
        # 취소 이전 측정값을 정지 확인에 재사용하지 않음
        self.odom = None
        after_stop_stamp = self.nav.get_clock().now().nanoseconds
        settled = None
        first_stamp = None
        self.nav.info(
            '[SAFETY_STOP] action termination check passed; waiting for fresh stopped odometry')
        while time.monotonic() < deadline:
            self.tick()
            msg = self.odom
            stopped = False
            if (msg is not None and self.fresh(msg.header.stamp)
                    and Time.from_msg(msg.header.stamp).nanoseconds > after_stop_stamp):
                v, w = msg.twist.twist.linear, msg.twist.twist.angular
                stopped = (math.hypot(v.x, v.y, v.z) <= self.args.linear_epsilon
                           and math.hypot(w.x, w.y, w.z) <= self.args.angular_epsilon)
            if not stopped:
                settled = first_stamp = None
                continue
            stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            if settled is None:
                settled, first_stamp = time.monotonic(), stamp
            if (stamp - first_stamp >= self.args.stop_hold
                    and time.monotonic() - settled >= self.args.stop_hold):
                self.nav.info('[SAFETY_STOP] fresh odometry confirmed a stop')
                return
        raise RuntimeError('Fresh odometry did not confirm a stop')
