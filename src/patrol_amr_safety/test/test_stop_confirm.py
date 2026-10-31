"""mission_supervisor가 쓰는 move_to_safetyzone.Evacuation.stop의 정지 확인 규칙(기존 시험 유지)."""

from pathlib import Path
import sys
from types import SimpleNamespace

from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import patrol_amr_safety.move_to_safetyzone as safety  # noqa: E402
from patrol_amr_safety.move_to_safetyzone import Evacuation  # noqa: E402
import pytest  # noqa: E402
from rclpy.clock import ClockType  # noqa: E402
from rclpy.task import Future  # noqa: E402
from rclpy.time import Time  # noqa: E402


def stop_harness(monkeypatch, *, moving=False, stale=False, cancel_done=True,
                 action_done=True, queued_before_stop=False):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(safety.time, 'monotonic', lambda: clock.now)
    result, cancellation = Future(), Future()
    if action_done:
        result.set_result(None)
    if cancel_done:
        cancellation.set_result(SimpleNamespace(goals_canceling=[]))
    nav = SimpleNamespace(
        result_future=result,
        goal_handle=SimpleNamespace(cancel_goal_async=lambda: cancellation),
        isTaskComplete=lambda: result.done(),
        info=lambda _: None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: Time(seconds=clock.now, clock_type=ClockType.ROS_TIME)))
    helper = Evacuation.__new__(Evacuation)
    helper.nav = nav
    helper.args = SimpleNamespace(stop_timeout=0.8, stop_hold=0.2,
                                  data_max_age=1.0, linear_epsilon=0.01,
                                  angular_epsilon=0.02)
    samples = []

    def tick():
        clock.now += 0.1
        msg = Odometry()
        stamp = 8.0 if stale else (9.99 if queued_before_stop else clock.now)
        msg.header.stamp = Time(seconds=stamp).to_msg()
        msg.twist.twist.linear.x = 0.1 if moving else 0.0
        helper.on_odom(msg)
        samples.append(msg)

    helper.tick = tick
    return helper, samples


def test_actual_stop_needs_multiple_fresh_stopped_odometry_samples(monkeypatch):
    helper, samples = stop_harness(monkeypatch)
    helper.stop()
    assert len(samples) >= 3


@pytest.mark.parametrize('options', [
    {'moving': True}, {'stale': True}, {'queued_before_stop': True},
    {'action_done': False}, {'action_done': False, 'cancel_done': False},
])
def test_stop_does_not_ack_movement_stale_data_or_unfinished_cancel(monkeypatch, options):
    helper, _ = stop_harness(monkeypatch, **options)
    with pytest.raises(RuntimeError, match='timeout|did not confirm'):
        helper.stop()
