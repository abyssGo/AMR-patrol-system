"""EStop, docking and output-arbitration rules (no ROS graph needed)."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patrol_amr_safety.dock_check import (  # noqa: E402
    DockCheck, DOCKED_HOLD, POWER_SUPPLY_STATUS_CHARGING, POWER_SUPPLY_STATUS_FULL)
from patrol_amr_safety.estop_check import EStopCheck, EStopFrame, STALE_AFTER  # noqa: E402
from patrol_amr_safety.local_safety_supervisor import (  # noqa: E402
    CANDIDATE_MAX_AGE, NAV2, select_candidate, YAW)
import pytest  # noqa: E402

DISCHARGING = 2


# ── EStop ───────────────────────────────────────────────────────────────────
def frame(target, active, seq, reason=1):
    return EStopFrame(target, active, reason, seq)


def test_estop_for_this_robot_or_all_is_active():
    check = EStopCheck('robot1')
    check.observe(frame('robot6', True, 1), 0.0)
    assert not check.active()                             # 다른 로봇 대상
    check.observe(frame('all', True, 2), 0.0)
    assert check.active()
    check.observe(frame('all', False, 3), 0.0)
    check.observe(frame('robot1', True, 4, reason=6), 0.0)
    assert check.active() and check.reason() == 6


def test_activation_applies_even_with_an_old_sequence():
    check = EStopCheck('robot1')
    check.observe(frame('robot1', False, 100), 0.0)
    check.observe(frame('robot1', True, 5), 0.1)          # 재시작 등으로 낮은 sequence
    assert check.active()


def test_release_needs_a_newer_sequence():
    check = EStopCheck('robot1')
    check.observe(frame('robot1', True, 10), 0.0)
    check.observe(frame('robot1', False, 9), 0.1)         # 지난 해제 프레임
    assert check.active()
    check.observe(frame('robot1', False, 11), 0.2)
    assert not check.active()


def test_link_loss_blocks_motion_and_recovers():
    check = EStopCheck('robot1')
    assert check.blocks_motion(0.0)                       # 한 번도 못 받음
    check.observe(frame('all', False, 1), 0.0)
    assert not check.blocks_motion(STALE_AFTER - 0.1)
    assert check.blocks_motion(STALE_AFTER + 0.1)
    check.observe(frame('robot6', False, 2), STALE_AFTER + 0.2)   # 어느 대상이든 링크 증거
    assert not check.blocks_motion(STALE_AFTER + 0.3)


# ── Docking (CR 9/9, adapted) ───────────────────────────────────────────────
def test_docked_needs_contact_hold_and_charging_after_docking():
    dock = DockCheck()
    dock.observe_battery(True, POWER_SUPPLY_STATUS_CHARGING, 0.5, 0.0)   # 도킹 전 샘플
    dock.observe_dock(True, 1.0)
    assert not dock.docked(1.0 + DOCKED_HOLD + 1)         # 도킹 뒤 배터리 샘플이 없다
    dock.observe_battery(True, POWER_SUPPLY_STATUS_CHARGING, 0.5, 2.0)
    assert not dock.docked(1.0 + DOCKED_HOLD - 0.1)       # 2초 유지 전
    assert dock.docked(1.0 + DOCKED_HOLD)


def test_full_battery_counts_and_discharging_does_not():
    dock = DockCheck()
    dock.observe_dock(True, 0.0)
    dock.observe_battery(True, DISCHARGING, 0.9, 1.0)
    assert not dock.docked(5.0)
    dock.observe_battery(True, POWER_SUPPLY_STATUS_FULL, 1.0, 6.0)
    assert dock.docked(6.0)


def test_contact_break_restarts_the_rule():
    dock = DockCheck()
    dock.observe_dock(True, 0.0)
    dock.observe_battery(True, POWER_SUPPLY_STATUS_CHARGING, 0.5, 1.0)
    dock.observe_dock(False, 2.5)
    dock.observe_dock(True, 3.0)
    assert not dock.docked(10.0)                          # 끊긴 뒤 충전 증거가 다시 필요
    assert dock.undocked() is False


def test_missing_battery_present_is_not_charging():
    dock = DockCheck()
    dock.observe_dock(True, 0.0)
    dock.observe_battery(False, POWER_SUPPLY_STATUS_CHARGING, float('nan'), 1.0)
    assert not dock.docked(5.0)
    assert dock.percentage is None


# ── local safety arbitration (CR 9/9) ───────────────────────────────────────
@pytest.mark.parametrize('rx, expected', [
    ({NAV2: None, YAW: None}, None),
    ({NAV2: 10.0, YAW: None}, NAV2),
    ({NAV2: None, YAW: 10.0}, YAW),
    ({NAV2: 10.0, YAW: 10.0}, None),                      # 둘 다 신선 → 0
    ({NAV2: 10.0 - CANDIDATE_MAX_AGE - 0.01, YAW: 10.0}, YAW),
])
def test_exactly_one_fresh_candidate_passes(rx, expected):
    assert select_candidate(rx, 10.0) == expected
