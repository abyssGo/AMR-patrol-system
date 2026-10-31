"""DriveToken validity rule shared by mission_supervisor and local_safety_supervisor."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patrol_amr_safety.drive_token_check import DriveTokenCheck, TokenFrame  # noqa: E402
import pytest  # noqa: E402


def frame(token='t', holder='robot1', lease=1.0, seq=1):
    return TokenFrame(token, holder, lease, seq)


def test_grant_and_lease_expiry():
    c = DriveTokenCheck('robot1')
    assert not c.valid(0.0)
    assert c.observe(frame(seq=1), now=10.0)
    assert c.valid(10.5)
    assert not c.valid(11.0)          # lease 1.0 s measured from receive time
    assert c.accepted_sequence == 1


def test_other_robot_is_ignored():
    c = DriveTokenCheck('robot1')
    assert not c.observe(frame(holder='robot6'), now=0.0)
    assert not c.valid(0.1)


def test_empty_token_revokes_immediately():
    c = DriveTokenCheck('robot1')
    c.observe(frame(seq=1), now=0.0)
    assert c.observe(frame(token='', seq=2), now=0.2)
    assert not c.valid(0.3)


def test_out_of_order_frame_ignored_while_live():
    c = DriveTokenCheck('robot1')
    c.observe(frame(seq=5), now=0.0)
    assert not c.observe(frame(token='', seq=4), now=0.1)   # stale revocation
    assert c.valid(0.2)


def test_lower_sequence_accepted_after_lapse_restarted_control():
    c = DriveTokenCheck('robot1')
    c.observe(frame(seq=50), now=0.0)
    assert not c.valid(2.0)
    assert c.observe(frame(seq=1), now=2.0)                 # control restarted
    assert c.valid(2.5)


def test_refresh_extends_lease():
    c = DriveTokenCheck('robot1')
    c.observe(frame(seq=1), now=0.0)
    c.observe(frame(seq=2), now=0.8)
    assert c.valid(1.5)
    assert not c.valid(1.81)


def test_empty_robot_id_rejected():
    with pytest.raises(ValueError):
        DriveTokenCheck('')
