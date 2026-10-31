"""Alignment bearing sign/size: turning by the bearing brings the bbox to image centre."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip('ultralytics')
from patrol_amr_safety.vision_node_v2 import bearing_from_bbox  # noqa: E402, I100

HFOV = math.radians(64.0)
W = 640


def box(cx, half=20):
    return (cx - half, 100, cx + half, 200)


def test_centre_is_zero():
    assert bearing_from_bbox(box(W / 2), W, HFOV) == pytest.approx(0.0)


def test_left_of_centre_turns_ccw_positive():
    assert bearing_from_bbox(box(100), W, HFOV) > 0.0


def test_right_of_centre_turns_cw_negative():
    assert bearing_from_bbox(box(540), W, HFOV) < 0.0


def test_image_edge_is_half_fov():
    assert bearing_from_bbox((0, 0, 0, 10), W, HFOV) == pytest.approx(HFOV / 2)
    assert bearing_from_bbox((W, 0, W, 10), W, HFOV) == pytest.approx(-HFOV / 2)


def test_pinhole_not_linear():
    # quarter-width offset: atan(0.5 * tan(32 deg)) ~= 17.35 deg, not 16 deg
    assert math.degrees(bearing_from_bbox(box(W / 4), W, HFOV)) == pytest.approx(17.35, abs=0.05)
