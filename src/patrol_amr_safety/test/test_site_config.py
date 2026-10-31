"""config/site.yaml: the patrol site coordinates and their checks."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patrol_amr_safety.site_config import load_site, SPIN  # noqa: E402
import pytest  # noqa: E402

SOURCE_SITE = Path(__file__).resolve().parents[1] / 'config' / 'site.yaml'


def test_site_file_keeps_the_demo_route_docks_and_safe_zones():
    site = load_site(SOURCE_SITE)
    waypoints = [step for step in site.route if step != SPIN]
    assert [wp[0] for wp in waypoints] == ['WP1', 'WP2', 'WP3', 'WP4', 'WP5', 'WP6', 'WP3', 'WP7']
    assert waypoints[0] == ('WP1', (-0.206, -1.038), 'WEST')
    assert site.route.count(SPIN) == 8                       # 경유지마다 360° 회전
    assert site.docks['robot1'] == ((0.0, 0.0), 'NORTH')
    assert site.docks['robot6'] == ((0.0, -0.7), 'NORTH')
    assert site.initial_poses['robot6'] == ((0.0, -0.7), 'SOUTH')
    assert sorted(site.safe_zones) == [1, 2, 4, 5, 6, 7]     # WP3은 안전구역 아님


def test_unknown_direction_is_rejected(tmp_path):
    bad = tmp_path / 'site.yaml'
    bad.write_text(SOURCE_SITE.read_text(encoding='utf-8').replace('dir: WEST', 'dir: LEFT', 1),
                   encoding='utf-8')
    with pytest.raises(ValueError, match='unknown direction'):
        load_site(bad)


def test_missing_route_is_rejected(tmp_path):
    bad = tmp_path / 'site.yaml'
    bad.write_text('route: []\ndocks: {robot1: {x: 0, y: 0, dir: NORTH, initial_dir: SOUTH}}\n'
                   'safe_zones: {1: [0, 0, 0]}\n', encoding='utf-8')
    with pytest.raises(ValueError, match='route'):
        load_site(bad)
