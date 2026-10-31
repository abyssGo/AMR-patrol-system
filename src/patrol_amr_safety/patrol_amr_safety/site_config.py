"""Patrol site coordinates (route, docks, safe zones) loaded from config/site.yaml."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

SPIN = 'spin'
DIRECTIONS = ('NORTH', 'NORTH_WEST', 'WEST', 'SOUTH_WEST',
              'SOUTH', 'SOUTH_EAST', 'EAST', 'NORTH_EAST')


@dataclass(frozen=True)
class Site:
    """Coordinates of one patrol site in the map frame."""

    route: tuple          # (waypoint_id, (x, y), direction name) or SPIN, in order
    docks: dict           # robot_id -> ((x, y), direction name)
    initial_poses: dict   # robot_id -> ((x, y), direction name)
    safe_zones: dict      # zone id -> (x, y, yaw degrees)


def default_site_file():
    """Return the installed config/site.yaml, or the one in the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('patrol_amr_safety')) / 'config'
        if (installed / 'site.yaml').exists():
            return installed / 'site.yaml'
    except Exception:  # noqa: B902 - not installed (unit tests from the source tree)
        pass
    return Path(__file__).resolve().parents[1] / 'config' / 'site.yaml'


def _xy(entry, where):
    x, y = float(entry['x']), float(entry['y'])
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError(f'{where}: coordinates must be finite')
    return x, y


def _direction(name, where):
    name = str(name).upper()
    if name not in DIRECTIONS:
        raise ValueError(f'{where}: unknown direction {name!r}')
    return name


def load_site(path=None):
    """Read and check a site file; raise ValueError when it is incomplete."""
    data = yaml.safe_load(Path(path or default_site_file()).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('site file must be a mapping')
    route = []
    for index, wp in enumerate(data.get('route') or []):
        where = f'route[{index}]'
        route.append((str(wp['id']), _xy(wp, where), _direction(wp['dir'], where)))
        route.append(SPIN)
    if not route:
        raise ValueError('route must have at least one waypoint')
    docks, initial = {}, {}
    for robot, dock in (data.get('docks') or {}).items():
        where = f'docks.{robot}'
        xy = _xy(dock, where)
        docks[str(robot)] = (xy, _direction(dock['dir'], where))
        initial[str(robot)] = (xy, _direction(dock['initial_dir'], where))
    if not docks:
        raise ValueError('docks must list at least one robot')
    zones = {}
    for zone, values in (data.get('safe_zones') or {}).items():
        x, y, yaw = (float(v) for v in values)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError(f'safe_zones.{zone}: values must be finite')
        zones[zone] = (x, y, yaw)
    if not zones:
        raise ValueError('safe_zones must list at least one zone')
    return Site(tuple(route), docks, initial, zones)
