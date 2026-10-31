"""
Docking success rule (CR 9/9 도킹·충전 확인), adapted to the real topic rates.

Docked means all of:
- `dock_status.is_docked` has been true continuously for DOCKED_HOLD seconds,
- a `battery_state` message received *after* that docking moment reports the
  battery present and CHARGING or FULL.

The Create 3 publishes battery_state only about every 10 s, so the CR's
"both inputs at most 1 s old" cannot hold; instead the charging evidence must
come from a battery sample taken after the robot touched the dock. A Dock
action success or dock_visible alone never counts as docked.

No ROS dependency, so the rule is unit-testable without a running graph.
"""

DOCKED_HOLD = 2.0        # s, dock contact held continuously
DOCK_ATTEMPT_TIMEOUT = 60.0   # s, one docking attempt (CR: DOCKING_TIMEOUT 60 s)

# sensor_msgs/BatteryState power_supply_status
POWER_SUPPLY_STATUS_CHARGING = 1
POWER_SUPPLY_STATUS_FULL = 4


class DockCheck:
    """Combine dock_status and battery_state into one docked verdict."""

    def __init__(self):
        self.is_docked = None        # last dock_status value (None = never received)
        self._docked_since = None    # monotonic time contact started
        self._charging_after_dock = False
        self.percentage = None       # last battery percentage 0..1 (None = unknown)

    def observe_dock(self, is_docked: bool, now: float) -> None:
        self.is_docked = bool(is_docked)
        if not is_docked:
            self._docked_since = None
            self._charging_after_dock = False
        elif self._docked_since is None:
            self._docked_since = now
            self._charging_after_dock = False

    def observe_battery(self, present: bool, power_supply_status: int,
                        percentage: float, now: float) -> None:
        if percentage == percentage and 0.0 <= percentage <= 1.0:   # not NaN
            self.percentage = float(percentage)
        else:
            self.percentage = None
        if self._docked_since is None or now < self._docked_since:
            return
        self._charging_after_dock = bool(present) and power_supply_status in (
            POWER_SUPPLY_STATUS_CHARGING, POWER_SUPPLY_STATUS_FULL)

    def docked(self, now: float) -> bool:
        return (self._docked_since is not None
                and now - self._docked_since >= DOCKED_HOLD
                and self._charging_after_dock)

    def undocked(self) -> bool:
        return self.is_docked is False
