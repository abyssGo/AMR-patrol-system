"""
Persist the control state that must survive a control node restart.

Fire hold must only be cleared by the operator (scenarios.md 4장), the EStop
state and sequence must not reset, and excluded robots must stay excluded.
The state is a small JSON file written atomically on every change.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

STATE_PATH = Path.home() / '.ros' / 'patrol_control' / 'state.json'


def load_state(path: Path | None = None) -> dict:
    """Return the saved snapshot, or {} when there is none or it is unreadable."""
    path = path or STATE_PATH
    try:
        with open(path, encoding='utf-8') as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def save_state(data: dict, path: Path | None = None) -> None:
    """Write the snapshot atomically (temporary file + rename)."""
    path = path or STATE_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix='.state-', suffix='.json')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            json.dump(data, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
