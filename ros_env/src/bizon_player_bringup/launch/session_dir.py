"""
Resolve the per-game debug session directory for one robot.

Deliberately free of any `launch`/`launch_ros`/ROS import: bizon_player.launch.py
imports resolve_session_dir() from here, but this module is also imported
directly by test_session_dir.py so the resolution logic -- `game:=new` vs
`game:=last`, an empty debug/<prefix>/ vs one with existing games -- can be
unit-tested with plain python3 on the host, where there is no ROS install to
even import `launch` (see CLAUDE.md: "There is no ROS on the host").
"""

import os
from datetime import datetime
from typing import Optional

GAME_DIR_PREFIX = "game_"
GAME_DIR_TIMESTAMP_FORMAT = "%Y-%m-%d_%H-%M-%S"


def _existing_game_dirs(robot_debug_dir: str) -> list:
    if not os.path.isdir(robot_debug_dir):
        return []
    return sorted(
        name for name in os.listdir(robot_debug_dir)
        if name.startswith(GAME_DIR_PREFIX)
        and os.path.isdir(os.path.join(robot_debug_dir, name))
    )


def resolve_session_dir(
    debug_root: str,
    prefix: str,
    game: str,
    now: Optional[datetime] = None,
) -> str:
    """
    Return the absolute path of the session directory to use.

    `game == 'new'` always creates debug/<prefix>/game_<timestamp>/.
    `game == 'last'` reuses the lexicographically-newest existing
    debug/<prefix>/game_*/ directory (the timestamp format sorts
    chronologically), falling back to creating a new one if none exists --
    matching the spec's "falls back to creating one if none exists".

    `now` is injectable so the timestamp in a freshly-created directory name
    is deterministic in tests; production callers leave it as None and get
    datetime.now().
    """
    if game not in ("new", "last"):
        raise ValueError(f"game must be 'new' or 'last', got {game!r}")

    robot_debug_dir = os.path.join(debug_root, "debug", prefix)
    os.makedirs(robot_debug_dir, exist_ok=True)

    if game == "last":
        existing = _existing_game_dirs(robot_debug_dir)
        if existing:
            return os.path.join(robot_debug_dir, existing[-1])
        # No existing game to resume -- fall back to creating one, same as
        # 'new'.

    timestamp = (now or datetime.now()).strftime(GAME_DIR_TIMESTAMP_FORMAT)
    session_dir = os.path.join(robot_debug_dir, f"{GAME_DIR_PREFIX}{timestamp}")
    os.makedirs(session_dir, exist_ok=True)
    return session_dir
