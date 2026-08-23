"""
Unit-test session_dir.resolve_session_dir().

No ROS import anywhere in this file or in session_dir.py, so this runs with
plain python3 on the host -- no container, no colcon. See session_dir.py's
module docstring.

Run directly:
    python3 ros_env/src/bizon_player_bringup/launch/test_session_dir.py
or, inside the container, via colcon test (ament_cmake_pytest, see
bizon_player_bringup/CMakeLists.txt).
"""
import os
import sys
import tempfile
import unittest
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from session_dir import resolve_session_dir  # noqa: E402


class ResolveSessionDirTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.debug_root = self._tmp.name
        self.now = datetime(2026, 8, 23, 15, 43, 2)

    def tearDown(self):
        self._tmp.cleanup()

    def test_new_creates_a_fresh_timestamped_directory(self):
        session_dir = resolve_session_dir(self.debug_root, "bizon2", "new", now=self.now)
        self.assertEqual(
            session_dir,
            os.path.join(self.debug_root, "debug", "bizon2", "game_2026-08-23_15-43-02"),
        )
        self.assertTrue(os.path.isdir(session_dir))

    def test_new_twice_creates_two_distinct_directories(self):
        first = resolve_session_dir(self.debug_root, "bizon2", "new", now=self.now)
        later = datetime(2026, 8, 23, 15, 44, 0)
        second = resolve_session_dir(self.debug_root, "bizon2", "new", now=later)
        self.assertNotEqual(first, second)
        self.assertTrue(os.path.isdir(first))
        self.assertTrue(os.path.isdir(second))

    def test_last_with_no_existing_games_falls_back_to_creating_one(self):
        session_dir = resolve_session_dir(self.debug_root, "bizon2", "last", now=self.now)
        self.assertEqual(
            session_dir,
            os.path.join(self.debug_root, "debug", "bizon2", "game_2026-08-23_15-43-02"),
        )
        self.assertTrue(os.path.isdir(session_dir))

    def test_last_reuses_the_newest_existing_game_directory(self):
        older = resolve_session_dir(
            self.debug_root, "bizon2", "new", now=datetime(2026, 8, 23, 10, 0, 0))
        newer = resolve_session_dir(
            self.debug_root, "bizon2", "new", now=datetime(2026, 8, 23, 12, 0, 0))

        resumed = resolve_session_dir(self.debug_root, "bizon2", "last", now=self.now)

        self.assertEqual(resumed, newer)
        self.assertNotEqual(resumed, older)

    def test_prefixes_are_isolated_from_each_other(self):
        white = resolve_session_dir(self.debug_root, "bizon2", "new", now=self.now)
        black = resolve_session_dir(self.debug_root, "bizon3", "new", now=self.now)
        self.assertNotEqual(white, black)
        self.assertIn(os.path.join("debug", "bizon2"), white)
        self.assertIn(os.path.join("debug", "bizon3"), black)

    def test_invalid_game_value_raises(self):
        with self.assertRaises(ValueError):
            resolve_session_dir(self.debug_root, "bizon2", "bogus", now=self.now)


if __name__ == "__main__":
    unittest.main()
