"""Smoother-only regression; no simulator flight or aircraft model."""
import math
from pathlib import Path
import shutil
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import replay_qtailsitter_braking as replay


class BrakingReplayTest(unittest.TestCase):
    def test_event_validation(self):
        self.assertRaises(ValueError, replay.releases, [])
        bad = dict(active=0, t=150, smooth_accel=[math.nan, 0, 0], smooth_velocity=[0, 0, 0])
        self.assertRaises(ValueError, replay.releases, [bad])
        event = dict(active=0, t=150, smooth_accel=[4.15, -1.28, -1.61],
                     smooth_velocity=[-.4714, -.1837, .4127])
        self.assertEqual(replay.releases([event])[0]['boot_s'], 150)

    def test_recorded_release_crosses_only_with_slow_jerk(self):
        repo = Path.home()/'PX4-Autopilot-Me'
        if not shutil.which('g++') or not (repo/'src/lib/matrix').is_dir():
            self.skipTest('Requires g++ and the existing PX4 headers')
        event = dict(boot_s=130.166620, acceleration=[4.1499095, -1.2808553, -1.6111968],
                     velocity=[-.4713933, -.1836896, .4127105])
        result = replay.replay(repo, [event])
        by_jerk = {row['jerk_mps3']: row for row in result['rows']}
        self.assertIsNotNone(by_jerk[.7]['first_5p2_crossing_s'])
        self.assertIsNone(by_jerk[4]['first_5p2_crossing_s'])
        self.assertGreater(by_jerk[.7]['peak_virtual_horizontal_speed_mps'], 5.2)
        self.assertLess(by_jerk[4]['peak_virtual_horizontal_speed_mps'], 3)
