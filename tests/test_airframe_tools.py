import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import analyze_qtailsitter_design as design
import check_px4_airframe_params as params


class AirframeToolTest(unittest.TestCase):
    def test_parse_airframe_defaults_and_compare(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "airframe"
            path.write_text(
                "param set-default SYS_AUTOSTART 5021\n"
                "param set-default MC_PITCH_P 0.50 # comment\n",
                encoding="utf-8",
            )
            expected = params.parse_airframe_defaults(path)

        self.assertEqual(expected["SYS_AUTOSTART"], "5021")
        matches, mismatches = params.compare_parameters(
            expected,
            {"SYS_AUTOSTART": 5021, "MC_PITCH_P": 0.50001},
            None,
            abs_tol=1e-3,
            rel_tol=1e-3,
        )
        self.assertEqual(len(matches), 2)
        self.assertEqual(mismatches, [])

        _, mismatches = params.compare_parameters(
            expected,
            {"SYS_AUTOSTART": 5020, "MC_PITCH_P": 0.9},
            ["SYS_AUTOSTART", "MC_PITCH_P"],
            abs_tol=1e-4,
            rel_tol=1e-4,
        )
        self.assertEqual({item["param"] for item in mismatches}, {"SYS_AUTOSTART", "MC_PITCH_P"})

    def test_qtailsitter_design_summary(self):
        acf = Path(__file__).resolve().parents[1] / "aircraft" / "QuadTailsitter" / "QuadTailsitter.acf"
        result = design.analyze(acf, None, 1.2)
        aircraft = result["aircraft"]

        self.assertGreater(aircraft["mass"]["empty_kg"], 4.0)
        self.assertLess(aircraft["mass"]["empty_kg"], 6.0)
        self.assertGreater(aircraft["wing"]["selected_area_m2"], 0.1)
        self.assertGreater(aircraft["propulsion"]["hover_per_motor_thrust_n"], 10.0)
        self.assertGreater(aircraft["propulsion"]["props"][0]["diameter_in"], 9.0)


if __name__ == "__main__":
    unittest.main()
