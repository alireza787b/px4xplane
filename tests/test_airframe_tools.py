import csv
import gzip
import math
import zipfile
from unittest.mock import patch
import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import analyze_qtailsitter_design as design
import check_px4_airframe_params as params


class AirframeToolTest(unittest.TestCase):
    def test_failed_zip_member_open_closes_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory)/'capture.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('frames.csv', 'frame_id,time\n1,1\n')
            reader = design.CaptureFrames(archive)
            with patch.object(zipfile.ZipFile, 'open', side_effect=zipfile.BadZipFile('damaged member')):
                with self.assertRaises(zipfile.BadZipFile):
                    with reader:
                        pass
            self.assertIsNone(reader._zip.fp)

    def test_capture_reader_streams_gzip_in_directory_and_zip(self):
        text = '\ufeffframe_id,sim_time_s,value\n1,1.0,kept\n2,1.1,complete\n'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            packed = root/'frames.csv.gz'
            packed.write_bytes(gzip.compress(text.encode()))
            with design.CaptureFrames(root) as stream:
                self.assertEqual(list(csv.DictReader(stream))[-1]['value'], 'complete')
            archive = root/'capture.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                z.write(packed, 'flight/truth/frames.csv.gz')
            with design.CaptureFrames(archive) as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 2)
            (root/'frames.csv').write_text(text)
            with self.assertRaisesRegex(FileNotFoundError, 'Expected one'):
                with design.CaptureFrames(root):
                    pass

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
        self.assertAlmostEqual(aircraft["wing"]["selected_area_m2"], 0.2812, places=4)

    def test_panel_orientation_does_not_duplicate_area(self):
        values = {"_wing/count": "2"}
        for index, orientation in enumerate((-1, 1)):
            for name, value in (("_semilen_SEG", 1), ("_Croot", 1),
                                ("_Ctip", 1), ("_is_right_mult", orientation)):
                values[f"_wing/{index}/{name}"] = str(value)
        area = design.estimate_wing_area_m2(values)
        self.assertAlmostEqual(area["panel_area_m2"], 2 * 0.3048 ** 2)
        self.assertEqual(area["panel_area_m2"], area["mirrored_area_m2"])

    def test_truth_uses_physical_prop_speed_not_governor_command(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            row = {"sim/flightmodel2/engines/prop_rotation_speed_rad_sec": "1000;1000;1000;1000",
                   "sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec": "9999;9999;9999;9999",
                   "sim/flightmodel/engine/ENGN_thro_use": "0.2;0.3;0.4;0.5"}
            with (path / "frames.csv").open("w") as output:
                writer = csv.DictWriter(output, fieldnames=list(row))
                writer.writeheader()
                writer.writerow(row)
            result = design.truth_summary(path)
        self.assertAlmostEqual(result["max_prop_rpm"]["p50"], 1000 * 60 / (2 * math.pi))
        self.assertEqual(result["max_throttle_used"]["p50"], .5)


if __name__ == "__main__":
    unittest.main()
