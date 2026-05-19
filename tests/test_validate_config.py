#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import validate_config


class ValidateConfigTest(unittest.TestCase):
    def test_project_config_is_valid(self):
        issues = validate_config.validate_config(Path("config/config.ini"))
        errors = [issue for issue in issues if issue.level == "error"]
        self.assertEqual(errors, [])

    def test_detects_missing_active_section_and_bad_channel(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "config.ini"
            path.write_text(
                "\n".join(
                    [
                        "config_name = Missing",
                        "",
                        "[Aircraft]",
                        "channel0 = , intArray, [0], [0 1]",
                        "autoPropBrakes = 0, 9, bad",
                        "autoPropBrakeApplyThreshold = 0.20",
                        "autoPropBrakeReleaseThreshold = 0.10",
                        "autoPropBrakeMode = broken",
                    ]
                ),
                encoding="utf-8",
            )

            messages = [str(issue) for issue in validate_config.validate_config(path)]

        self.assertTrue(any("active config 'Missing'" in message for message in messages))
        self.assertTrue(any("empty dataref" in message for message in messages))
        self.assertTrue(any("unsupported type 'intArray'" in message for message in messages))
        self.assertTrue(any("motor index 9 outside" in message for message in messages))
        self.assertTrue(any("invalid motor index 'bad'" in message for message in messages))
        self.assertTrue(any("release threshold must be greater" in message for message in messages))
        self.assertTrue(any("expected one of: feather, hard_lock, prop_separate" in message for message in messages))

    def test_detects_float_array_without_index_and_bad_range(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "config.ini"
            path.write_text(
                "\n".join(
                    [
                        "config_name = Aircraft",
                        "",
                        "[Aircraft]",
                        "channel0 = sim/test/ref, floatArray, 0, [1 1]",
                    ]
                ),
                encoding="utf-8",
            )

            messages = [str(issue) for issue in validate_config.validate_config(path)]

        self.assertTrue(any("floatArray mapping must specify" in message for message in messages))
        self.assertTrue(any("invalid range" in message for message in messages))

    def test_schema_validates_global_types_and_ranges(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "config.ini"
            path.write_text(
                "\n".join(
                    [
                        "config_name = Aircraft",
                        "diagnostic_log_enabled = maybe",
                        "fps_warning_threshold = 5",
                        "mavlink_sensor_rate_hz = 900",
                        "unexpected_key = true",
                        "",
                        "[Aircraft]",
                        "channel0 = sim/test/ref, float, 0, [0 1]",
                    ]
                ),
                encoding="utf-8",
            )

            messages = [str(issue) for issue in validate_config.validate_config(path)]

        self.assertTrue(any("expected boolean" in message for message in messages))
        self.assertTrue(any("below minimum 20" in message for message in messages))
        self.assertTrue(any("above maximum 500" in message for message in messages))
        self.assertTrue(any("unknown global config key" in message for message in messages))

    def test_schema_field_listing_is_available(self):
        schema = validate_config.load_schema()
        self.assertIn("global_fields", schema)
        self.assertIn("config_name", schema["global_fields"])
        self.assertEqual(schema["global_fields"]["config_name"]["reload_policy"], "reconnect_before_flight")


if __name__ == "__main__":
    unittest.main()
