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


if __name__ == "__main__":
    unittest.main()
