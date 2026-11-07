"""Keep accepted Alia tuning separate from matching official reference data."""
import configparser
import hashlib
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from check_px4_airframe_params import parse_airframe_defaults


class AliaProfileContractTest(unittest.TestCase):
    def test_official_reference_values_match_reviewed_pin(self):
        expected = {
            '5010_xplane_ehang184': '3d6ff2a3b172ee2264cd0a0c809592bc41b5b4ca497cc2f370211bab4148bde4',
            '5020_xplane_alia250': '545a3ca21f7b8eec4bd24408442c3a78f197c0275290b103536c6bee6d67877e',
        }
        # Numeric maps were checked against PX4 95a0c2348232; comments may improve.
        for name, digest in expected.items():
            with self.subTest(airframe=name):
                values = {k: float(v) for k, v in parse_airframe_defaults(
                    ROOT / 'config/px4_params' / name).items()}
                payload = json.dumps(values, sort_keys=True, separators=(',', ':')).encode()
                self.assertEqual(hashlib.sha256(payload).hexdigest(), digest)

    def test_candidate_changes_only_the_eight_flown_values(self):
        name = '5020_xplane_alia250'
        baseline = {k: float(v) for k, v in parse_airframe_defaults(
            ROOT / 'config/px4_params' / name).items()}
        candidate = {k: float(v) for k, v in parse_airframe_defaults(
            ROOT / 'config/experimental/alia250' / name).items()}
        changes = {'WV_YRATE_MAX': 10, 'FW_P_TC': 1, 'VT_B_DEC_I': .08,
                   'VT_B_TRANS_DUR': 40, 'VT_ARSP_TRANS': 48, 'FW_RR_FF': 1.5,
                   'CA_SV_CS0_TRIM': -.3333333333, 'CA_SV_CS1_TRIM': -.3333333333}
        self.assertEqual(candidate, {**baseline, **changes})
        self.assertNotIn('SDLOG_MODE', candidate)
        self.assertEqual(candidate['MIS_TAKEOFF_ALT'], baseline['MIS_TAKEOFF_ALT'])

    def test_mapping_preserved_and_neutral_requires_verified_reversal(self):
        ini = configparser.ConfigParser(interpolation=None, strict=True)
        ini.optionxform = str
        ini.read_string('[GLOBAL]\n' + (ROOT / 'config/config.ini').read_text())
        alia = ini['Alia250']
        self.assertEqual(ini['GLOBAL']['config_name'], 'Alia250')
        self.assertFalse(alia['autoPropBrakes'].strip())
        for channel in ('channel4', 'channel5'):
            self.assertEqual(alia[channel].count('[-20 10]'), 3)
        candidate = parse_airframe_defaults(ROOT / 'config/experimental/alia250/5020_xplane_alia250')
        self.assertEqual(candidate['PWM_MAIN_REV'], '112')
        self.assertEqual(candidate['PWM_MAIN_FUNC5'], '201')
        self.assertEqual(candidate['PWM_MAIN_FUNC6'], '202')
        for i in (0, 1):
            servo = float(candidate[f'CA_SV_CS{i}_TRIM'])
            self.assertAlmostEqual(-5 - 15 * servo, 0, places=7)

    def test_packaging_does_not_promote_experimental_profiles(self):
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        self.assertIn('config/px4_params', cmake)
        self.assertNotIn('config/experimental/alia250', cmake)
        guide = (ROOT / 'docs/alia250-profile.md').read_text()
        for limitation in ('30 m/s', '20 s', 'not a full lap', '45 m', 'cruise reserve'):
            self.assertIn(limitation, guide)


if __name__ == '__main__':
    unittest.main()
