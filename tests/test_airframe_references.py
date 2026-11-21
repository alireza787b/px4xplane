"""Contracts for the integrated airframe files and actuator correction."""
import configparser
import importlib.util
from pathlib import Path
import re
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('references', ROOT / 'tools/check_airframe_references.py')
references = importlib.util.module_from_spec(spec)
spec.loader.exec_module(references)


def defaults(name):
    return dict((k, float(v)) for k, v in re.findall(
        r'^param set-default ([A-Z0-9_]+)\s+([^#\s]+)',
        (ROOT / 'config/px4_params' / name).read_text(), re.M))


class AirframeReferences(unittest.TestCase):
    def test_accepted_handling_values_are_not_mission_overrides(self):
        expected = {
            '5001_xplane_cessna172': {'FW_P_LIM_MIN': -5, 'FW_WR_P': 1},
            '5010_xplane_ehang184': {'MPC_THR_HOVER': .25, 'MPC_YAWRAUTO_MAX': 10,
                                    'MPC_YAWRAUTO_ACC': 10, 'MPC_AUTO_NUDGING': 0},
            '5020_xplane_alia250': {'WV_YRATE_MAX': 10, 'FW_P_TC': 1, 'VT_B_DEC_I': .08,
                                  'VT_B_TRANS_DUR': 40, 'VT_ARSP_TRANS': 48, 'FW_RR_FF': 1.5,
                                  'CA_SV_CS0_TRIM': -1/3, 'CA_SV_CS1_TRIM': -1/3,
                                  'MPC_AUTO_NUDGING': 0},
        }
        for name, values in expected.items():
            actual = defaults(name)
            for key, value in values.items():
                with self.subTest(profile=name, parameter=key):
                    self.assertAlmostEqual(actual[key], value, places=6)
        self.assertNotEqual(defaults('5001_xplane_cessna172')['NAV_LOITER_RAD'], 1000)
        self.assertNotEqual(defaults('5010_xplane_ehang184').get('MIS_TAKEOFF_ALT'), 30)

    def test_canonical_profiles_have_unique_defaults_and_current_parameter_names(self):
        for name in references.PROFILES:
            text = (ROOT / 'config/px4_params' / name).read_text()
            names = re.findall(r'^param set-default ([A-Z0-9_]+) ', text, re.M)
            self.assertEqual(len(names), len(set(names)), name)
            self.assertNotIn('MPC_LAND_RC_HELP', names)
            self.assertNotIn('FD_FAIL_R', names)

    def test_c172_elevator_drives_both_actual_surface_indices(self):
        ini = configparser.ConfigParser(interpolation=None)
        ini.read_string('[global]\n' + (ROOT / 'config/config.ini').read_text())
        self.assertEqual(ini['Cessna172']['channel2'],
                         'sim/flightmodel2/wing/elevator1_deg, floatArray, [8 9], [-20 +20]')
        self.assertEqual(set(k for k in ini['QuadTailsitter'] if k.startswith('channel')),
                         {'channel0', 'channel1', 'channel2', 'channel3'})

    def test_reference_checker_catches_source_drift_without_writing(self):
        with tempfile.TemporaryDirectory() as folder:
            px4 = Path(folder)
            source = px4 / 'ROMFS/px4fmu_common/init.d-posix/airframes'
            source.mkdir(parents=True)
            for name in references.PROFILES:
                (source / name).write_bytes((ROOT / 'config/px4_params' / name).read_bytes())
            self.assertTrue(all(r['matches_px4'] for r in references.check(px4, ROOT / 'config/px4_params')))
            (source / references.PROFILES[0]).write_text('changed\n')
            result = references.check(px4, ROOT / 'config/px4_params')
            self.assertFalse(result[0]['matches_px4'])
            self.assertEqual((source / references.PROFILES[0]).read_text(), 'changed\n')
