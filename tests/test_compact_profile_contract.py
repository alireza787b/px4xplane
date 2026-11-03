"""The compact-model migration must not masquerade as the legacy profile."""
import configparser
from pathlib import Path
import re
import unittest

ROOT=Path(__file__).resolve().parents[1]


class CompactProfileContract(unittest.TestCase):
    def defaults(self):
        text=(ROOT/'config/px4_params/5021_xplane_qtailsitter').read_text()
        entries=re.findall(r'^param set-default ([A-Z][A-Z0-9_]*) (\S+)$',text,re.M)
        self.assertEqual(len(entries),len(dict(entries)))
        self.assertEqual(len(entries),211)
        return {k:float(v) for k,v in entries}

    def test_geometry_and_mode_specific_damping(self):
        p=self.defaults()
        self.assertEqual(p['CA_ROTOR_COUNT'],4)
        for i in range(4):
            self.assertAlmostEqual(abs(p[f'CA_ROTOR{i}_PX']),.15)
            self.assertAlmostEqual(abs(p[f'CA_ROTOR{i}_PY']),.30)
            self.assertAlmostEqual(p[f'CA_ROTOR{i}_PZ'],-.045)
            self.assertGreater(abs(p[f'CA_ROTOR{i}_CT']*p[f'CA_ROTOR{i}_KM']),.05)
        self.assertEqual(p['FW_RR_P'],.26)
        self.assertEqual(p['MC_PITCHRATE_P'],.16)
        self.assertEqual(p['FW_PR_P'],.133333)
        self.assertEqual(p['FW_T_I_GAIN_PIT'],.10)
        self.assertEqual(p['FW_THR_TRIM'],.18)
        self.assertEqual(p['FW_ARSP_SCALE_EN'],0)
        self.assertEqual(p['FW_USE_AIRSPD'],1)

    def test_speed_bounds_and_known_exceptions_are_explicit(self):
        p=self.defaults()
        for request,bound in [('MPC_XY_CRUISE','MPC_XY_VEL_MAX'),
                              ('MPC_Z_V_AUTO_UP','MPC_Z_VEL_MAX_UP'),
                              ('MPC_Z_V_AUTO_DN','MPC_Z_VEL_MAX_DN')]:
            self.assertLessEqual(p[request],p[bound])
        self.assertEqual(p['VT_B_TRANS_DUR'],15)
        self.assertEqual(p['FW_R_TC'],1.5)
        self.assertNotIn('CAL_BARO0_OFF',p)
        self.assertNotIn('CAL_ACC0_ID',p)
        self.assertIn('exceeds metadata',
                      (ROOT/'config/px4_params/5021_xplane_qtailsitter').read_text())

    def test_bridge_mapping_unchanged_and_private_asset_not_packaged(self):
        ini=configparser.ConfigParser(interpolation=None)
        ini.read_string('[global]\n'+(ROOT/'config/config.ini').read_text())
        q=ini['QuadTailsitter']
        self.assertEqual(q['aircraftMatch'],'QuadTailsitter,PB50')
        self.assertEqual(q['airspeedSource'],'body_axis')
        self.assertEqual(q['pitotAxisBody'],'-Z')
        self.assertEqual(float(q['actuatorSmoothingTimeConstantSec']),0)
        for i in range(4):
            self.assertIn(f'[{i}]',q[f'channel{i}'])
        self.assertFalse((ROOT/'aircraft/NewQuadTailsitter').exists())
        cmake=(ROOT/'CMakeLists.txt').read_text()
        self.assertNotRegex(cmake,r'copy_directory\s+\$\{CMAKE_SOURCE_DIR\}/aircraft/QuadTailsitter')

    def test_public_identity_stays_quadtailsitter(self):
        profile=(ROOT/'config/px4_params/5021_xplane_qtailsitter').read_text()
        self.assertIn('@name              QuadTailsitter X-Plane SITL',profile)
        for relative in ('config/px4_params/5021_xplane_qtailsitter',
                         'config/config.ini', 'config/px4_params/README.md',
                         'docs/compact-qtailsitter.md',
                         'docs/QUADTAILSITTER_XPLANE12_TEST.md',
                         'aircraft/QuadTailsitter/README.md'):
            with self.subTest(path=relative):
                self.assertNotIn('NewQuadTailsitter',(ROOT/relative).read_text())


if __name__=='__main__':
    unittest.main()
