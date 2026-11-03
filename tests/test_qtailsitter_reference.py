import hashlib
import math
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import build_qtailsitter_reference as reference
import package_qtailsitter_reference as packaging
from analyze_qtailsitter_design import parse_acf, estimate_wing_area_m2, as_float


class ReferenceTest(unittest.TestCase):
    source = Path(__file__).resolve().parents[1] / "aircraft/QuadTailsitter/QuadTailsitter.acf"

    def test_source_preserved_and_candidate_consistent(self):
        before = hashlib.sha256(self.source.read_bytes()).hexdigest()
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            doc = reference.build(self.source, out)
            candidate = parse_acf(out / "aircraft/QuadTailsitterRef6/QuadTailsitterRef6.acf")
            self.assertAlmostEqual(as_float(candidate, "acf/_m_empty") * reference.LB_TO_KG, 2)
            self.assertAlmostEqual(estimate_wing_area_m2(candidate)["main_panel_area_m2"], reference.sizing.AREA, places=7)
            self.assertGreater(doc["minimum_planar_prop_clearance_m"], .04)
            self.assertAlmostEqual(as_float(candidate, 'acf/_cgY') * reference.FT_TO_M, reference.sizing.CG_Y)
            self.assertEqual(as_float(candidate, 'acf/_fixed_max/0'), 0)
            for i in (8, 9, 10, 11):
                self.assertEqual(as_float(candidate, f'_wing/{i}/_semilen_SEG'), 0)
            self.assertLess(doc["estimated_electrical_budget_w"], doc["battery_nominal_budget_w"])
            self.assertFalse(doc["flight_validated"])
            self.assertEqual(doc['acf_sha256'], 'c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918',
                             'Preserve the recorded R6/R7 aircraft bytes')
            self.assertTrue((out / "aircraft/QuadTailsitterRef6/Airfoil/NACA 2412 (popular).afl").is_file())
            for i in range(4):
                self.assertEqual(as_float(candidate, f"_prop/{i}/_num_blades"), 2)
                self.assertAlmostEqual(as_float(candidate, f"_blad/{i}/_semilen_SEG") * reference.FT_TO_M * 2, .254)
                original = parse_acf(self.source)
                for group in ('_engn', '_blad'):
                    for axis in ('phi','psi','the'):
                        key=f'{group}/{i}/_part_{axis}'
                        if key in original:
                            self.assertEqual(candidate[key],original[key],key)
                self.assertEqual(candidate[f'_prop/{i}/_prop_dir'], original[f'_prop/{i}/_prop_dir'])
                self.assertEqual(as_float(candidate, f"_engn/{i}/_RSC_redline_ENGN"), 13000)
                self.assertAlmostEqual(as_float(candidate, f'_engn/{i}/_power_max_limit') * 745.699872, 500, places=5)
                self.assertAlmostEqual(as_float(candidate, f'_wing/{i}/_Croot') * reference.FT_TO_M, .1)
                mesh_x = [abs(float(v)) * reference.FT_TO_M for k, v in candidate.items()
                          if k.startswith(f'_wing/{i}/_geo_xyz/') and k.endswith(',0')]
                self.assertAlmostEqual(max(mesh_x)*2, .6, places=6)
                self.assertEqual(candidate[f'_wing/{i}/_geo_xyz/i_count'], '20')
                for rib in range(11):
                    self.assertAlmostEqual(as_float(candidate, f"_wing/{i}/_chord_for_RIB/{rib}"),
                                           as_float(candidate, f"_wing/{i}/_Croot"))
                self.assertAlmostEqual(as_float(candidate, f"_blad/{i}/_chord_for_RIB/0"),
                                       as_float(candidate, f"_blad/{i}/_Croot"))
            # Stored visual blade sections must rotate in the same direction
            # as their physical incidence, not the opposite direction.
            original = parse_acf(self.source)
            for key, value in original.items():
                if key.endswith(('_count', '/count')):
                    self.assertEqual(candidate[key], value, key)
            def angle(v, station):
                p = f"_blad/0/_geo_xyz/{station},"
                return math.atan2(as_float(v, p+'5,2') - as_float(v, p+'0,2'),
                                  -(as_float(v, p+'5,1') - as_float(v, p+'0,1')))
            offset = angle(original, 0) + math.radians(45)
            for station in (0, 6, 10, 18, 19):
                radius = max(.2 * .127, as_float(candidate, f"_blad/0/_geo_xyz/{station},0,0") * reference.FT_TO_M)
                expected = offset - math.atan2(.2032, 2 * math.pi * radius)
                self.assertAlmostEqual(angle(candidate, station), expected, places=5)
            with self.assertRaises(FileExistsError):
                reference.build(self.source, out)
        self.assertEqual(before, hashlib.sha256(self.source.read_bytes()).hexdigest())

    def test_fails_closed_on_incompatible_source(self):
        with self.assertRaises(ValueError):
            reference.rewrite("P known 1\n", {"missing": "2"})
        with self.assertRaises(ValueError):
            reference.rewrite("P known 1\nP known 2\n", {"known": "2"})
        with self.assertRaises(ValueError):
            reference.number(float("nan"))

    def test_rotation_and_allocation_pairing(self):
        with tempfile.TemporaryDirectory() as tmp:
            reference.build(self.source, Path(tmp))
            params, _ = reference.parameter_profile(parse_acf(Path(tmp) / 'aircraft/QuadTailsitterRef6/QuadTailsitterRef6.acf'))
        self.assertEqual(sum(params[f"CA_ROTOR{i}_KM"] for i in range(4)), 0)
        self.assertEqual(params['MPC_THR_HOVER'], .26)
        self.assertEqual(params['MPC_JERK_AUTO'], 4)
        self.assertEqual(params['MPC_ACC_HOR'], 2)
        self.assertEqual(params['MPC_ACC_HOR_MAX'], 2)
        self.assertEqual(params['MC_PITCHRATE_P'], .16)
        self.assertEqual(params['MC_PITCHRATE_I'], .015)
        self.assertEqual(params['MC_PITCHRATE_D'], .0012)
        self.assertGreaterEqual(params['MC_PITCHRATE_P'], .01)
        self.assertEqual(params['MC_PITCHRATE_K'], 1)
        self.assertEqual(params['MC_ROLLRATE_P'], .060)
        self.assertEqual(params['MC_ROLLRATE_I'], .007954)
        self.assertEqual(params['MC_ROLLRATE_D'], .000636)
        self.assertEqual(params['FW_ARSP_SCALE_EN'], 0)
        self.assertEqual(params['FW_USE_AIRSPD'], 1)
        self.assertEqual(params['FW_THR_MIN'], .08)
        self.assertEqual(params['VT_B_TRANS_DUR'], 15)
        for fw,mc,scale in (('PR','PITCH',.90),):
            for term in ('I','D'):
                self.assertAlmostEqual(params[f'FW_{fw}_{term}']*scale,
                                       params[f'MC_{mc}RATE_{term}']*params[f'MC_{mc}RATE_K'],places=6)
        self.assertEqual(params['FW_RR_P'], .26)
        self.assertEqual(params['FW_PR_P'], .133333,
                         'A recovery MC pitch change must not also change FW pitch P')
        self.assertEqual(params['FW_T_I_GAIN_PIT'], .10)
        self.assertEqual(params['FW_THR_TRIM'], .18)
        self.assertNotIn('FW_R_TC', params, 'Do not bundle a 50% outer-loop gain change')
        self.assertNotIn('FW_T_STE_R_TC', params, 'Retain the existing throttle feedback filter')
        self.assertEqual(params['MPC_XY_CRUISE'], 5)
        self.assertEqual(params['MPC_XY_VEL_MAX'], 6)
        self.assertEqual(params['MPC_VEL_MANUAL'], 5)
        self.assertEqual(params['MPC_Z_V_AUTO_UP'], 2.5)
        self.assertEqual(params['MPC_Z_VEL_MAX_UP'], 3)
        self.assertEqual(params['MPC_Z_V_AUTO_DN'], 1.6)
        self.assertEqual(params['MPC_Z_VEL_MAX_DN'], 1.8)
        self.assertEqual(params['MPC_TKO_SPEED'], 1.5)
        self.assertLessEqual(params['MPC_XY_CRUISE'], params['MPC_XY_VEL_MAX'])
        self.assertLessEqual(params['MPC_Z_V_AUTO_UP'], params['MPC_Z_VEL_MAX_UP'])
        self.assertLessEqual(params['MPC_Z_V_AUTO_DN'], params['MPC_Z_VEL_MAX_DN'])
        order = list(params)
        for bound, request in (('MPC_XY_VEL_MAX', 'MPC_XY_CRUISE'),
                               ('MPC_Z_VEL_MAX_UP', 'MPC_Z_V_AUTO_UP'),
                               ('MPC_Z_VEL_MAX_DN', 'MPC_Z_V_AUTO_DN')):
            self.assertLess(order.index(bound), order.index(request),
                            'Apply the new bound before its request to avoid runtime clamping')
        self.assertEqual(params['MC_ROLL_P'], 1.4)
        self.assertEqual(params['MC_PITCH_P'], 1.0)
        self.assertEqual(params['FW_RR_I'], .014318)
        self.assertEqual(params['FW_RR_D'], .001507)
        # Changing MC roll must not silently retune a different mode/axis.
        self.assertEqual(tuple(params['FW_YR_'+term] for term in ('P','I','D')),
                         (.099422,.024856,.001987))
        self.assertLess(params['MC_YAWRATE_P'], .105262)
        original_axes=((- .078,.040,-.996),(.078,-.040,-.996),(-.078,-.040,-.996),(.078,.040,-.996))
        for i in range(4):
            for axis,value in zip(('AX','AY','AZ'),original_axes[i]):
                self.assertEqual(params[f'CA_ROTOR{i}_{axis}'], value)
            self.assertEqual(params[f"CA_ROTOR{i}_CT"], 6.5)
            self.assertGreater(abs(params[f"CA_ROTOR{i}_CT"] * params[f"CA_ROTOR{i}_KM"]), .05)
            self.assertEqual(params[f"CA_ROTOR{i}_AZ"], -.996)
            self.assertAlmostEqual(abs(params[f"CA_ROTOR{i}_PX"]), .15, places=6)
            self.assertAlmostEqual(abs(params[f"CA_ROTOR{i}_PY"]), .3, places=6)

    def test_package_manifest_and_overwrite_refusal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output, archive, digest = packaging.package(root / "output", root / "archives")
            self.assertEqual(hashlib.sha256(archive.read_bytes()).hexdigest(), digest)
            self.assertIn("ref6.py", (output / "SHA256SUMS").read_text())
            with zipfile.ZipFile(archive) as zipped:
                self.assertIn(packaging.BUNDLE_NAME + "/QUICKSTART.md", zipped.namelist())
                self.assertFalse(any("__pycache__" in name or ".ref6.lock" in name for name in zipped.namelist()))
            with self.assertRaises(FileExistsError):
                packaging.package(root / "output", root / "archives")


if __name__ == "__main__":
    unittest.main()
