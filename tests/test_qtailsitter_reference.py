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
            self.assertAlmostEqual(as_float(candidate, "acf/_m_empty") * reference.LB_TO_KG, 6)
            self.assertAlmostEqual(estimate_wing_area_m2(candidate)["main_panel_area_m2"], .331, places=7)
            self.assertGreater(doc["minimum_planar_prop_clearance_m"], .18)
            self.assertLess(doc["estimated_electrical_budget_w"], doc["battery_nominal_budget_w"])
            self.assertFalse(doc["flight_validated"])
            self.assertTrue((out / "aircraft/QuadTailsitterRef6/Airfoil/NACA 2412 (popular).afl").is_file())
            for i in range(4):
                self.assertEqual(as_float(candidate, f"_prop/{i}/_num_blades"), 2)
                self.assertAlmostEqual(as_float(candidate, f"_blad/{i}/_semilen_SEG") * reference.FT_TO_M * 2, .254)
                self.assertEqual(as_float(candidate, f"_engn/{i}/_part_psi"), 0)
                self.assertEqual(as_float(candidate, f"_engn/{i}/_RSC_redline_ENGN"), 13000)
                for rib in range(11):
                    self.assertAlmostEqual(as_float(candidate, f"_wing/{i}/_chord_for_RIB/{rib}"),
                                           as_float(candidate, f"_wing/{i}/_Croot"))
                self.assertAlmostEqual(as_float(candidate, f"_blad/{i}/_chord_for_RIB/0"),
                                       as_float(candidate, f"_blad/{i}/_Croot"))
            # Stored visual blade sections must rotate in the same direction
            # as their physical incidence, not the opposite direction.
            original = parse_acf(self.source)
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
        params, _ = reference.parameter_profile(parse_acf(self.source))
        self.assertEqual(sum(params[f"CA_ROTOR{i}_KM"] for i in range(4)), 0)
        self.assertEqual(params['MPC_THR_HOVER'], .71)
        for i in range(4):
            self.assertEqual(params[f"CA_ROTOR{i}_CT"], 6.5)
            self.assertGreater(abs(params[f"CA_ROTOR{i}_CT"] * params[f"CA_ROTOR{i}_KM"]), .05)
            self.assertEqual(params[f"CA_ROTOR{i}_AZ"], -1)
            self.assertAlmostEqual(abs(params[f"CA_ROTOR{i}_PX"]), .219456, places=6)
            self.assertAlmostEqual(abs(params[f"CA_ROTOR{i}_PY"]), .42672, places=6)

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
