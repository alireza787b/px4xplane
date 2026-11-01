import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import qtailsitter_sizing as sizing


class SizingTests(unittest.TestCase):
    def test_mass_cg_and_inertia(self):
        doc=sizing.screen()
        self.assertAlmostEqual(doc['mass_kg'],2)
        self.assertAlmostEqual(doc['cg_xplane_m'][1],sizing.CG_Y)
        self.assertAlmostEqual(doc['main_area_m2'],.1343226362)
        tensor=doc['inertia_tensor_xplane_kg_m2']
        for i in range(3):
            self.assertGreater(tensor[i][i],0)
            for j in range(3):self.assertAlmostEqual(tensor[i][j],tensor[j][i])

    def test_propulsion_bounded(self):
        for volts in (25.2,22.2,19.8):
            for rho in (1.225,1.059,1):
                p=sizing.propulsion(volts,rho)
                self.assertLessEqual(p['shaft_w'],500.00001)
                self.assertLessEqual(p['rpm'],13000)
                self.assertLessEqual(p['required_motor_voltage_v'],volts)

    def test_envelope_is_prediction_not_validation(self):
        doc=sizing.screen()
        self.assertFalse(doc['flight_validated'])
        selected=[p for p in doc['envelope'] if p['mass_kg']==2 and p['static_margin_candidate']==.05]
        self.assertTrue(all(p['screen_pass'] for p in selected))
