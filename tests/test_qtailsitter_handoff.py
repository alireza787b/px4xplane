"""Offline analysis frame tests; never run or modify a simulator."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
try:
    from analyze_qtailsitter_handoff import multiply, quaternion, euler, peak, mean_or_none
    import numpy as np
except ModuleNotFoundError:
    multiply = None


@unittest.skipIf(multiply is None, 'Analysis tests require numpy and pyulog')
class HandoffTests(unittest.TestCase):
    def test_no_braking_has_json_compatible_empty_mean(self):
        self.assertIsNone(mean_or_none([]))
        self.assertIsNone(mean_or_none([float('nan')]))
        self.assertEqual(mean_or_none([1, 3]), 2)

    def test_fw_level_is_mc_nose_down_90(self):
        angle = 2**-.5
        fw = multiply(np.array([[angle, 0, -angle, 0]]), [angle, 0, angle, 0])
        np.testing.assert_allclose(euler(fw), [[0, 0, 0]], atol=1e-6)

    def test_nonzero_fw_bank_survives_frame_roundtrip(self):
        bank = np.radians(23)
        fw = np.array([[np.cos(bank/2), np.sin(bank/2), 0, 0]])
        mc = multiply(fw, [2**-.5, 0, -2**-.5, 0])
        np.testing.assert_allclose(euler(multiply(mc, [2**-.5, 0, 2**-.5, 0])), [[23, 0, 0]], atol=1e-6)

    def test_quaternion_column_order_and_frequency(self):
        data = {f'q[{i}]': np.full(2, float(i == 0)) for i in range(4)}
        np.testing.assert_array_equal(quaternion(data, 'q'), [[1, 0, 0, 0]]*2)
        times = np.arange(0, 20, .01)
        self.assertAlmostEqual(peak(times, np.sin(2*np.pi*.4*times), 0, 20), .4)


if __name__ == '__main__':
    unittest.main()
