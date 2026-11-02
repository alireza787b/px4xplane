"""Analysis-only regression tests; numpy/pyulog are optional runtime dependencies."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
try:
    from analyze_qtailsitter_tracking import goto_requests
    import numpy as np
except ModuleNotFoundError:
    goto_requests=None


@unittest.skipIf(goto_requests is None,'Analysis tests require numpy and pyulog')
class TrackingTests(unittest.TestCase):
    def requests(self,times,latitude,altitude):
        return {'command':np.full(len(times),192),'timestamp':np.array(times)*1e6,
                'param5':np.array(latitude),'param6':np.ones(len(times))*52,
                'param7':np.array(altitude)}

    def test_qgc_retries_do_not_create_false_legs(self):
        command=self.requests([60.8,61.4,97.9,98.4,99],[29,29,29.001,29.001,29.001],
                              [1530,1530,1530,1530,1530.02])
        raw,unique=goto_requests(command)
        np.testing.assert_array_equal(raw,[0,1,2,3,4])
        np.testing.assert_array_equal(unique,[0,2])

    def test_new_location_height_or_later_request_is_preserved(self):
        command=self.requests([10,10.5,11,15],[29,29.001,29.001,29.001],[100,100,101,101])
        _,unique=goto_requests(command)
        np.testing.assert_array_equal(unique,[0,1,2,3])


if __name__=='__main__':
    unittest.main()
