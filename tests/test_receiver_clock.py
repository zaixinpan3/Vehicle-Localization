"""Clock robustness, rollover and artifact invalidation without pose fitting."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from receiverClock import fit_clock, convert_time, native_seconds, ensure_clock


def delayed_pairs():
    time = np.arange(1001) * .02
    delay = np.zeros(len(time))
    delay[300:309] = [.01, .02, .03, .04, .05, .04, .03, .02, .01]
    delay[700:709] = delay[300:309]
    stamps = 1717776571.0 + (time - .007) / 1.09 + delay
    return stamps, time


class ReceiverClockTest(unittest.TestCase):
    def test_bursts_do_not_warp_query_time(self):
        stamps, time = delayed_pairs()
        model = fit_clock(stamps, time)
        query = 1717776571.0 + (6.08 - .007) / 1.09
        self.assertGreater(abs(np.interp(query, stamps, time) - 6.08), .03)
        self.assertLess(abs(float(convert_time(model, query)) - 6.08), 1e-6)
        self.assertFalse(model['absoluteLatencyCalibrated'])
        self.assertTrue(model['offlineUsesFutureSamples'])

    def test_missing_packets_and_gps_week_rollover(self):
        stamps, time = delayed_pairs()
        keep = np.arange(len(time)) % 7 != 0
        absolute = 604795 + time[keep]
        receiver = native_seconds(2317 + np.floor(absolute / 604800), absolute % 604800,
                                  2317, 604795)
        model = fit_clock(stamps[keep], receiver)
        np.testing.assert_allclose(convert_time(model, stamps[[100, 500, 900]]),
                                   time[[100, 500, 900]], atol=1e-6)

    def test_clock_reset_rejected(self):
        stamps, time = delayed_pairs()
        stamps[500:] -= 20
        with self.assertRaises(ValueError):
            fit_clock(stamps, time)

    def test_nonlinear_drift_rejected(self):
        stamps, time = delayed_pairs()
        with self.assertRaises(ValueError):
            fit_clock(stamps + .002 * time**2, time)

    def test_no_unbounded_extrapolation_or_nan(self):
        stamps, time = delayed_pairs()
        model = fit_clock(stamps, time)
        with self.assertRaises(ValueError):
            convert_time(model, stamps[-1] + .1)
        with self.assertRaises(ValueError):
            convert_time(model, np.nan)

    def test_invalid_coefficients_rejected(self):
        stamps, time = delayed_pairs()
        model = fit_clock(stamps, time)
        model['scale'] = -1
        with self.assertRaises(ValueError):
            convert_time(model, stamps)

    def test_too_few_or_duplicate_pairs_rejected(self):
        with self.assertRaises(ValueError):
            fit_clock(np.arange(19), np.arange(19))
        with self.assertRaises(ValueError):
            fit_clock(np.zeros(30), np.arange(30))

    def test_source_change_invalidates_cached_clock(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'ins.csv'
            header = 'stamp_sec,gps_week,gps_seconds\n'
            path.write_text(header + ''.join(f'{100+i},2317,{200+i}\n' for i in range(30)))
            first = ensure_clock(path)
            self.assertEqual(first, ensure_clock(path))
            path.write_text(header + ''.join(f'{100+i},2317,{200+1.01*i}\n' for i in range(30)))
            second = ensure_clock(path)
            self.assertNotEqual(first['modelId'], second['modelId'])
            self.assertAlmostEqual(second['scale'], 1.01)

    def test_shared_matlab_fixture_matches_python(self):
        folder = Path(__file__).parent / 'fixtures/receiver_clock'
        model = json.loads((folder / 'native.clock.json').read_text())
        query = json.loads((folder / 'queries.json').read_text())
        np.testing.assert_allclose(convert_time(model, query['stamps']), query['receiverSeconds'], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
