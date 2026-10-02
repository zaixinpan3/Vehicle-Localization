"""Distribution and information-boundary checks for synthetic sensor packets."""
import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts/carla'))
from mncavVdbSensors import SENSOR_COLUMNS, synthesize_sensors


class SensorNoiseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names = ['time', 'x', 'y', 'zDown', 'roll', 'pitch', 'yaw',
                 'p', 'q', 'r', 'ax', 'ayRight', 'azDown',
                 'omegaFL', 'omegaFR', 'omegaRL', 'omegaRR', 'steerFL', 'steerFR',
                 'vx', 'vyRight', 'ReFL']
        cls.truth = np.zeros(20000, dtype=[(x, float) for x in names])
        cls.truth['time'] = np.arange(len(cls.truth)) * .02
        cls.cfg = dict(accelerometerNoiseStdMps2=.05, gyroNoiseStdRadps=.002,
                       wheelNoiseStdRadps=.01, steeringNoiseStdRad=.0005,
                       gnssNoiseStdM=.1, gnssPeriodSeconds=.1)
        cls.profiles = json.loads((ROOT / 'config/mncavVdbSensorNoise.json').read_text())
        cls.white = synthesize_sensors(cls.truth, cls.cfg, 0., 57, cls.profiles['white'])
        cls.moderate = synthesize_sensors(cls.truth, cls.cfg, 0., 57, cls.profiles['moderate'])

    def test_white_noise_has_declared_scale_and_no_large_mean(self):
        columns = list(range(1, 14))
        sigmas = [.05] * 3 + [.002] * 3 + [.01] * 4 + [.0005] + [.1] * 2
        for column, sigma in zip(columns, sigmas):
            observed = self.white[:, column].copy()
            if column == 3:
                observed -= 9.81
            observed = observed[np.isfinite(observed)]
            self.assertLess(abs(np.std(observed) / sigma - 1), .05)
            self.assertLess(abs(np.mean(observed)), 5 * sigma / np.sqrt(len(observed)))

    def test_reproducibility_and_seed_changes(self):
        same = synthesize_sensors(self.truth[:100], self.cfg, 0., 57, self.profiles['moderate'])
        repeat = synthesize_sensors(self.truth[:100], self.cfg, 0., 57, self.profiles['moderate'])
        other = synthesize_sensors(self.truth[:100], self.cfg, 0., 58, self.profiles['moderate'])
        np.testing.assert_array_equal(same, repeat)
        self.assertFalse(np.array_equal(same[:, 1:7], other[:, 1:7]))

    def test_native_gnss_packets_do_not_supply_intermediate_positions(self):
        valid = self.white[:, 15].astype(bool)
        np.testing.assert_array_equal(np.flatnonzero(valid), np.arange(0, len(valid), 5))
        self.assertTrue(np.all(np.isnan(self.white[~valid, 12:14])))
        self.assertTrue(np.all(np.isfinite(self.white[valid, 12:14])))

    def test_correlated_gnss_error_and_marginal_variance(self):
        valid = self.white[:, 15].astype(bool)
        residual = self.moderate[valid, 12] - self.white[valid, 12]
        self.assertGreater(np.corrcoef(residual[:-1], residual[1:])[0, 1], .97)
        self.assertGreater(np.std(residual), .01)
        np.testing.assert_allclose(self.moderate[:, 14], .1**2 + .05**2)

    def test_encoder_quantization_and_persistent_steering_zero(self):
        quant = self.profiles['moderate']['wheelQuantizationStepRadps']
        x = self.moderate[:, 7:11] / quant
        np.testing.assert_allclose(x, np.round(x), atol=1e-12)
        steering_delta = self.moderate[:, 11] - self.white[:, 11]
        self.assertGreater(abs(steering_delta[0]), 1e-7)
        self.assertLess(np.ptp(steering_delta), 1e-17)

    def test_gnss_error_choice_cannot_change_inertial_or_wheel_streams(self):
        errors = dict(self.profiles['white'], gnssCorrelatedErrorStdM=4.)
        sample = self.truth[:100]
        a = synthesize_sensors(sample, self.cfg, 0., 91, self.profiles['white'])
        b = synthesize_sensors(sample, self.cfg, 0., 91, errors)
        np.testing.assert_array_equal(a[:, 1:12], b[:, 1:12])

    def test_body_speed_and_effective_radius_are_not_measurements(self):
        a = self.truth[:100].copy()
        b = a.copy(); b['vx'] = 999.; b['vyRight'] = -100.; b['ReFL'] = 4.
        np.testing.assert_array_equal(
            synthesize_sensors(a, self.cfg, 0., 9),
            synthesize_sensors(b, self.cfg, 0., 9))
        self.assertEqual(len(SENSOR_COLUMNS), 17)
        self.assertFalse(any(x in SENSOR_COLUMNS for x in ['vx', 'vyRight', 'ReFL', 'yaw']))


if __name__ == '__main__':
    unittest.main()
