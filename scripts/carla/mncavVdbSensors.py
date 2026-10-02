"""Simulator-side sensor physics. Never imported by the localization runtime.

Truth generates an ideal sensor signal; independent errors are then applied.
Only the returned measurement columns cross the estimator boundary. Bias
realizations and ideal signals are deliberately not exported in that table.
"""
import numpy as np
from scipy.spatial.transform import Rotation

SAE_TO_PROJECT = np.diag([1., -1., -1.])
Z_FLIP = np.diag([1., 1., -1.])
SENSOR_COLUMNS = ['time','specificX','specificY','specificZ','gyroX','gyroY','gyroZ',
                  'omegaFL','omegaFR','omegaRL','omegaRR','roadWheelAngle',
                  'gnssX','gnssY','gnssVariance','gnssValid','reserved']


def rotations(row):
    """SAE body->world and project body->world rotations."""
    r = Rotation.from_euler('xyz', [row['roll'], row['pitch'], row['yaw']]).as_matrix()
    return r, SAE_TO_PROJECT @ r @ SAE_TO_PROJECT


def drifting_bias(rng, dt, n, width, turn_on, walk):
    increments = rng.normal(size=(n, width)) * walk * np.sqrt(dt)[:, None]
    increments[0] = 0.
    return rng.normal(0., turn_on, width) + np.cumsum(increments, axis=0)


def synthesize_sensors(truth, cfg, offset, seed, errors=None):
    """Generate noisy specific force, gyro, native encoders and GNSS packets.

    The optional error model adds turn-on bias, random walk, correlated GNSS
    error, encoder scale/quantization and steering zero error. Independent RNG
    streams retain identical white errors in paired white/moderate profiles.
    """
    errors = errors or {}
    n = len(truth)
    dt = np.r_[0., np.diff(truth['time'])]
    assert n > 0 and np.all(dt[1:] > 0)
    rng = [np.random.default_rng(x) for x in np.random.SeedSequence(seed).spawn(10)]
    values = np.zeros((n, 17));values[:, 0] = truth['time']
    for k, row in enumerate(truth):
        _, rp = rotations(row)
        acceleration = SAE_TO_PROJECT @ np.array([row['ax'], row['ayRight'], row['azDown']])
        values[k, 1:4] = acceleration - rp.T @ np.array([0., 0., -9.81])
        values[k, 4:7] = SAE_TO_PROJECT @ np.array([row['p'], row['q'], row['r']])
        values[k, 7:11] = [row[f'omega{x}'] for x in ['FL','FR','RL','RR']]
        tangents = np.tan([row['steerFL'], row['steerFR']])
        values[k, 11] = -np.arctan(2. / np.sum(1. / tangents)) if np.all(np.abs(tangents)>1e-8) else -np.mean(tangents)
        position = np.array([row['x'], -row['y'], -row['zDown']]) + rp @ np.array([-offset, 0., 0.])
        values[k, 12:14] = position[:2]
    values[:, 1:4] += rng[0].normal(0., cfg['accelerometerNoiseStdMps2'], (n,3))
    values[:, 4:7] += rng[1].normal(0., cfg['gyroNoiseStdRadps'], (n,3))
    values[:, 7:11] *= 1. + rng[8].normal(0., errors.get('wheelScaleErrorStdFraction',0.), 4)
    values[:, 7:11] += rng[2].normal(0., cfg['wheelNoiseStdRadps'], (n,4))
    quant = errors.get('wheelQuantizationStepRadps',0.)
    if quant > 0:values[:, 7:11] = np.round(values[:, 7:11] / quant) * quant
    values[:, 11] += rng[3].normal(0., cfg['steeringNoiseStdRad'], n)
    values[:, 11] += rng[9].normal(0., errors.get('steeringZeroErrorStdRad',0.))
    values[:, 1:4] += drifting_bias(rng[5],dt,n,3,errors.get('accelerometerTurnOnBiasStdMps2',0.),errors.get('accelerometerBiasRandomWalkMps2PerSqrtSecond',0.))
    values[:, 4:7] += drifting_bias(rng[6],dt,n,3,errors.get('gyroTurnOnBiasStdRadps',0.),errors.get('gyroBiasRandomWalkRadpsPerSqrtSecond',0.))
    correlated = np.zeros((n,2));sigma = errors.get('gnssCorrelatedErrorStdM',0.)
    correlated[0] = rng[7].normal(0.,sigma,2)
    for k in range(1,n):
        decay = np.exp(-dt[k]/errors.get('gnssCorrelationTimeSeconds',20.))
        correlated[k] = decay*correlated[k-1]+rng[7].normal(0.,sigma*np.sqrt(1.-decay**2),2)
    values[:, 12:14] += rng[4].normal(0.,cfg['gnssNoiseStdM'],(n,2)) + correlated
    values[:, 14] = cfg['gnssNoiseStdM']**2 + sigma**2
    period = cfg.get('gnssPeriodSeconds',0.)
    valid = np.ones(n,dtype=bool) if period == 0 else np.isclose(truth['time']/period,np.round(truth['time']/period),atol=1e-7,rtol=0)
    values[:, 15] = valid
    values[~valid, 12:14] = np.nan
    return values
