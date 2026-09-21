"""Shared offline ROS-header to native GPS receiver clock.

Only timestamp pairs are fitted. Publication outliers cannot locally warp the
clock. The unknown constant publication latency is NOT calibrated by this fit.
MATLAB reads the same serialized model instead of estimating another bridge.
"""
from pathlib import Path
import argparse
import hashlib
import json
import struct

import numpy as np

CONFIG = dict(minimumSamples=20, minimumInlierFraction=0.6,
              minimumTimeCoverage=0.9, residualFloorSeconds=0.0005,
              maximumInlierP95Seconds=0.002, maximumExtrapolationSeconds=0.05)


def native_seconds(week, seconds, origin_week=None, origin_seconds=None):
    """Difference weeks before combining, retaining precision across rollover."""
    week, seconds = np.asarray(week, dtype=float), np.asarray(seconds, dtype=float)
    if week.shape != seconds.shape or not np.all(np.isfinite([week, seconds])):
        raise ValueError("Invalid GPS week/seconds")
    if np.any(week != np.floor(week)) or np.any((seconds < 0) | (seconds >= 604800)):
        raise ValueError("GPS time must contain full integer weeks and seconds of week")
    if origin_week is None:
        origin_week, origin_seconds = week.flat[0], seconds.flat[0]
    return (week - origin_week) * 604800 + (seconds - origin_seconds)


def fit_clock(stamps, receiver):
    """Fit one clock segment; resets/drift fail validation rather than bend time."""
    stamps, receiver = np.asarray(stamps, dtype=float), np.asarray(receiver, dtype=float)
    if (stamps.ndim != 1 or stamps.shape != receiver.shape
            or len(stamps) < CONFIG['minimumSamples']
            or not np.all(np.isfinite([stamps, receiver]))
            or np.any(np.diff(stamps) <= 0) or np.any(np.diff(receiver) <= 0)):
        raise ValueError("Need at least 20 finite strictly increasing clock pairs; split clock resets explicitly")
    x = stamps - stamps[0]
    # A bounded, deterministic long-baseline median-slope initialization avoids
    # allowing a burst of delayed messages to determine the initial clock rate.
    indices = np.unique(np.linspace(0, len(x) - 1, min(65, len(x))).astype(int))
    dx = x[indices, None] - x[None, indices]
    dy = receiver[indices, None] - receiver[None, indices]
    long = dx > 0.25 * x[-1]
    scale = float(np.median(dy[long] / dx[long]))
    offset = float(np.median(receiver - scale * x))
    keep = np.ones(len(x), dtype=bool)
    for _ in range(12):
        residual = receiver - (scale * x + offset)
        center = np.median(residual)
        sigma = 1.4826 * np.median(np.abs(residual - center))
        selected = np.abs(residual - center) <= max(CONFIG['residualFloorSeconds'], 3 * sigma)
        if np.count_nonzero(selected) < CONFIG['minimumSamples']:
            raise ValueError("Insufficient clock inliers")
        design = np.column_stack((x[selected], np.ones(np.count_nonzero(selected))))
        scale, offset = map(float, np.linalg.lstsq(design, receiver[selected], rcond=None)[0])
        converged = np.array_equal(selected, keep)
        keep = selected
        if converged:
            break
    residual = receiver - (scale * x + offset)
    fraction = float(np.mean(keep))
    coverage = float(np.ptp(x[keep]) / x[-1])
    p95 = float(np.percentile(np.abs(residual[keep]), 95))
    if (scale <= 0 or fraction < CONFIG['minimumInlierFraction']
            or coverage < CONFIG['minimumTimeCoverage']
            or p95 > CONFIG['maximumInlierP95Seconds']):
        raise ValueError("Clock is not a supported stable affine segment; inspect/reset/segment the recording")
    return dict(schemaVersion=1, method='robust_affine_receiver_clock',
                sourceOriginSeconds=float(stamps[0]), scale=scale, offsetSeconds=offset,
                sourceSpanSeconds=float(x[-1]),
                maximumExtrapolationSeconds=CONFIG['maximumExtrapolationSeconds'],
                offlineUsesFutureSamples=True, absoluteLatencyCalibrated=False,
                lidarEpoch='recorded_pointcloud_header; scan start/end offset not calibrated',
                configuration=CONFIG.copy(),
                quality=dict(samples=len(x), inliers=int(keep.sum()), inlierFraction=fraction,
                             inlierTimeCoverage=coverage, inlierResidualP95Seconds=p95,
                             rejectedSampleIndicesOneBased=(np.flatnonzero(~keep) + 1).tolist(),
                             maximumAbsoluteResidualSeconds=float(np.max(np.abs(residual)))))


def convert_time(model, stamps):
    """Return receiver seconds from the declared GPS origin, without clamping."""
    if model['schemaVersion'] != 1 or model['method'] != 'robust_affine_receiver_clock':
        raise ValueError("Unsupported receiver clock model")
    coefficients = [model[k] for k in ['sourceOriginSeconds', 'scale', 'offsetSeconds',
                                      'sourceSpanSeconds', 'maximumExtrapolationSeconds']]
    if (not np.all(np.isfinite(coefficients)) or model['scale'] <= 0
            or model['sourceSpanSeconds'] <= 0 or model['maximumExtrapolationSeconds'] < 0):
        raise ValueError("Invalid receiver clock coefficients")
    values = np.asarray(stamps, dtype=float) - model['sourceOriginSeconds']
    margin = model['maximumExtrapolationSeconds']
    if (not np.all(np.isfinite(values)) or np.any(values < -margin)
            or np.any(values > model['sourceSpanSeconds'] + margin)):
        raise ValueError("Timestamp outside the clock segment and its declared edge allowance")
    return model['scale'] * values + model['offsetSeconds']


def coefficient_digest(model):
    """Hash little-endian binary doubles identically in Python and MATLAB."""
    coefficients = [model[k] for k in ['sourceOriginSeconds', 'scale', 'offsetSeconds',
        'sourceSpanSeconds', 'maximumExtrapolationSeconds', 'receiverOriginGpsWeek', 'receiverOriginGpsSeconds']]
    return hashlib.sha256(struct.pack('<7d', *coefficients)).hexdigest()


def ensure_clock(inspva_path):
    """Create/reuse the source-hashed model next to a native INSPVA export."""
    inspva_path = Path(inspva_path)
    digest = hashlib.sha256(inspva_path.read_bytes()).hexdigest()
    path = inspva_path.with_suffix('.clock.json')
    if path.exists():
        model = json.loads(path.read_text())
        if (model.get('sourceSha256') == digest and model.get('configuration') == CONFIG
                and model.get('schemaVersion') == 1 and 'coefficientSha256' in model):
            check = dict(model)
            identity = check.pop('modelId')
            check.pop('coefficientSha256')
            if (identity == hashlib.sha256(json.dumps(check, sort_keys=True).encode()).hexdigest()
                    and model['coefficientSha256'] == coefficient_digest(model)):
                return model
            raise ValueError("Receiver clock model has been modified; remove it and regenerate")
    table = np.genfromtxt(inspva_path, delimiter=',', names=True)
    receiver = native_seconds(table['gps_week'], table['gps_seconds'])
    model = fit_clock(table['stamp_sec'], receiver)
    model.update(sourceSha256=digest, receiverOriginGpsWeek=int(table['gps_week'][0]),
                 receiverOriginGpsSeconds=float(table['gps_seconds'][0]))
    model['coefficientSha256'] = coefficient_digest(model)
    identity_payload = {k: v for k, v in model.items() if k != 'coefficientSha256'}
    model['modelId'] = hashlib.sha256(json.dumps(identity_payload, sort_keys=True).encode()).hexdigest()
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(model, indent=2) + '\n')
    temporary.replace(path)
    return model


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inspva', type=Path)
    args = parser.parse_args()
    print(json.dumps(ensure_clock(args.inspva), indent=2))
