"""Compare a constant epoch offset with an effective planar spatial offset.

Use saved feature-to-feature registrations; frame 94 is excluded from fitting.
Run: uv run --offline --with numpy --with scipy python <this file>
These same-drive, reference-assisted fits do not identify surveyed extrinsics.
"""
from pathlib import Path
import csv
import json
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).parent


def rotation(yaw):
    return np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def main():
    with (DEST / 'translation_training_pairs.csv').open() as handle:
        pairs = [r for r in csv.DictReader(handle) if r['usedForFit'] == '1']
    calls = np.genfromtxt(ROOT / 'output/receiver_clock_20260921/coarse_pipeline/matching/calls.csv',
                         delimiter=',', names=True, encoding='utf-8')
    native = np.genfromtxt(ROOT / 'output/receiver_synchronized_inputs/native_reference.csv',
                          delimiter=',', names=True, encoding='utf-8')
    native_pose = np.column_stack([native['x'], native['y'], np.unwrap(native['psi'])])
    time = native['time']
    frames = np.array([[int(r['query']), int(r['fixed'])] for r in pairs])
    assert not np.any(frames == 94)
    observed = []
    for row in pairs:
        q, f = int(row['query'])-1, int(row['fixed'])-1
        p_q = np.array([calls['referenceX'][q], calls['referenceY'][q]])
        p_f = np.array([calls['referenceX'][f], calls['referenceY'][f]])
        delta = np.array([float(row['errorX']), float(row['errorY'])])
        relative = rotation(calls['referencePsi'][f]).T @ (p_q + delta - p_f)
        relative_yaw = wrap(calls['referencePsi'][q] + float(row['yawErrorRad']) - calls['referencePsi'][f])
        observed.append([*relative, relative_yaw])
    observed = np.array(observed)

    def residual(parameters):
        bx, by, mounting_yaw, lag = parameters
        epochs = calls['timeSeconds'][frames - 1] + lag
        pose = np.stack([np.interp(epochs.ravel(), time, native_pose[:, j]).reshape(epochs.shape)
                         for j in range(3)], axis=2)
        values = []
        for k, (q, f) in enumerate(pose):
            body_delta = rotation(f[2]).T @ (q[:2] - f[:2])
            body_rotation = rotation(q[2] - f[2])
            predicted = rotation(mounting_yaw).T @ (body_delta + (body_rotation - np.eye(2)) @ [bx, by])
            values.append([*(predicted - observed[k, :2]), 10*wrap(q[2]-f[2]-observed[k, 2])])
        return np.array(values)

    models = [('identity', []), ('translation', [0, 1]), ('time_only', [3]),
              ('translation_and_time', [0, 1, 3]), ('translation_yaw_and_time', [0, 1, 2, 3])]
    records = []
    for name, indices in models:
        parameters = np.zeros(4)
        if indices:
            initial = np.array([2., 0., 0., 0.])[indices]
            lower = np.array([-5., -5., -.15, -.25])[indices]
            upper = -np.array([-5., -5., -.15, -.25])[indices]

            def loss(x):
                p = np.zeros(4); p[indices] = x
                return residual(p).ravel()

            fit = least_squares(loss, initial, bounds=(lower, upper), loss='soft_l1', f_scale=.1,
                                xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=2000)
            parameters[indices] = fit.x
            singular = np.linalg.svd(fit.jac, compute_uv=False).tolist()
        else:
            singular = []
        errors = residual(parameters)
        records.append(dict(model=name,translationXYM=parameters[:2].tolist(),mountingYawDeg=np.rad2deg(parameters[2]),
                            epochShiftSeconds=parameters[3],translationResidualRmseM=np.sqrt(np.mean(np.sum(errors[:, :2]**2,axis=1))),
                            yawResidualRmseDeg=np.rad2deg(np.sqrt(np.mean(errors[:, 2]**2))/10),jacobianSingularValues=singular))
    report = dict(trainingPairs=len(pairs),frame94Excluded=True,models=records,
                  residual='Relative planar transform disagreement; 10 m angular lever for fitting, robust soft-L1 0.1 m scale.',
                  epochShiftBoundsSeconds=[-.25,.25],
                  limitations='Same-drive paired features and recorded INS; gravity-aligned planar approximation. Fitted increments conflate physical mounting, pose convention, residual time and scan distortion. Not an absolute time or extrinsic calibration.')
    (DEST / 'spatial_vs_time_models.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
