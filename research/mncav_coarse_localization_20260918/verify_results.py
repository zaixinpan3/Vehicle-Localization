#!/usr/bin/env python3
"""Audit raw coarse localization and independently recompute its metrics.

uv run --offline --with numpy --with pandas --with h5py python research/mncav_coarse_localization_20260918/verify_results.py
"""
from pathlib import Path
import json
import h5py
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output/mncav_coarse_localization_20260918'
DEST = Path(__file__).resolve().parent


def score(pose, reference):
    error = np.linalg.norm(pose[:, :2] - reference[:, :2], axis=1)
    angle = np.arctan2(np.sin(pose[:, 2] - reference[:, 2]),
                       np.cos(pose[:, 2] - reference[:, 2]))
    return dict(positionRmseM=np.sqrt(np.mean(error**2)),
                positionMedianM=np.median(error),
                positionP95M=np.percentile(error, 95, method='hazen'),
                positionMaximumM=error.max(), fractionAtMost10cm=np.mean(error <= .1),
                headingRmseDeg=np.rad2deg(np.sqrt(np.mean(angle**2))))


def main():
    calls = pd.read_csv(OUT / 'matching/calls.csv')
    assert np.array_equal(calls.frame, np.arange(1, 1171))
    accepted = calls.accepted.to_numpy(bool)
    directional = calls.directionalAccepted.to_numpy(bool)
    assert not np.any(accepted & directional)
    information = np.array([[calls.informationXX, calls.informationXY, calls.informationXPsi],
                            [calls.informationXY, calls.informationYY, calls.informationYPsi],
                            [calls.informationXPsi, calls.informationYPsi, calls.informationPsiPsi]]).transpose(2, 0, 1)
    assert np.isfinite(information[accepted]).all()
    eigenvalues = np.linalg.eigvalsh(information[accepted])
    assert eigenvalues.min() > 0
    # Reconstruct every recursive initial guess from the saved integrated
    # wheel/gyro motion and the previous output, without evaluation poses.
    dead = pd.read_csv(OUT / 'matching/dead_reckoning.csv')[['x', 'y', 'psi']].to_numpy()
    outputs = calls[['x', 'y', 'psi']].to_numpy()
    predicted = calls[['predictedX', 'predictedY', 'predictedPsi']].to_numpy()
    a = dead[:-1, 2]
    rotation = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]).transpose(2, 0, 1)
    local = np.einsum('nji,nj->ni', rotation, np.diff(dead[:, :2], axis=0))
    a = outputs[:-1, 2]
    rotation = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]).transpose(2, 0, 1)
    expected = outputs[:-1, :2] + np.einsum('nij,nj->ni', rotation, local)
    prediction_residual = np.max(abs(expected - predicted[1:, :2]))
    assert prediction_residual < 1e-7
    angle = predicted[1:, 2] - outputs[:-1, 2] - np.diff(dead[:, 2])
    assert np.max(abs(np.arctan2(np.sin(angle), np.cos(angle)))) < 1e-12
    rejected = ~(accepted | directional)
    assert np.array_equal(outputs[rejected], predicted[rejected])
    audit = json.loads((OUT / 'call_path_audit.json').read_text())
    assert audit['coarseEntryExecuted'] and audit['fineRefinementCalls'] == 0
    assert not any('refinePerceptionCandidates' in name for name in audit['functionNames'])
    metadata = json.loads((OUT / 'matching/metadata.json').read_text())
    assert metadata['mode'] == 'recursive' and metadata['perceptionRerun']
    assert metadata['perceptionMode'] == 'coarseProbabilityCloud' and not metadata['finePerceptionUsed']
    metrics = pd.read_csv(OUT / 'observer/metrics.csv')
    maximum_difference = 0
    unchanged = ['data/highRate/time', 'data/highRate/longitudinalSpeed',
                 'data/gnss/position', 'data/gnss/information',
                 'lateral/lateralVelocity', 'reference', 'cfg/gains', 'cfg/gnss/positionGain']
    source_roundoff = {}
    with h5py.File(OUT / 'observer/experiment.mat') as f, h5py.File(ROOT / 'output/mncav_bestpos_alignment_20260917/experiment.mat') as baseline:
        time = f['data/highRate/time'][()].ravel()
        reference = f['reference'][()].T
        valid = f['data/lidar/valid'][()].ravel().astype(bool)
        assert len(time) == 1169
        for field in unchanged:
            source_roundoff[field] = float(np.nanmax(abs(f[field][()] - baseline[field][()])))
            np.testing.assert_allclose(f[field][()], baseline[field][()], atol=1e-8, rtol=0, err_msg=field)
        assert np.array_equal(valid, accepted[:len(time)])
        np.testing.assert_allclose(f['data/lidar/pose'][()].T[valid], outputs[:len(time)][valid], atol=1e-8, rtol=0)
        np.testing.assert_allclose(f['data/lidar/information'][()].transpose(0, 2, 1), information[:len(time)], atol=1e-8, rtol=1e-14)
        assert np.isnan(f['data/lidar/pose'][()].T[~valid]).all()
        for i, name in enumerate(['both', 'lidar_only', 'gnss_only', 'gnss_outage', 'lidar_outage', 'both_outage', 'alternating']):
            estimate = f[f['runs'][0, i]]['estimate']
            pose = estimate['pose'][()].T
            assert np.isfinite(pose).all()
            for key in ['virtualPoseUpdates', 'integrationSubsteps', 'stateResets']:
                assert estimate['diagnostics/' + key][()].item() == 0
            assert estimate['diagnostics/localizationUpdates'][()].item() == len(time)-1
            for population, mask in [('full', np.ones(len(time), bool)), ('outage_40_60', (time >= 40) & (time < 60)), ('recovery_60_70', (time >= 60) & (time < 70))]:
                row = metrics[(metrics.scenario == name) & (metrics.population == population)].iloc[0]
                assert row.samples == mask.sum()
                maximum_difference = max(maximum_difference, *(abs(row[k]-v) for k, v in score(pose[mask], reference[mask]).items()))
        fused = f[f['runs'][0, 0]]['estimate/pose'][()].T
        previous = baseline[baseline['runs'][0, 0]]['estimate/pose'][()].T
        trajectory = pd.DataFrame(dict(frame=calls.frame[:len(time)], time=time,
            referenceX=reference[:, 0], referenceY=reference[:, 1], referencePsi=reference[:, 2],
            x=fused[:, 0], y=fused[:, 1], psi=fused[:, 2], lidarAccepted=valid,
            positionErrorM=np.linalg.norm(fused[:, :2]-reference[:, :2], axis=1)))
        trajectory.to_csv(OUT / 'trajectory.csv', index=False)
        historical = dict(current=score(fused, reference), previous=score(previous, reference),
            caution='Fine per-frame reference-seeded historical input versus fresh coarse recursive matching; not an isolated perception comparison')
    assert maximum_difference < 1e-10
    result = dict(rawScans=1170, observerFrames=len(time), fullPoseMeasurements=int(accepted.sum()),
        directionalMeasurements=int(directional.sum()), minimumAcceptedInformationEigenvalue=float(eigenvalues.min()),
        recursivePredictionMaximumResidualM=float(prediction_residual), metricRowsVerified=len(metrics),
        maximumMetricResidual=float(maximum_difference), observerSourceMaximumRoundoff=source_roundoff,
        rejectedFullPosePayloadsAreNaN=True, sourceMapWasNotRebuilt=True,
        fineRefinementCallsInIndependentTrace=0, historicalComparison=historical)
    (DEST / 'independent_checks.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
