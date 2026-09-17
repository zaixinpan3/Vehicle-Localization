#!/usr/bin/env python3
"""Calibrate a planar BESTPOS-to-INS output-point offset on a separate drive.

uv run --offline --with rosbags --with numpy --with pandas --with pyproj python scripts/calibrateMncavBestposOutputPoint.py
Only seconds 1--40 of 12-11-24 fit the offset. The 12-09-31 evaluation drive
is never read. This is an empirical relative calibration, not a hardware survey.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
import pandas as pd
from pyproj import Proj, Transformer
from rosbags.rosbag1 import Reader
from extractGnssFromBag import build_typestore, read_topic_rows, bestpos_row, inspva_row, write_csv

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output/mncav_bestpos_alignment_20260917'


def main():
    sources = OUT / 'calibration_sources'
    sources.mkdir(parents=True, exist_ok=True)
    bag = ROOT / 'data/raw/Missisipi/raw_data_2024-06-07-12-11-24_0.bag'
    with Reader(bag) as reader:
        types = build_typestore(reader, {'/novatel/oem7/bestpos', '/novatel/oem7/inspva'})
        for name, builder in [('bestpos', bestpos_row), ('inspva', inspva_row)]:
            write_csv(sources / (name + '.csv'), read_topic_rows(reader, types, '/novatel/oem7/' + name, builder))
    best, ins = [pd.read_csv(sources / (name + '.csv')) for name in ['bestpos', 'inspva']]
    def clock(frame):
        return (frame.gps_week.to_numpy()-ins.gps_week.iloc[0])*604800 + frame.gps_seconds.to_numpy()-ins.gps_seconds.iloc[0]
    tb, ti = clock(best), clock(ins)
    assert np.all(np.diff(tb) > 0) and np.all(np.diff(ti) > 0)
    transform = Transformer.from_crs(4326, 32615, always_xy=True)
    xb, yb = transform.transform(best.longitude_deg, best.latitude_deg)
    xi, yi = transform.transform(ins.longitude_deg, ins.latitude_deg)
    convergence = Proj('EPSG:32615').get_factors(ins.longitude_deg, ins.latitude_deg).meridian_convergence
    yaw = np.unwrap(np.deg2rad(90-ins.azimuth_deg.to_numpy()+convergence))
    psi = np.interp(tb, ti, yaw)
    dx, dy = xb-np.interp(tb, ti, xi), yb-np.interp(tb, ti, yi)
    body = np.column_stack((np.cos(psi)*dx+np.sin(psi)*dy, -np.sin(psi)*dx+np.cos(psi)*dy))
    status = np.interp(tb, ti, ins.ins_status)
    valid = (tb >= ti[0]) & (tb <= ti[-1]) & (best.solution_status == 0) & (status == 3) & np.isfinite(body).all(axis=1)
    train = valid & (tb >= 1) & (tb <= 40)
    holdout = valid & (tb > 40)
    assert train.sum() >= 300 and holdout.sum() >= 300
    offset = np.median(body[train], axis=0)
    # A 5 cm floor covers planar-model residuals; do not treat the sample-mean
    # uncertainty as the uncertainty of a future individual position.
    sigma = np.maximum(.05, 1.4826*np.median(abs(body[train]-offset), axis=0))
    calibration = dict(identifier='mncav-bestpos-relative-point-12-11-24-seconds-1-40-v1',
        bodyOffset=offset.tolist(), bodyCovariance=np.diag(sigma**2).tolist(), headingStdRad=float(np.deg2rad(1)),
        convention='BESTPOS point minus observer/map INSPVA output point; forward/left metres; subtract R(estimated yaw)*offset',
        interpretation='Independent-drive empirical relative output-point calibration, not a surveyed antenna-to-CG lever arm',
        calibrationBag=str(bag.relative_to(ROOT)), trainingSeconds=[1, 40], trainingSamples=int(train.sum()),
        evaluationDriveUsed=False, runtimeReferenceAttitudeUsed=False, runtimeReferencePositionUsed=False,
        uncertainty='Per-sample robust training residual sigma with 0.05 m floor per body axis; declared 1 degree heading sigma; unknown receiver correlations',
        sources=[dict(path=str(p.relative_to(ROOT)), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in [sources/'bestpos.csv', sources/'inspva.csv']])
    (ROOT/'config/mncavBestposOutputPoint.json').write_text(json.dumps(calibration, indent=2)+'\n')
    rows=[]
    for label, mask in [('training_1_40', train), ('held_out_after_40', holdout)]:
        residual=np.linalg.norm(body[mask]-offset, axis=1)
        rows.append(dict(population=label, samples=int(mask.sum()), beforeRmseM=float(np.sqrt(np.mean(np.sum(body[mask]**2, axis=1)))),
            afterRmseM=float(np.sqrt(np.mean(residual**2))), afterMedianM=float(np.median(residual)),
            afterP95M=float(np.percentile(residual, 95)), afterMaximumM=float(residual.max())))
    pd.DataFrame(rows).to_csv(OUT/'calibration_validation.csv', index=False)
    pd.DataFrame(dict(time=tb, forwardOffset=body[:,0], leftOffset=body[:,1], training=train, heldOut=holdout)).to_csv(sources/'body_offsets.csv', index=False)
    print(json.dumps(calibration, indent=2));print(pd.DataFrame(rows).to_string(index=False))


if __name__ == '__main__':
    main()
