#!/usr/bin/env python3
"""Verify current wheel-only source routing and full-observer metrics.

uv run --offline --with numpy --with pandas --with h5py python research/mncav_wheel_only_20260916/verify_results.py
"""
from pathlib import Path
import hashlib,json,shutil
import h5py
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/mncav_wheel_only_20260916'
DEST=Path(__file__).resolve().parent


def main():
    for folder,count in [('sensors',5845),('calibration_sensors',3894)]:
        path=OUT/folder
        assert not (path/'twist.csv').exists()
        assert 'speed_mps' not in pd.read_csv(path/'steering.csv').columns
        wheel=pd.read_csv(path/'wheel_speed_report.csv');assert len(wheel)==count
        original=ROOT/'output/mncav_interface_audit_20260916'/('12-09-31' if folder=='sensors' else '12-11-24')/'wheel_speed_report.csv'
        old=pd.read_csv(original)
        fields=['stamp_sec','front_left','front_right','rear_left','rear_right']
        assert np.array_equal(wheel[fields].to_numpy(),old[fields].to_numpy())
        assert 'twist' not in json.loads((path/'manifest.json').read_text())['exports']
    routes=['scripts/prepareWheelMotionInputs.m','scripts/prepareMncavObserverReplay.m',
            'scripts/replayMississippiLocalization.m','scripts/runMncavFullObserverExperiment.m',
            'scripts/extractVehicleReplaySensors.py','scripts/calibrateMncavWheelSpeed.py',
            'scripts/runMncavWheelSpeedExperiment.m']
    for path in routes:
        code=(ROOT/path).read_text()
        assert 'twist.csv' not in code and 'linear_x_mps' not in code and 'speed_mps' not in code,path
    assert not (ROOT/'scripts/auditMncavReplayInterfaces.py').exists()
    table=pd.read_csv(OUT/'full_observer/metrics.csv');difference=0.
    with h5py.File(OUT/'full_observer/experiment.mat') as f:
        time=f['data/highRate/time'][()].ravel();ref=f['reference'][()].T
        speed=f['data/highRate/longitudinalSpeed'][()].ravel()
        assert np.array_equal(speed,np.maximum(0,f['wheel/longitudinalSpeed'][()].ravel()))
        valid=f['wheel/valid'][()].ravel().astype(bool)
        assert (~valid).sum()==3 and valid[3:].all()
        for i,scenario in enumerate(['both','lidar_only','gnss_only','gnss_outage','lidar_outage','both_outage','alternating']):
            run=f[f['runs'][0,i]]['estimate'];pose=run['pose'][()].T
            assert np.isfinite(pose).all()
            for population,mask in [('full',np.ones(len(time),bool)),('outage_40_60',(time>=40)&(time<60)),('recovery_60_70',(time>=60)&(time<70))]:
                row=table[(table.scenario==scenario)&(table.population==population)].iloc[0]
                e=np.linalg.norm(pose[mask,:2]-ref[mask,:2],axis=1)
                angle=np.arctan2(np.sin(pose[mask,2]-ref[mask,2]),np.cos(pose[mask,2]-ref[mask,2]))
                expected=dict(positionRmseM=np.sqrt(np.mean(e*e)),positionMedianM=np.median(e),positionP95M=np.percentile(e,95,method='hazen'),
                              positionMaximumM=e.max(),fractionAtMost10cm=np.mean(e<=.1),headingRmseDeg=np.rad2deg(np.sqrt(np.mean(angle*angle))))
                assert row.samples==mask.sum()
                difference=max(difference,*(abs(row[key]-v) for key,v in expected.items()))
        full_pose=f[f['runs'][0,0]]['estimate/pose'][()].T
    frame=pd.read_csv(OUT/'calibration/frame_errors.csv');accepted=frame.accepted.astype(bool)
    report=json.loads((OUT/'calibration/summary.json').read_text())
    paired=np.sqrt(np.mean(frame.positionErrorM[accepted]**2))
    assert accepted.sum()==1083
    assert abs(paired-report['pairedWheelMetrics'][0]['positionRmseM'])<1e-12
    with h5py.File(OUT/'calibration/experiment.mat') as f:
        union=f['atNative/time'][()].ravel();index=np.searchsorted(union,time)
        assert np.array_equal(union[index],time)
        assert np.array_equal(f['atNative/z'][()].T[index],f['estimate/z'][()].T)
        # MATLAB and Python clock bridges differ by roundoff. The optional
        # selection runner must agree with production within nanometres.
        selected_pose=f['estimate/pose'][()].T
        selection_pose_difference=float(np.max(abs(selected_pose-full_pose)))
        assert selection_pose_difference<1e-7
    velocity=pd.read_csv(OUT/'calibration/velocity_comparison.csv')
    velocity_rmse=float(np.sqrt(np.mean((velocity.wheelVx-velocity.referenceVx)**2)))
    assert abs(velocity_rmse-report['velocityMetrics'][0]['rmseMps'])<1e-12
    validation=json.loads((OUT/'full_observer/validation.json').read_text())
    assert validation['tests']==126 and validation['allTestsPassed'] and validation['analyzerFindings']==0
    assert validation['repeatMaximumStateDifference']==validation['futureMutationPrefixDifference']==0
    assert difference<1e-12
    smoke=json.loads((OUT/'matching_smoke/metadata.json').read_text())
    assert 'four-wheel' in smoke['motionSource'] and 'INSPVA' in smoke['reference']
    assert smoke['frameCount']==2
    check={'sourceRoutesWithoutAlternateSpeed':len(routes),'exportedEvaluationWheelPackets':5845,'exportedCalibrationWheelPackets':3894,
           'nativeWheelValuesUnchanged':True,'fullMetricRowsVerified':len(table),'maximumMetricDifference':difference,
           'productionVxExactlyWheelDerived':True,'initialPredictionOnlySamples':3,'sameAcceptedFrames':1083,
           'sameFramePositionRmseM':float(paired),'velocityRmseMps':velocity_rmse,
           'optionalRunnerMaximumPoseDifference':selection_pose_difference,'matlabTestsPassed':126,
           'matchingSmokeFrames':2,'oldDefaultRemoved':True}
    (DEST/'independent_checks.json').write_text(json.dumps(check,indent=2)+'\n')
    for source,name in [('full_observer/metrics.csv','metrics.csv'),('full_observer/summary.json','summary.json'),
                        ('full_observer/validation.json','validation.json'),('full_observer/tests.csv','tests.csv'),
                        ('full_observer/code_analyzer.csv','code_analyzer.csv'),('calibration/summary.json','wheel_summary.json')]:
        shutil.copy2(OUT/source,DEST/name)
    paths=list(OUT.rglob('*'))+[ROOT/name for name in routes]
    manifest=[]
    for path in sorted(paths):
        if path.is_file():
            with path.open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
            manifest.append(dict(path=str(path.relative_to(ROOT)),sha256=digest,bytes=path.stat().st_size))
    (DEST/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(check,indent=2))


if __name__=='__main__':main()
