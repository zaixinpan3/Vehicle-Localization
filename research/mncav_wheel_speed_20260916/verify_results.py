#!/usr/bin/env python3
"""Independently check wheel-speed replay exports and recorded provenance.

uv run --offline --with numpy --with pandas --with h5py --with matplotlib python research/mncav_wheel_speed_20260916/verify_results.py
"""
from pathlib import Path
import hashlib
import json
import shutil
import h5py
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'output/mncav_wheel_speed_20260916'
DEST = Path(__file__).resolve().parent


def metrics(error, pose=False):
    e = np.asarray(error)
    if pose:
        return dict(positionRmseM=float(np.sqrt(np.mean(e*e))),positionMedianM=float(np.median(e)),
                    positionP95M=float(np.percentile(e,95,method='hazen')),positionMaximumM=float(np.max(e)))
    return dict(rmseMps=float(np.sqrt(np.mean(e*e))),biasMps=float(np.mean(e)),
                p95Mps=float(np.percentile(abs(e),95,method='hazen')),maximumMps=float(np.max(abs(e))))


def main():
    report=json.loads((OUT/'summary.json').read_text())
    source_checks={}
    for name in ['sensors','calibration_sensors']:
        folder=ROOT/'output/mississippi_20240607_120931_20260907'/name
        twist=pd.read_csv(folder/'twist.csv');steer=pd.read_csv(folder/'steering.csv')
        paired=twist.merge(steer,on='stamp_sec',validate='one_to_one')
        assert len(paired)==len(twist)==len(steer)
        assert np.array_equal(paired.linear_x_mps,paired.speed_mps)
        source_checks[name]={'samples':len(paired),'maximumTwistCanSpeedDifferenceMps':0.0}
    exported=pd.read_csv(OUT/'velocity_comparison.csv');maximum_difference=0.
    with h5py.File(OUT/'experiment.mat') as f:
        time=f['data/highRate/time'][()].ravel()
        speed=f['data/highRate/longitudinalSpeed'][()].ravel()
        raw_speed=f['stored/data/highRate/longitudinalSpeed'][()].ravel()
        ref=f['referenceVx'][()].ravel()
        assert np.allclose(exported.wheelVx,speed,rtol=0,atol=1e-12)
        assert np.array_equal(f['data/gnss/position'][()],f['stored/data/gnss/position'][()])
        assert np.array_equal(f['data/lidar/pose'][()],f['stored/data/lidar/pose'][()],equal_nan=True)
        for row,values in zip(report['velocityMetrics'],[raw_speed,speed]):
            for key,value in metrics(values-ref).items():
                maximum_difference=max(maximum_difference,abs(value-row[key]))
        old=f[f['stored/runs'][0,0]]['estimate/pose'][()].T
        pose=f['estimate/pose'][()].T;reference=f['stored/reference'][()].T
        old_error=np.linalg.norm(old[:,:2]-reference[:,:2],axis=1)
        wheel_error=np.linalg.norm(pose[:,:2]-reference[:,:2],axis=1)
        for row,poses in zip(report['localizationMetrics'],[old,pose]):
            for key,value in metrics(np.linalg.norm(poses[:,:2]-reference[:,:2],axis=1),True).items():
                maximum_difference=max(maximum_difference,abs(value-row[key]))
            yaw=np.arctan2(np.sin(poses[:,2]-reference[:,2]),np.cos(poses[:,2]-reference[:,2]))
            assert abs(np.rad2deg(np.sqrt(np.mean(yaw*yaw)))-row['headingRmseDeg'])<1e-10
        union=f['atNative/time'][()].ravel();indices=np.searchsorted(union,time)
        assert np.array_equal(union[indices],time)
        assert np.array_equal(f['atNative/z'][()].T[indices],f['estimate/z'][()].T)
        native_time=f['data/lidar/time'][()].ravel();ni=np.searchsorted(union,native_time)
        with h5py.File(ROOT/'output/saved_perception_inspva_20260915/experiment.mat') as mapping:
            frame_reference=mapping['reference'][()].T[:len(native_time)]
        frame_errors=np.linalg.norm(f['atNative/pose'][()].T[ni,:2]-frame_reference[:,:2],axis=1)
        frame_csv=pd.read_csv(OUT/'frame_errors.csv')
        assert np.max(abs(frame_errors-frame_csv.positionErrorM))<1e-11
        accepted=f['data/lidar/valid'][()].ravel().astype(bool)
        for key,value in metrics(frame_errors[accepted],True).items():
            maximum_difference=max(maximum_difference,abs(value-report['pairedWheelMetrics'][0][key]))
        cal=pd.read_csv(OUT/'12-11-24/motion.csv');selected=f['selected/longitudinalSpeed'][()].ravel()
        mask=cal.time>=40
        for key,value in metrics((selected-cal.referenceVx)[mask]).items():
            maximum_difference=max(maximum_difference,abs(value-report['calibrationSelected'][0][key]))
    assert maximum_difference<1e-10
    candidates=pd.read_csv(OUT/'calibration_candidates.csv')
    assert int(candidates.rmseMps.argmin())+1==report['selectionRow']
    # No threshold or radius is selected from the evaluation rows.
    assert report['selectedConfiguration']['method']=='robust'
    assert report['selectedConfiguration']['filterTimeConstant']==.01
    tests=pd.read_csv(OUT/'tests.csv');assert len(tests)==51 and tests.Passed.all() and not tests.Failed.any()
    assert len(pd.read_csv(OUT/'code_analyzer.csv'))==0
    segments=[]
    for name,mask in [('all',np.ones(len(time),bool)),('stationary_reference',abs(ref)<.1),
                      ('moving_reference',abs(ref)>=.1),('time_80_86',(time>=80)&(time<86))]:
        for method,values in [('legacy_twist',raw_speed),('wheel_fusion',speed)]:
            segments.append(dict(population=name,method=method,samples=int(mask.sum()),**metrics((values-ref)[mask])))
    pd.DataFrame(segments).to_csv(OUT/'velocity_segments.csv',index=False)
    fig,ax=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
    ax[0].plot(time,ref,color='black',lw=1,label='INSPVA reference');ax[0].plot(time,speed,lw=.7,label='Wheel fusion')
    ax[0].set_ylabel('Longitudinal speed (m/s)');ax[0].legend()
    ax[1].plot(time,raw_speed-ref,lw=.8,label='Legacy CAN speed');ax[1].plot(time,speed-ref,lw=.8,label='Wheel fusion')
    ax[1].set_ylabel('Velocity error (m/s)');ax[1].legend()
    ax[2].plot(time,old_error*100,lw=.8,label='Full observer: legacy CAN speed')
    ax[2].plot(time,wheel_error*100,lw=.8,label='Full observer: wheel fusion')
    ax[2].set_ylabel('Position discrepancy (cm)');ax[2].set_xlabel('Receiver time (s)');ax[2].legend()
    for axis in ax:axis.grid(alpha=.25)
    fig.suptitle('Independent-drive calibration does not improve this evaluation drive')
    fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)
    checks={'maximumMetricDifference':maximum_difference,'originalSourcePacketsUnchanged':True,
            'nativeOutputPreservesUniformStates':True,'sameAcceptedFrameCount':int(accepted.sum()),
            'matlabTestsPassed':51,'codeAnalyzerFindings':0,'twistCanSpeedAudit':source_checks,
            'evaluationWheelBiasSquaredFraction':report['velocityMetrics'][1]['biasMps']**2/report['velocityMetrics'][1]['rmseMps']**2,
            'claim':'Working wheel-derived candidate; no improvement over existing input on evaluation drive; not promoted to default.'}
    (DEST/'independent_checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    for name in ['summary.json','input_audit.json','calibration_candidates.csv','velocity_metrics.csv',
                 'velocity_segments.csv','localization_metrics.csv','tests.csv','code_analyzer.csv']:
        shutil.copy2(OUT/name,DEST/name)
    paths=list(OUT.rglob('*'))+[ROOT/p for p in ['scripts/calibrateMncavWheelSpeed.py','scripts/runMncavWheelSpeedExperiment.m',
          'localization/estimateWheelLongitudinalSpeed.m','config/wheelSpeedObserverConfig.m','config/mncavWheelSpeedCalibration.json',
          'tests/wheelLongitudinalSpeedTest.m']]
    manifest=[]
    for path in sorted(paths):
        if path.is_file():
            with path.open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
            manifest.append(dict(path=str(path.relative_to(ROOT)),bytes=path.stat().st_size,sha256=digest))
    (DEST/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(checks,indent=2))


if __name__=='__main__':main()
