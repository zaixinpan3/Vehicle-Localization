#!/usr/bin/env python3
"""Independently verify frame-rate localization, sources and discrepancy metrics.

uv run --offline --with numpy --with pandas --with h5py --with pyproj python research/mncav_synchronous_bestpos_20260917/verify_results.py
"""
from pathlib import Path
import hashlib,json,shutil
import h5py
import numpy as np
import pandas as pd
from pyproj import Transformer

ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent
OUT=ROOT/'output/mncav_synchronous_bestpos_20260917'


def score(pose,reference):
    e=np.linalg.norm(pose[:,:2]-reference[:,:2],axis=1)
    angle=np.arctan2(np.sin(pose[:,2]-reference[:,2]),np.cos(pose[:,2]-reference[:,2]))
    return dict(positionRmseM=np.sqrt(np.mean(e*e)),positionMedianM=np.median(e),
        positionP95M=np.percentile(e,95,method='hazen'),positionMaximumM=e.max(),
        fractionAtMost10cm=np.mean(e<=.1),headingRmseDeg=np.rad2deg(np.sqrt(np.mean(angle*angle))))


def main():
    table=pd.read_csv(OUT/'metrics.csv');max_difference=0
    with h5py.File(OUT/'experiment.mat') as f:
        t=f['data/highRate/time'][()].ravel();reference=f['reference'][()].T
        assert len(t)==1169 and .09<np.median(np.diff(t))<.11
        for name in ['gnss','lidar']:
            assert np.array_equal(t,f[f'data/{name}/time'][()].ravel())
        wheel_time=f['wheel/time'][()].ravel();wheel_speed=f['wheel/longitudinalSpeed'][()].ravel()
        assert np.max(abs(f['data/highRate/longitudinalSpeed'][()].ravel()-np.interp(t,wheel_time,np.maximum(0,wheel_speed))))<1e-12
        for i,scenario in enumerate(['both','lidar_only','gnss_only','gnss_outage','lidar_outage','both_outage','alternating']):
            est=f[f['runs'][0,i]]['estimate'];pose=est['pose'][()].T
            assert np.isfinite(pose).all()
            assert est['diagnostics/virtualPoseUpdates'][()].item()==0
            assert est['diagnostics/localizationUpdates'][()].item()==len(t)-1
            assert est['diagnostics/integrationSubsteps'][()].item()==0
            for population,mask in [('full',np.ones(len(t),bool)),('outage_40_60',(t>=40)&(t<60)),('recovery_60_70',(t>=60)&(t<70))]:
                row=table[(table.scenario==scenario)&(table.population==population)].iloc[0]
                assert row.samples==mask.sum()
                max_difference=max(max_difference,*(abs(row[k]-v) for k,v in score(pose[mask],reference[mask]).items()))
        full_pose=f[f['runs'][0,0]]['estimate/pose'][()].T
        current_gnss=f['data/gnss/position'][()].T
        lidar=f['data/lidar/pose'][()].T;accepted=f['data/lidar/valid'][()].ravel().astype(bool)
        assert accepted.sum()==1083
    calls=pd.read_csv(ROOT/'output/saved_perception_inspva_20260915/calls.csv')
    calls=calls[(calls['mode']=='per_frame_zero')&(calls.time<=t[-1])]
    assert np.max(abs(t-calls.time.to_numpy()))<1e-12
    assert np.allclose(lidar,calls[['measurementX','measurementY','measurementPsi']].to_numpy(),rtol=0,atol=1e-8,equal_nan=True)
    best=pd.read_csv(OUT/'bestpos.csv');expected=np.column_stack([np.interp(t,best.time,best[c]) for c in ['x','y']])
    assert np.max(abs(current_gnss-expected))<2e-9
    raw=pd.read_csv(ROOT/'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_bestpos.csv')
    x,y=Transformer.from_crs(4326,32615,always_xy=True).transform(raw.longitude_deg,raw.latitude_deg)
    assert np.max(abs(best.x-x))<1e-9 and np.max(abs(best.y-y))<1e-9
    pairs=pd.read_csv(OUT/'paired_comparison.csv')
    for name,pose in [('synchronous_10hz',full_pose),('raw_lidar',lidar)]:
        row=pairs[pairs.method==name].iloc[0]
        max_difference=max(max_difference,*(abs(row[k]-v) for k,v in score(pose[accepted],reference[accepted]).items()))
    assert max_difference<1e-12
    # Diagnostic only: compare output points without fitting/applying a correction.
    native=pd.read_csv(ROOT/'output/mncav_inspva_observer_20260915/native_reference.csv')
    covered=(best.time>=native.time.min())&(best.time<=native.time.max())
    diagnostic_excluded=int((~covered).sum());best=best[covered]
    yaw=np.interp(best.time,native.time,native.psi)
    dx=best.x-np.interp(best.time,native.time,native.x);dy=best.y-np.interp(best.time,native.time,native.y)
    body=np.column_stack((np.cos(yaw)*dx+np.sin(yaw)*dy,-np.sin(yaw)*dx+np.cos(yaw)*dy))
    speed=pd.read_csv(ROOT/'output/mncav_wheel_only_20260916/calibration/velocity_comparison.csv')
    stopped=np.interp(best.time,speed.time,speed.wheelVx)<.1
    discrepancy=dict(positionRmseM=float(np.sqrt(np.mean(dx**2+dy**2))),bodyMedianM=np.median(body,axis=0).tolist(),
        bodyP05M=np.percentile(body,5,axis=0).tolist(),bodyP95M=np.percentile(body,95,axis=0).tolist(),
        samples=len(best),excludedOutsideReferenceCoverage=diagnostic_excluded,
        stationarySamples=int(stopped.sum()),stationaryBodyMedianM=np.median(body[stopped],axis=0).tolist(),
        interpretation='A persistent body-frame offset including standstill suggests different output points; no verified lever arm or correction inferred.')
    (DEST/'bestpos_reference_discrepancy.json').write_text(json.dumps(discrepancy,indent=2)+'\n')
    validation=json.loads((OUT/'validation.json').read_text());report=json.loads((OUT/'summary.json').read_text())
    assert validation['tests']==142 and validation['allTestsPassed'] and validation['analyzerFindings']==0
    assert validation['repeatMaximumStateDifference']==validation['futureMutationPrefixDifference']==0
    sync=report['metadata']['synchronization']
    assert not sync['motionModelMeasurementExtrapolation'] and sync['maximumGnssWaitSeconds']<=.1+1e-8
    check=dict(metricRowsVerified=len(table),maximumMetricDifference=max_difference,frames=len(t),acceptedFrames=int(accepted.sum()),
        localizationRateHz=float(1/np.median(np.diff(t))),bestposRateHz=float(1/np.median(np.diff(best.time))),
        identicalSourceClocks=True,wheelOnlyVx=True,originalLidarPosesPreserved=True,bestposProjectionVerified=True,
        measurementPoseExtrapolation=False,integrationSubsteps=0,matlabTests=validation['tests'],
        maximumGnssAlignmentWaitSeconds=sync['maximumGnssWaitSeconds'],bestposPureGnss=False)
    (DEST/'independent_checks.json').write_text(json.dumps(check,indent=2)+'\n')
    for name in ['summary.json','metrics.csv','paired_comparison.csv','validation.json','tests.csv','code_analyzer.csv','bestpos_metadata.json']:
        shutil.copy2(OUT/name,DEST/name)
    manifest=[]
    for p in sorted(OUT.rglob('*')):
        if p.is_file():
            with p.open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
            manifest.append(dict(path=str(p.relative_to(ROOT)),sha256=digest,bytes=p.stat().st_size))
    (DEST/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(check,indent=2));print(json.dumps(discrepancy,indent=2))


if __name__=='__main__':main()
