#!/usr/bin/env python3
"""Independently audit fixed-calibration replay, metrics and measurement geometry.

uv run --offline --with numpy --with pandas --with h5py python research/mncav_bestpos_alignment_20260917/verify_results.py
"""
from pathlib import Path
import hashlib,json,shutil
import h5py
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent
OUT=ROOT/'output/mncav_bestpos_alignment_20260917'
OLD=ROOT/'output/mncav_synchronous_bestpos_20260917'


def score(pose,reference):
    e=np.linalg.norm(pose[:,:2]-reference[:,:2],axis=1)
    angle=np.arctan2(np.sin(pose[:,2]-reference[:,2]),np.cos(pose[:,2]-reference[:,2]))
    return dict(positionRmseM=np.sqrt(np.mean(e*e)),positionMedianM=np.median(e),
        positionP95M=np.percentile(e,95,method='hazen'),positionMaximumM=e.max(),
        fractionAtMost10cm=np.mean(e<=.1),headingRmseDeg=np.rad2deg(np.sqrt(np.mean(angle*angle))))


def main():
    cal=json.loads((ROOT/'config/mncavBestposOutputPoint.json').read_text());ell=np.array(cal['bodyOffset'])
    assert not cal['evaluationDriveUsed'] and '12-11-24' in cal['calibrationBag']
    body=pd.read_csv(OUT/'calibration_sources/body_offsets.csv')
    training=body[body.training][['forwardOffset','leftOffset']].to_numpy()
    assert len(training)==360 and np.max(abs(np.median(training,axis=0)-ell))<1e-14
    table=pd.read_csv(OUT/'metrics.csv');pairs=pd.read_csv(OUT/'paired_comparison.csv');max_difference=0
    with h5py.File(OUT/'experiment.mat') as f,h5py.File(OLD/'experiment.mat') as previous:
        t=f['data/highRate/time'][()].ravel();reference=f['reference'][()].T
        for field in ['data/highRate/time','data/highRate/longitudinalSpeed','data/gnss/position','data/gnss/information',
                      'data/lidar/pose','data/lidar/valid','lateral/lateralVelocity','reference','cfg/gains','cfg/initialState']:
            assert np.array_equal(f[field][()],previous[field][()],equal_nan=True),field
        assert f['cfg/gnss/positionGain'][()].item()==4 and previous['cfg/gnss/positionGain'][()].item()==1
        for field in ['headingGain','gainInformationScale']:
            assert np.array_equal(f['cfg/gnss/'+field][()],previous['cfg/gnss/'+field][()])
        wheel_time=f['wheel/time'][()].ravel();wheel_speed=f['wheel/longitudinalSpeed'][()].ravel()
        assert np.max(abs(f['data/highRate/longitudinalSpeed'][()].ravel()-np.interp(t,wheel_time,np.maximum(0,wheel_speed))))<1e-12
        accepted=f['data/lidar/valid'][()].ravel().astype(bool)
        assert len(t)==1169 and accepted.sum()==1083
        for name in ['gnss','lidar']:assert np.array_equal(t,f[f'data/{name}/time'][()].ravel())
        for i,scenario in enumerate(['both','lidar_only','gnss_only','gnss_outage','lidar_outage','both_outage','alternating']):
            est=f[f['runs'][0,i]]['estimate'];pose=est['pose'][()].T
            assert np.isfinite(pose).all()
            for key in ['virtualPoseUpdates','integrationSubsteps','stateResets']:assert est['diagnostics/'+key][()].item()==0
            assert est['diagnostics/localizationUpdates'][()].item()==len(t)-1
            for population,mask in [('full',np.ones(len(t),bool)),('outage_40_60',(t>=40)&(t<60)),('recovery_60_70',(t>=60)&(t<70))]:
                row=table[(table.scenario==scenario)&(table.population==population)].iloc[0]
                assert row.samples==mask.sum()
                max_difference=max(max_difference,*(abs(row[k]-v) for k,v in score(pose[mask],reference[mask]).items()))
        est=f[f['runs'][0,0]]['estimate'];pose=est['pose'][()].T
        yaw=est['headingUnwrapped'][()].ravel();raw=f['data/gnss/position'][()].T
        rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]).transpose(2,0,1)
        corrected=raw-np.einsum('nij,j->ni',rotation,ell)
        point_residual=np.max(abs(corrected-est['diagnostics/gnssPositionAtObserverPoint'][()].T))
        assert point_residual<1e-8
        raw_info=f['data/gnss/information'][()].transpose(0,2,1)
        tangent=np.einsum('nij,j->ni',rotation,[-ell[1],ell[0]])
        covariance=np.linalg.inv(raw_info)+rotation@np.array(cal['bodyCovariance'])@rotation.transpose(0,2,1)+cal['headingStdRad']**2*tangent[:,:,None]*tangent[:,None,:]
        info_residual=np.max(abs(np.linalg.inv(covariance)-est['diagnostics/gnssInformationAtObserverPoint'][()].transpose(0,2,1)))
        assert info_residual<1e-9
        for name,p in [('aligned_bestpos_10hz',pose),('unaligned_bestpos_10hz',previous[previous['runs'][0,0]]['estimate/pose'][()].T),('raw_lidar',f['data/lidar/pose'][()].T)]:
            row=pairs[pairs.method==name].iloc[0]
            max_difference=max(max_difference,*(abs(row[k]-v) for k,v in score(p[accepted],reference[accepted]).items()))
        error=np.linalg.norm(pose[:,:2]-reference[:,:2],axis=1);worst=int(np.argmax(error))
        populations={label:dict(samples=int(mask.sum()),rmseM=float(np.sqrt(np.mean(error[mask]**2))),squaredErrorFraction=float(np.sum(error[mask]**2)/np.sum(error**2))) for label,mask in [('lidar_accepted',accepted),('lidar_unavailable',~accepted)]}
        # Scoring diagnostic only: evaluate the frozen calibration using reference
        # yaw. That attitude is never supplied to the online point correction.
        psi=reference[:,2];delta=raw-reference[:,:2]
        local=np.column_stack((np.cos(psi)*delta[:,0]+np.sin(psi)*delta[:,1],-np.sin(psi)*delta[:,0]+np.cos(psi)*delta[:,1]))
        transfer=dict(samples=len(t),beforeRmseM=float(np.sqrt(np.mean(np.sum(local**2,axis=1)))),afterRmseM=float(np.sqrt(np.mean(np.sum((local-ell)**2,axis=1)))),scope='Frozen independent-drive calibration; evaluation-reference yaw used for this diagnostic only')
    assert max_difference<1e-12
    controls=pd.read_csv(OUT/'alignment_controls.csv').set_index('method')
    assert abs(controls.loc['unaligned','positionRmseM']-.397953028252893)<1e-12
    assert controls.loc['aligned_gain4','acceptedPositionRmseM']<pairs[pairs.method=='raw_lidar'].positionRmseM.iloc[0]
    validation=json.loads((OUT/'validation.json').read_text())
    assert validation['tests']==152 and validation['allTestsPassed'] and validation['analyzerFindings']==0
    assert validation['repeatMaximumStateDifference']==validation['futureMutationPrefixDifference']==0
    check=dict(metricRowsVerified=len(table),maximumMetricDifference=max_difference,
        correctedPositionEquationResidualM=float(point_residual),correctedInformationResidual=float(info_residual),
        unchangedRawSourcesClocksLateralStatesAndOtherGains=True,gnssPositionGainBefore=1,gnssPositionGainAfter=4,wheelOnlyVx=True,calibrationTrainingSamples=len(training),
        calibrationUsesSeparateDrive=True,evaluationReferenceUsedByRuntime=False,matlabTests=validation['tests'],
        worstFrame=dict(time=float(t[worst]),errorM=float(error[worst]),lidarAccepted=bool(accepted[worst])),
        errorPopulations=populations,outputPointTransferDiagnostic=transfer)
    (DEST/'independent_checks.json').write_text(json.dumps(check,indent=2)+'\n')
    for name in ['summary.json','metrics.csv','paired_comparison.csv','alignment_controls.csv','calibration_validation.csv','validation.json','tests.csv','code_analyzer.csv']:
        shutil.copy2(OUT/name,DEST/name)
    manifest=[]
    for p in sorted(OUT.rglob('*')):
        if p.is_file() and p.name!='prototype.mat':
            with p.open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
            manifest.append(dict(path=str(p.relative_to(ROOT)),sha256=digest,bytes=p.stat().st_size))
    (DEST/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(check,indent=2))


if __name__=='__main__':main()
