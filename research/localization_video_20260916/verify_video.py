#!/usr/bin/env python3
"""Check video clocks, exported state fidelity and original result metrics.

uv run --offline --with numpy --with h5py python research/localization_video_20260916/verify_video.py
The renderer separately checks encoded frame counts and complete decoding.
"""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import h5py
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/localization_video_20260916'
DEST=Path(__file__).resolve().parent


def main():
    tr=np.genfromtxt(OUT/'trajectory.csv',delimiter=',',names=True)
    frames=np.genfromtxt(OUT/'frames.csv',delimiter=',',names=True)
    clock=np.genfromtxt(OUT/'video_frame_clock.csv',delimiter=',',names=True)
    validation=json.loads((OUT/'video_validation.json').read_text())
    with h5py.File(ROOT/'output/mncav_interface_audit_20260916/correction_experiment.mat') as f:
        r=f[f['runs'][3,0]];estimate=np.column_stack((r['estimate/position'][()].T,r['estimate/headingUnwrapped'][()].ravel()))
        source_time=r['estimate/time'][()].ravel()
    with h5py.File(ROOT/'output/mncav_inspva_observer_20260915/experiment.mat') as f:
        r=f[f['experiments'][0,0]];truth=r['reference'][()].T;uniform=r['uniformIndices'][()].ravel().astype(int)-1
        frame_indices=r['frameIndices'][()].ravel().astype(int)-1
    exported_truth=np.column_stack([tr[k] for k in ['truthX','truthY','truthYaw']])
    exported_estimate=np.column_stack([tr[k] for k in ['estimateX','estimateY','estimateYaw']])
    maximum_pose_difference=float(max(np.max(np.abs(truth-exported_truth)),np.max(np.abs(estimate-exported_estimate))))
    assert maximum_pose_difference<1e-8 and np.max(np.abs(tr['time']-source_time))<1e-10
    assert np.max(np.abs(frames['time']-source_time[frame_indices]))<1e-9
    selected=np.searchsorted(frames['time'],clock['replayTime']+1e-10,side='right')
    assert np.array_equal(selected,clock['perceptionFrame'])
    ages=clock['replayTime']-frames['time'][selected-1]
    assert np.min(ages)>-1e-10 and np.max(ages)<np.max(np.diff(frames['time']))+1e-10
    assert set(selected)==set(range(1,1171)) and len(clock)==validation['encodedFrames']==3508
    assert abs(validation['durationSeconds']-validation['sourceDurationSeconds'])<2/30
    position=np.linalg.norm(exported_estimate[:,:2]-exported_truth[:,:2],axis=1)
    yaw=np.arctan2(np.sin(exported_estimate[:,2]-exported_truth[:,2]),np.cos(exported_estimate[:,2]-exported_truth[:,2]))
    rms=float(np.sqrt(np.mean(position[uniform]**2)));yaw_rms=float(np.rad2deg(np.sqrt(np.mean(yaw[uniform]**2))))
    with (ROOT/'research/mncav_interface_audit_20260916/correction_metrics.csv').open() as f:
        row=next(r for r in csv.DictReader(f) if r['mode']=='per_frame_zero' and r['population']=='uniform_all' and r['method']=='steering_and_lidar_bias')
    assert abs(rms-float(row['positionRmseM']))<1e-8 and abs(yaw_rms-float(row['headingRmseDeg']))<1e-10
    assert (OUT/'decode.log').stat().st_size==0
    checks={'sourceTrajectorySamples':len(tr),'maximumOriginalPoseExportDifferenceMOrRad':maximum_pose_difference,
            'allOriginalStatesPreserved':True,'uniformPositionRmseM':rms,'uniformHeadingRmseDeg':yaw_rms,
            'all1170PerceptionFramesShown':True,'futurePerceptionFrameUsed':False,
            'maximumPerceptionAgeSeconds':float(np.max(ages)),'encodedFrames':len(clock),'completeDecodePassed':True,
            'visualInspection':'Six preview frames and the encoded sample were inspected; original metric map scale is retained.'}
    (DEST/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    for name in ['video_validation.json','cloud_metadata.json','cloud_crosscheck.json','imagery_metadata.json','data_metadata.json']:
        shutil.copy2(OUT/name,DEST/name)
    paths=[ROOT/'output/mncav_interface_audit_20260916/correction_experiment.mat',
           ROOT/'output/mncav_inspva_observer_20260915/experiment.mat',
           ROOT/'output/mississippi_mapping_inspva_20260915/feature_observations.mat',
           ROOT/'data/raw/MissisipiPointClouds.mat',ROOT/'output/saved_perception_inspva_20260915/calls.csv',
           ROOT/'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_front_lidar_inspva_pose_1_1170.csv']
    paths += [p for p in OUT.iterdir() if p.is_file() and p.name not in ['encoding.log','render_progress.json']]
    manifest=[]
    for path in sorted(paths):
        with path.open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
        manifest.append({'path':str(path.relative_to(ROOT)),'bytes':path.stat().st_size,'sha256':digest})
    (DEST/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(checks,indent=2))


if __name__=='__main__':main()
