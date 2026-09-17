#!/usr/bin/env python3
"""Read only required HDF5 coordinates from the existing MATLAB point cache.

uv run --offline --with numpy --with scipy --with h5py python scripts/prepareLocalizationVideoClouds.py
No perception or localization algorithm is re-run. Gray context is thinned;
all saved semantic points are retained before display clipping.
"""
import argparse
import json
import hashlib
import shutil
from pathlib import Path
import time
import h5py
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/localization_video_20260917')
    parser.add_argument('--reuse-from',type=Path,help='Reuse an unchanged perception display cache after checking frame clocks and counts.')
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    if args.reuse_from:
        reuse_cache(out,args.reuse_from);return
    poses=np.genfromtxt(ROOT/'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_front_lidar_inspva_pose_1_1170.csv',
                        delimiter=',',names=True,dtype=None,encoding='utf-8')
    frames=np.genfromtxt(out/'frames.csv',delimiter=',',names=True)
    indices=frames['frame'].astype(int)-1
    assert np.max(np.abs(poses['receiver_time_sec'][indices]-frames['time']))<1e-9
    timer=time.monotonic();clouds={};counts=np.zeros((len(frames),3),int);round_trip=0.;raw_counts=[]
    original=ROOT/'data/raw/MissisipiPointClouds.mat'
    semantic=ROOT/'output/mississippi_mapping_inspva_20260915/feature_observations.mat'
    with h5py.File(original) as raw_file,h5py.File(semantic) as feature_file:
        raw=raw_file['pointClouds'];data=feature_file['featureData']
        calibration=data['frameCalibration'];cal_r=calibration['rotation'][()].T;cal_t=calibration['translation'][()].ravel()
        expected=data['counts'][()].T.astype(int)[indices]
        for display_index,k in enumerate(indices):
            # HDF5 arrays are transposed relative to MATLAB, so C flattening
            # here preserves MATLAB's column-major point ordering.
            pts=np.column_stack([raw_file[raw[name][k,0]][()].ravel() for name in ['x','y','z']]).astype(float)
            ok=np.all(np.isfinite(pts),axis=1)&(pts[:,0]>=-15)&(pts[:,0]<=55)&(np.abs(pts[:,1])<=32)&(pts[:,2]>=-4)&(pts[:,2]<=10)
            pts=pts[ok];before=len(pts)
            if len(pts)>10000:
                # MATLAB round(linspace(1,N,...)) translated to zero indexing.
                sample_indices=np.floor(np.linspace(1,len(pts),10000)+.5).astype(int)-1;pts=pts[sample_indices]
            raw_counts.append([before,len(pts)])
            pts=pts@cal_r.T+cal_t;parts=[np.column_stack((pts,np.zeros(len(pts))))]
            row=poses[k];T=np.array([row[name] for name in ['pose_x_m','pose_y_m','pose_z_m']])
            q=np.array([row[name] for name in ['pose_qx','pose_qy','pose_qz','pose_qw']]);R=Rotation.from_quat(q).as_matrix()
            for j in range(3):
                if expected[display_index,j]==0:continue
                point=feature_file[data['pointsByFeatureFrame'][k,j]][()].T
                assert point.ndim==2 and point.shape[1]==3
                local=(point-T)@R;counts[display_index,j]=len(local)
                round_trip=max(round_trip,float(np.max(np.abs(local@R.T+T-point))))
                parts.append(np.column_stack((local,np.full(len(local),j+1))))
            clouds[f'frame_{k+1:04d}']=np.vstack(parts).astype(np.float32)
            if (k+1)%200==0:print(f'Point cache {display_index+1}/{len(frames)} ({time.monotonic()-timer:.1f} s)',flush=True)
    assert np.array_equal(counts,expected) and round_trip<1e-8
    for j,name in enumerate(['curbPoints','polePoints','trafficSignPoints']):assert np.array_equal(counts[:,j],frames[name])
    np.savez_compressed(out/'display_clouds.npz',**clouds)
    metadata=json.loads((out/'data_metadata.json').read_text())
    metadata.update(maximumSemanticRoundTripM=round_trip,
        cloudExportMethod='Direct HDF5 coordinate-only read, full inverse saved SE(3); deterministic raw-context thinning',
        elapsedSeconds=time.monotonic()-timer)
    (out/'cloud_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    np.savetxt(out/'raw_display_counts.csv',raw_counts,delimiter=',',header='finiteInRoi,displayedGrayPoints',comments='',fmt='%d')
    print(json.dumps(metadata,indent=2))


def reuse_cache(out,previous):
    frames=np.genfromtxt(out/'frames.csv',delimiter=',',names=True)
    old_frames=np.genfromtxt(previous/'frames.csv',delimiter=',',names=True)
    metadata=json.loads((out/'data_metadata.json').read_text())
    old_meta=json.loads((previous/'cloud_metadata.json').read_text())
    assert metadata['perceptionSource']==old_meta['perceptionSource']
    assert metadata['rawDisplayBoundsM']==old_meta['rawDisplayBoundsM']
    assert metadata['rawDisplayPointLimit']==old_meta['rawDisplayPointLimit']
    indices=np.searchsorted(old_frames['frame'],frames['frame'])
    for name in frames.dtype.names:
        if name=='time':assert np.max(abs(frames[name]-old_frames[name][indices]))<1e-9
        else:assert np.array_equal(frames[name],old_frames[name][indices]),name
    source=previous/'display_clouds.npz'
    with np.load(source) as cache:
        for row in frames:
            cloud=cache[f"frame_{int(row['frame']):04d}"]
            for cls,name in enumerate(['curbPoints','polePoints','trafficSignPoints'],1):
                assert np.count_nonzero(cloud[:,3]==cls)==int(row[name])
    shutil.copy2(source,out/'display_clouds.npz')
    counts=np.genfromtxt(previous/'raw_display_counts.csv',delimiter=',',names=True)
    np.savetxt(out/'raw_display_counts.csv',np.column_stack([counts[name][indices] for name in counts.dtype.names]),
        delimiter=',',header=','.join(counts.dtype.names),comments='',fmt='%d')
    with source.open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
    metadata.update(cloudExportMethod='Reuse verified unchanged perception display cache; select only observer-covered frame IDs',
        reusedCache=str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) else str(source),reusedCacheSha256=digest,
        maximumSemanticRoundTripM=old_meta['maximumSemanticRoundTripM'],
        semanticRoundTripProvenance='Inherited unchanged-cache validation; no new SE(3) round-trip computation')
    (out/'cloud_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
