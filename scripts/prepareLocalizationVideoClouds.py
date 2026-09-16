#!/usr/bin/env python3
"""Read only required HDF5 coordinates from the existing MATLAB point cache.

uv run --offline --with numpy --with scipy --with h5py python scripts/prepareLocalizationVideoClouds.py
No perception or localization algorithm is re-run. Gray context is thinned;
all saved semantic points are retained before display clipping.
"""
import argparse
import json
from pathlib import Path
import time
import h5py
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/localization_video_20260916')
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    poses=np.genfromtxt(ROOT/'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_front_lidar_inspva_pose_1_1170.csv',
                        delimiter=',',names=True,dtype=None,encoding='utf-8')
    frames=np.genfromtxt(out/'frames.csv',delimiter=',',names=True)
    trajectory=np.genfromtxt(out/'trajectory.csv',delimiter=',',names=True)
    assert len(poses)==len(frames)==1170 and np.max(np.abs(poses['receiver_time_sec']-frames['time']))<1e-9
    timer=time.monotonic();clouds={};counts=np.zeros((1170,3),int);round_trip=0.;raw_counts=[]
    original=ROOT/'data/raw/MissisipiPointClouds.mat'
    semantic=ROOT/'output/mississippi_mapping_inspva_20260915/feature_observations.mat'
    with h5py.File(original) as raw_file,h5py.File(semantic) as feature_file:
        raw=raw_file['pointClouds'];data=feature_file['featureData']
        calibration=data['frameCalibration'];cal_r=calibration['rotation'][()].T;cal_t=calibration['translation'][()].ravel()
        expected=data['counts'][()].T.astype(int)
        for k in range(1170):
            # HDF5 arrays are transposed relative to MATLAB, so C flattening
            # here preserves MATLAB's column-major point ordering.
            pts=np.column_stack([raw_file[raw[name][k,0]][()].ravel() for name in ['x','y','z']]).astype(float)
            ok=np.all(np.isfinite(pts),axis=1)&(pts[:,0]>=-15)&(pts[:,0]<=55)&(np.abs(pts[:,1])<=32)&(pts[:,2]>=-4)&(pts[:,2]<=10)
            pts=pts[ok];before=len(pts)
            if len(pts)>10000:
                # MATLAB round(linspace(1,N,...)) translated to zero indexing.
                indices=np.floor(np.linspace(1,len(pts),10000)+.5).astype(int)-1;pts=pts[indices]
            raw_counts.append([before,len(pts)])
            pts=pts@cal_r.T+cal_t;parts=[np.column_stack((pts,np.zeros(len(pts))))]
            row=poses[k];T=np.array([row[name] for name in ['pose_x_m','pose_y_m','pose_z_m']])
            q=np.array([row[name] for name in ['pose_qx','pose_qy','pose_qz','pose_qw']]);R=Rotation.from_quat(q).as_matrix()
            for j in range(3):
                if expected[k,j]==0:continue
                point=feature_file[data['pointsByFeatureFrame'][k,j]][()].T
                assert point.ndim==2 and point.shape[1]==3
                local=(point-T)@R;counts[k,j]=len(local)
                round_trip=max(round_trip,float(np.max(np.abs(local@R.T+T-point))))
                parts.append(np.column_stack((local,np.full(len(local),j+1))))
            clouds[f'frame_{k+1:04d}']=np.vstack(parts).astype(np.float32)
            if (k+1)%200==0:print(f'Point cache {k+1}/1170 ({time.monotonic()-timer:.1f} s)',flush=True)
    assert np.array_equal(counts,expected) and round_trip<1e-8
    for j,name in enumerate(['curbPoints','polePoints','trafficSignPoints']):assert np.array_equal(counts[:,j],frames[name])
    np.savez_compressed(out/'display_clouds.npz',**clouds)
    with h5py.File(ROOT/'output/mncav_inspva_observer_20260915/experiment.mat') as source:
        e=source[source['experiments'][0,0]];uniform=e['uniformIndices'][()].ravel().astype(int)-1
    reference=np.column_stack([trajectory[n] for n in ['truthX','truthY','truthYaw']])
    estimate=np.column_stack([trajectory[n] for n in ['estimateX','estimateY','estimateYaw']])
    error=np.linalg.norm(estimate[:,:2]-reference[:,:2],axis=1)
    yaw=np.arctan2(np.sin(estimate[:,2]-reference[:,2]),np.cos(estimate[:,2]-reference[:,2]))
    metadata={'experiment':'steering_and_lidar_bias / per_frame_zero',
              'sourceExperiment':'output/mncav_interface_audit_20260916/correction_experiment.mat',
              'perceptionSource':str(semantic.relative_to(ROOT)),'rawPointSource':str(original.relative_to(ROOT)),
              'coordinates':'EPSG:32615; grid yaw; recorded INS output point',
              'time':'INSPVA receiver seconds relative to first LiDAR frame',
              'frames':1170,'durationSeconds':float(frames['time'][-1]),'maximumSemanticRoundTripM':round_trip,
              'semanticCounts':counts.sum(axis=0).tolist(),'rawDisplayPointLimit':10000,
              'rawDisplayBoundsM':[-15,55,-32,32,-4,10],'semanticSelectionChanged':False,
              'uniformSamples':len(uniform),'uniformPositionRmseM':float(np.sqrt(np.mean(error[uniform]**2))),
              'uniformHeadingRmseDeg':float(np.rad2deg(np.sqrt(np.mean(yaw[uniform]**2)))),
              'estimatorRerun':False,'visualizationReferenceOnly':True,
              'limitations':'Offline replay; same-drive map and reference-seeded matching; imagery is context, not accuracy reference',
              'cloudExportMethod':'Direct HDF5 coordinate-only read, full inverse saved SE(3); deterministic raw-context thinning',
              'elapsedSeconds':time.monotonic()-timer}
    assert abs(metadata['uniformPositionRmseM']-.101402302434833)<1e-8
    (out/'cloud_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    np.savetxt(out/'raw_display_counts.csv',raw_counts,delimiter=',',header='finiteInRoi,displayedGrayPoints',comments='',fmt='%d')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
