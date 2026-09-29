"""Diagnostic within-track timing/origin fit, trained on frames 1--585 only.

This estimates sequence-specific consistency parameters, not surveyed mounting
or verified absolute sensor latency. Held-out scores refit no global parameter.
"""
from pathlib import Path
import json
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[2]
DEST=Path(__file__).resolve().parent

def design(data):
    poses=pd.read_csv(ROOT/'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_front_lidar_synchronized_pose_1_1170.csv').set_index('frame_index').loc[data.frame]
    w,x,y,z=[poses['pose_q'+s].to_numpy() for s in ['w','x','y','z']]
    r=np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*y+z*w),1-2*(x*x+z*z)],axis=1).reshape(-1,2,2)
    derivative=data[['derivativeX','derivativeY']].to_numpy()
    return np.concatenate([r,-derivative[:,:,None]],axis=2)

def center(values,groups,weights):
    result=np.zeros_like(values)
    for g in np.unique(groups):
        k=groups==g;result[k]=values[k]-np.average(values[k],axis=0,weights=weights[k])
    return result

def score(data,theta):
    count=data.groupby('track').frame.transform('size');d=data[count>=5]
    a=design(d);y=d[['zeroPhaseX','zeroPhaseY']].to_numpy();g=d.track.to_numpy();weights=np.ones(len(d))
    original=d[['originalX','originalY']].to_numpy()
    corrected=y+np.einsum('nij,j->ni',a,theta)
    before=center(original,g,weights);after=center(corrected,g,weights)
    return dict(observations=len(d),tracks=len(np.unique(g)),originalRmsM=float(np.sqrt(np.mean(np.sum(before**2,axis=1)))),correctedRmsM=float(np.sqrt(np.mean(np.sum(after**2,axis=1)))))

def main():
    data=pd.read_csv(DEST/'calibration_tracks.csv');train=data[data.frame<=585].copy()
    train=train[train.groupby('track').frame.transform('size')>=5]
    y=train[['zeroPhaseX','zeroPhaseY']].to_numpy();a=design(train);groups=train.track.to_numpy();report=[]
    for kind,columns,theta in [('phaseOnly',[2],np.zeros(3)),('originAtMidScan',[0,1],np.array([0.,0.,.05])),('joint',[0,1,2],np.zeros(3))]:
        weights=np.ones(len(train));base=theta.copy()
        for _ in range(15):
            yc=center(y+np.einsum('nij,j->ni',a,base),groups,weights);ac=center(a[:,:,columns],groups,weights)
            A=ac.reshape(-1,len(columns))*np.repeat(np.sqrt(weights),2)[:,None];b=-yc.ravel()*np.repeat(np.sqrt(weights),2)
            delta=np.linalg.lstsq(A,b,rcond=None)[0];theta=base.copy();theta[columns]+=delta
            residual=center(y+np.einsum('nij,j->ni',a,theta),groups,weights);norm=np.linalg.norm(residual,axis=1);weights=np.minimum(1,.05/np.maximum(norm,1e-12))
        report.append(dict(model=kind,originCorrectionXYM=theta[:2].tolist(),scanPhaseSeconds=float(theta[2]),singularValues=np.linalg.svd(A,compute_uv=False).tolist(),training=score(train,theta),heldOut=score(data[data.frame>585],theta)))
    (DEST/'timing_origin_fit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
