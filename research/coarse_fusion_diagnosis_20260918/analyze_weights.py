from pathlib import Path
import h5py,numpy as np,json,os
os.chdir(Path(__file__).resolve().parents[2])
p=Path('output/mncav_coarse_localization_20260918/observer/experiment.mat')
with h5py.File(p) as f:
 ref=f['reference'][()].T;valid=f['data/lidar/valid'][()].ravel().astype(bool);L=f['data/lidar/pose'][()].T
 est=f[f['runs'][0,0]]['estimate'];G=est['diagnostics/gnssPositionAtObserverPoint'][()].T
 IL=f['data/lidar/information'][()].transpose(0,2,1);IG=est['diagnostics/gnssInformationAtObserverPoint'][()].transpose(0,2,1)
 WL=np.stack([np.linalg.solve(I+.001*np.eye(3),I)[:2,:2] for I in IL[valid]])
 WG=np.stack([np.linalg.solve(I+16*np.eye(2),I) for I in IG[valid]])
 eL=L[valid,:2]-ref[valid,:2];eG=G[valid]-ref[valid,:2]
 yaw=ref[valid,2];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]).transpose(2,0,1)
 bL=np.einsum('nji,nj->ni',R,eL);bG=np.einsum('nji,nj->ni',R,eG)
 weightsL=np.linalg.eigvalsh(WL);weightsG=np.linalg.eigvalsh(WG)
 j=np.flatnonzero(valid[:-1]&valid[1:]);allL=L[:,:2]-ref[:,:2]
 result=dict(samples=int(valid.sum()),lidarRmseM=float(np.sqrt(np.mean(np.sum(eL**2,axis=1)))),gnssAtFusedObserverPointRmseM=float(np.sqrt(np.mean(np.sum(eG**2,axis=1)))),
 lidarBodyMeanErrorM=np.mean(bL,axis=0).tolist(),gnssBodyMeanErrorM=np.mean(bG,axis=0).tolist(),
 lidarWeightEigenvalueRange=[weightsL.min(),weightsL.max()],lidarWeightEigenvalueMedian=np.median(weightsL,axis=0).tolist(),gnssWeightEigenvalueMedian=np.median(weightsG,axis=0).tolist(),
 medianLidarPositionGainPerSecond=np.median(4*weightsL,axis=0).tolist(),medianGnssPositionGainPerSecond=np.median(4*weightsG,axis=0).tolist(),
 errorCorrelationBodyLongitudinal=float(np.corrcoef(bL[:,0],bG[:,0])[0,1]),errorCorrelationBodyLateral=float(np.corrcoef(bL[:,1],bG[:,1])[0,1]),
 lidarLagOneErrorCorrelationMapXY=[float(np.corrcoef(allL[j,k],allL[j+1,k])[0,1]) for k in [0,1]],
 fractionAcceptedLidarFurtherFromReferenceThanAlignedGnss=float(np.mean(np.linalg.norm(eL,axis=1)>np.linalg.norm(eG,axis=1))))
 print(json.dumps(result,indent=2));Path('output/coarse_fusion_diagnosis_20260918/diagnostics.json').write_text(json.dumps(result,indent=2)+'\n')
