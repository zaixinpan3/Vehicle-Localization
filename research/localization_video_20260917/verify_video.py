#!/usr/bin/env python3
"""Audit video source fidelity, actual-state holding and paired metrics.

uv run --offline --with numpy --with h5py --with pillow python research/localization_video_20260917/verify_video.py
FFmpeg decoding and encoded dimensions are checked by the renderer.
"""
import hashlib,importlib.util,json,shutil
from pathlib import Path
import h5py
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'output/localization_video_20260917'
OLD=ROOT/'output/localization_video_20260916';DEST=Path(__file__).resolve().parent


def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def main():
    tr=np.genfromtxt(OUT/'trajectory.csv',delimiter=',',names=True)
    frames=np.genfromtxt(OUT/'frames.csv',delimiter=',',names=True)
    clock=np.genfromtxt(OUT/'video_frame_clock.csv',delimiter=',',names=True)
    metadata=json.loads((OUT/'data_metadata.json').read_text())
    validation=json.loads((OUT/'video_validation.json').read_text())
    source=ROOT/metadata['sourceExperiment']
    with h5py.File(source) as f:
        r=f[f['runs'][0,0]]['estimate']
        estimate=np.column_stack((r['position'][()].T,r['headingUnwrapped'][()].ravel()))
        truth=f['reference'][()].T;t=r['time'][()].ravel()
        wheel_speed=f['data/highRate/longitudinalSpeed'][()].ravel()
        accepted=f['data/lidar/valid'][()].ravel().astype(bool)
        lidar=f['data/lidar/pose'][()].T
    exported_truth=np.column_stack([tr[k] for k in ['truthX','truthY','truthYaw']])
    exported_estimate=np.column_stack([tr[k] for k in ['estimateX','estimateY','estimateYaw']])
    maximum_difference=float(max(np.max(abs(truth-exported_truth)),np.max(abs(estimate-exported_estimate))))
    assert maximum_difference<1e-8 and np.max(abs(t-tr['time']))<1e-10
    assert np.max(abs(wheel_speed-tr['speedMps']))<1e-10
    assert np.array_equal(frames['time'],tr['time']) and len(tr)==1169
    assert np.array_equal(frames['measurementStatus']==1,accepted) and accepted.sum()==1083
    error=np.linalg.norm(estimate[:,:2]-truth[:,:2],axis=1)
    yaw=np.arctan2(np.sin(estimate[:,2]-truth[:,2]),np.cos(estimate[:,2]-truth[:,2]))
    paired_error=np.linalg.norm(lidar[accepted,:2]-truth[accepted,:2],axis=1)
    values=dict(positionRmseM=np.sqrt(np.mean(error**2)),positionMedianM=np.median(error),
        headingRmseDeg=np.rad2deg(np.sqrt(np.mean(yaw**2))),acceptedFusionPositionRmseM=np.sqrt(np.mean(error[accepted]**2)),
        acceptedLidarPositionRmseM=np.sqrt(np.mean(paired_error**2)))
    assert max(abs(metadata[k]-v) for k,v in values.items())<1e-12
    assert np.max(abs(tr['positionErrorM']-error))<1e-12
    selected=np.searchsorted(frames['time'],clock['replayTime']+1e-10,side='right')-1
    assert np.array_equal(frames['frame'][selected],clock['perceptionFrame'])
    assert np.array_equal(selected+1,clock['localizationSample'])
    ages=clock['replayTime']-frames['time'][selected]
    assert ages.min()>-1e-10 and ages.max()<np.max(np.diff(frames['time']))+1e-10
    assert np.max(abs(ages-clock['localizationAgeSeconds']))<1e-10
    assert np.array_equal(clock['localizationAgeSeconds'],clock['perceptionAgeSeconds'])
    assert set(selected)==set(range(1169)) and len(clock)==validation['encodedFrames']
    # Exercise the real display sampler, including points between state times.
    spec=importlib.util.spec_from_file_location('renderer',ROOT/'scripts/renderLocalizationVideo.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);renderer=module.Renderer(OUT)
    for time,index in zip(clock['replayTime'],selected):
        truth_sample,estimate_sample,speed_sample,frame=renderer.sample(time+1e-10)
        assert frame==index
        assert np.array_equal(truth_sample,exported_truth[index]) and np.array_equal(estimate_sample,exported_estimate[index])
        assert speed_sample==tr['speedMps'][index]
    old_manifest=json.loads((ROOT/'research/localization_video_20260916/artifact_manifest.json').read_text())
    for name in ['display_clouds.npz','orthoimagery.jpg','imagery_metadata.json']:
        old_row=next(row for row in old_manifest if row['path']==str((OLD/name).relative_to(ROOT)))
        assert digest(OUT/name)==old_row['sha256']==digest(OLD/name)
    counts=np.zeros(3,dtype=int)
    for i,row in enumerate(frames):
        cloud=renderer.clouds[i]
        for cls,name in enumerate(['curbPoints','polePoints','trafficSignPoints'],1):
            count=np.count_nonzero(cloud[:,3]==cls);assert count==row[name];counts[cls-1]+=count
    assert np.array_equal(counts,metadata['semanticCounts'])
    assert (OUT/'decode.log').stat().st_size==0
    original=np.asarray(Image.open(OUT/'preview_050.00s.png'),dtype=float)
    decoded=np.asarray(Image.open(OUT/'decoded_050s.png'),dtype=float)
    psnr=float(10*np.log10(255**2/np.mean((original-decoded)**2)));assert psnr>30
    fresh=OUT/'cache_rebuild_check'
    with np.load(fresh/'display_clouds.npz') as rebuilt:
        assert len(rebuilt.files)==len(frames)
        for index,row in enumerate(frames):
            assert np.array_equal(rebuilt[f"frame_{int(row['frame']):04d}"],renderer.clouds[index])
    assert (fresh/'raw_display_counts.csv').read_bytes()==(OUT/'raw_display_counts.csv').read_bytes()
    fresh_meta=json.loads((fresh/'cloud_metadata.json').read_text())
    crosscheck=dict(coveredCloudsCompared=len(frames),pointArraysBitIdentical=True,grayCountsBitIdentical=True,
        maximumFreshSemanticRoundTripM=fresh_meta['maximumSemanticRoundTripM'],
        method='Compare reused cache against fresh coordinate-only HDF5 export; no perception inference rerun')
    (OUT/'cloud_crosscheck.json').write_text(json.dumps(crosscheck,indent=2)+'\n')
    checks=dict(sourceExperiment=metadata['sourceExperiment'],sourceTrajectorySamples=len(tr),
        maximumOriginalPoseExportDifferenceMOrRad=maximum_difference,allOriginalStatesPreserved=True,
        metrics=values,all1169CoveredPerceptionFramesShown=True,excludedOutsideObserverCoverage=1,
        localizationSampleHoldingVerified=True,futureStateOrPerceptionUsed=False,
        maximumPerceptionAgeSeconds=float(ages.max()),encodedFrames=len(clock),
        semanticCounts=counts.tolist(),perceptionAndImageryCacheHashesUnchanged=True,
        completeDecodePassed=True,encoded50SecondPreviewPsnrDb=psnr,freshCloudExportCrosscheck=crosscheck,
        visualInspection='Six-frame storyboard, full-size 50-second preview and decoded 50-second frame inspected.')
    (DEST/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    for name in ['video_validation.json','data_metadata.json','cloud_metadata.json','imagery_metadata.json','cloud_crosscheck.json']:
        shutil.copy2(OUT/name,DEST/name)
    paths=[source]+[p for p in OUT.iterdir() if p.is_file() and p.name not in ['encoding.log','render_progress.json']]
    manifest=[dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(paths)]
    (DEST/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(checks,indent=2))


if __name__=='__main__':main()
