"""Audit the production replay, compare the frozen baseline and plot correction."""
from pathlib import Path
import hashlib,json,shutil
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
DEST=Path(__file__).resolve().parent
OUT=ROOT/'output/line_direction_matching_20260928'

def score(c):
    e=np.hypot(c.x-c.referenceX,c.y-c.referenceY)
    yaw=np.rad2deg(np.arctan2(np.sin(c.psi-c.referencePsi),np.cos(c.psi-c.referencePsi)))
    np.testing.assert_allclose(e,c.positionErrorM,atol=1e-8,rtol=0)
    np.testing.assert_allclose(yaw,c.yawErrorDeg,atol=1e-10,rtol=0)
    return dict(frames=len(c),accepted=int(c.accepted.sum()),directional=int(c.directionalAccepted.sum()),
                noUpdate=int((~(c.accepted.astype(bool)|c.directionalAccepted.astype(bool))).sum()),
                rmseM=float(np.sqrt(np.mean(e**2))),p95M=float(np.percentile(e,95)),maximumM=float(e.max()),
                maximumAfterStartupM=float(e.iloc[1:].max()),yawRmseDeg=float(np.sqrt(np.mean(yaw**2))),
                totalMedianMs=float(c.totalMs.median()),totalP95Ms=float(np.percentile(c.totalMs,95)),
                perceptionMedianMs=float(c.perceptionMs.median()),registrationMedianMs=float(c.registrationMs.median()))

def main():
    baseline=pd.read_csv(ROOT/'research/mississippi_matching_20260928/recursive/calls.csv')
    current=pd.read_csv(OUT/'production/calls.csv')
    assert np.array_equal(current.frame,np.arange(1,1171))
    for field in ['referenceX','referenceY','referencePsi','timeSeconds']:
        np.testing.assert_allclose(current[field],baseline[field],atol=1e-10,rtol=0)
    expected=pd.read_csv(DEST/'direction_association.csv')
    difference=float(np.max(np.abs(current[['x','y','psi']].to_numpy()-expected[['x','y','psi']].to_numpy())))
    assert difference<1e-7,difference
    np.testing.assert_array_equal(current.accepted,expected.accepted)
    old=score(baseline);new=score(current)
    c=current.set_index('frame').loc[601];b=baseline.set_index('frame').loc[601]
    assert c.accepted and c.positionErrorM<.21 and abs(c.yawErrorDeg)<.8
    tests=pd.read_csv(DEST/'tests.csv');assert tests.Passed.astype(bool).all()
    manifest=json.loads((OUT/'production_source_manifest.json').read_text())
    for name,digest in manifest['sources'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    original=json.loads((ROOT/'research/mississippi_matching_20260928/source_manifest.json').read_text())
    stable=[name for name in original['sources'] if name.startswith(('perception/','mapping/','config/')) and name!='config/distributionRegistrationConfig.m']
    assert all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==original['sources'][name] for name in stable)
    inputs=json.loads((ROOT/'research/mississippi_matching_20260928/input_manifest.json').read_text())['inputs']
    for x in inputs:
        with (ROOT/x['path']).open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==x['sha256'],x['path']
    summary=dict(baseline=old,production=new,frame601=dict(beforeM=float(b.positionErrorM),afterM=float(c.positionErrorM),
        beforeYawDeg=float(b.yawErrorDeg),afterYawDeg=float(c.yawErrorDeg),accepted=True,
        errorReductionFraction=float(1-c.positionErrorM/b.positionErrorM)),tests=dict(passed=int(tests.Passed.sum()),failed=int(tests.Failed.sum()),incomplete=int(tests.Incomplete.sum())),
        validation=dict(prototypeProductionMaximumPoseDifference=difference,executedSourceHashesUnchanged=True,
                        unchangedPerceptionMappingAndConfigurationFiles=len(stable),originalInputFilesVerified=len(inputs),
                        positionReferenceUsedOnlineAfterInitialization=False,knownInsTiltUsed=True,perceptionPrecisionGatesUnchanged=True),
        frameChanges=dict(improved=int((current.positionErrorM<baseline.positionErrorM-1e-6).sum()),worsened=int((current.positionErrorM>baseline.positionErrorM+1e-6).sum())),
        qualification='Same-drive tuning and validation; map includes query observations. No independent physical accuracy or new-drive test. No fine labels or reference positions supplied to the new matching factor.')
    (DEST/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for filename in ['calls.csv','summary.json','metadata.json','source_windows.csv','dead_reckoning.csv']:
        (DEST/'production').mkdir(exist_ok=True);shutil.copy2(OUT/'production'/filename,DEST/'production'/filename)
    shutil.copy2(OUT/'production_source_manifest.json',DEST/'source_manifest.json')
    plot(baseline,current)
    print(json.dumps(summary,indent=2))

def plot(before,after):
    fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
    for table,label,color in [(before,'Before','#C54F39'),(after,'With curb direction','#167DA5')]:
        region=table[table.frame.between(580,615)]
        axes[0,0].plot(region.frame,region.positionErrorM*100,'o-',ms=3,label=label,color=color)
        axes[0,1].plot(region.frame,region.yawErrorDeg,'o-',ms=3,color=color)
        axes[1,0].plot(table.frame,table.positionErrorM*100,lw=.9,label=label,color=color)
    axes[0,0].set(xlabel='Frame',ylabel='Position error (cm)',title='Frame 601 and neighboring scans');axes[0,0].legend()
    axes[0,1].set(xlabel='Frame',ylabel='Heading error (degrees)',title='Suppressing the biased rotation')
    axes[1,0].set(xlabel='Frame',ylabel='Position error (cm)',title='All 1,170 outputs; no frame exclusion')
    pairs=pd.read_csv(DEST/'frame601_pairs.csv');targets=pd.read_csv(ROOT/'research/frame601_matching_diagnosis_20260928/map_targets.csv')
    for _,row in targets[targets['class']=='curb'].iterrows():
        xy=np.array([row.x,row.y]);tangent=np.array([-row.normalY,row.normalX]);segment=xy+np.array([[-2],[2]])*row.majorStd*tangent
        axes[1,1].plot(segment[:,0],segment[:,1],color='black',lw=2)
    selected=pairs.semanticName=='curb';xy=pairs.loc[selected,['sourceMean_1','sourceMean_2']].to_numpy();a=before.set_index('frame').loc[601]
    ref=np.array([a.referenceX,a.referenceY]);rot=np.array([[np.cos(a.referencePsi),-np.sin(a.referencePsi)],[np.sin(a.referencePsi),np.cos(a.referencePsi)]])
    for table,label,color in [(before,'Before','#C54F39'),(after,'After','#167DA5')]:
        c=table.set_index('frame').loc[601];d=np.array([c.x,c.y])-ref;d=d@rot;yaw=c.psi-c.referencePsi;r=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        q=xy@r.T+d;axes[1,1].scatter(q[:,0],q[:,1],marker='x',s=40,color=color,label=label)
    axes[1,1].scatter(xy[:,0],xy[:,1],facecolors='none',edgecolors='gray',s=30,label='At reference pose')
    axes[1,1].set(xlabel='Forward in frame-601 reference axes (m)',ylabel='Left (m)',title='Same selected curbs; changed pose estimate',xlim=(1,15),ylim=(-4,-2));axes[1,1].legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.suptitle('Curb direction in correspondence selection and pose optimization\nSame perception pillars and frozen map; no reference-position resets')
    for ext in ('png','pdf'):fig.savefig(DEST/f'correction.{ext}',dpi=160)
    plt.close(fig)

if __name__=='__main__':main()
