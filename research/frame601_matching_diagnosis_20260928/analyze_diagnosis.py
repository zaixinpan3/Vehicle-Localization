"""Independently verify the frame-601 attribution and draw diagnostic geometry."""
from pathlib import Path
import hashlib
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent


def main():
    base = ROOT / 'research/mississippi_matching_20260928'
    calls = pd.read_csv(base / 'recursive/calls.csv')
    peak = calls.loc[calls.positionErrorM.idxmax()]
    assert peak.frame == 601
    ref = peak[['referenceX', 'referenceY', 'referencePsi']].to_numpy(float)
    pose = peak[['x', 'y', 'psi']].to_numpy(float)
    rotation = np.array([[np.cos(ref[2]), -np.sin(ref[2])], [np.sin(ref[2]), np.cos(ref[2])]])
    body = (pose[:2]-ref[:2]) @ rotation
    yaw = np.arctan2(np.sin(pose[2]-ref[2]), np.cos(pose[2]-ref[2]))
    control = pd.read_csv(DEST / 'controls.csv').set_index('variant')
    np.testing.assert_allclose([np.linalg.norm(body), *body, np.rad2deg(yaw)],
                               control.loc['production_reproduced', ['errorM','forwardErrorM','leftErrorM','yawErrorDeg']].to_numpy(float), atol=1e-8)
    neighborhood = pd.read_csv(DEST / 'source_neighborhood.csv')
    assert neighborhood.reproductionMaxAbs.max() < 1e-7
    assert control.loc['exact_reference_seed', 'errorM'] > .85
    assert abs(control.loc['reference_window_motion','errorM']-.6998) < .0001
    geometry = pd.read_csv(DEST / 'geometry_controls.csv').set_index('variant')
    support = pd.read_csv(DEST / 'support_controls.csv').set_index('variant')
    assert geometry.loc['fine_window_pole_only_replacement','errorM'] > .85
    assert support.loc['restore_curb_only','errorM'] < .28
    assert support.loc['restore_curb_and_map_center_pole_oracle','errorM'] < .031
    pole = json.loads((DEST / 'pole_geometry.json').read_text())
    centers = pd.read_csv(DEST / 'map_pole_observed_centers.csv')
    at601 = centers.set_index('frame').loc[601]
    np.testing.assert_allclose(pole['nearbyFinePoleCenter'][:2],
                               at601[['reference601BodyX','reference601BodyY']].to_numpy(float), atol=1e-8)
    means = centers[['reference601BodyX','reference601BodyY']].to_numpy()
    spread = np.linalg.norm(means[:,None]-means[None,:], axis=2).max()
    source = pd.read_csv(DEST / 'source_geometry.csv')
    targets = pd.read_csv(DEST / 'map_targets.csv')
    pairs = pd.read_csv(DEST / 'correspondences.csv')
    costs = pd.read_csv(DEST / 'class_costs.csv')
    source_manifest = json.loads((base / 'source_manifest.json').read_text())
    changed = [name for name, sha in source_manifest['sources'].items()
               if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != sha]
    assert not changed
    inputs = json.loads((base/'input_manifest.json').read_text())['inputs']
    for record in inputs:
        with (ROOT/record['path']).open('rb') as handle:
            assert hashlib.file_digest(handle,'sha256').hexdigest() == record['sha256'], record['path']
    checks = dict(frame=601, positionErrorM=float(np.linalg.norm(body)), bodyErrorM=body.tolist(),
                  headingErrorDeg=float(np.rad2deg(yaw)), maximumNeighborhoodReproductionResidual=float(neighborhood.reproductionMaxAbs.max()),
                  mapPoleFineCenterOffsetM=float(at601.distanceToMapCenterM),
                  mapPoleCoarseWindowOffsetM=float(np.linalg.norm(np.array(pole['coarseWindowCenter'])-pole['mapCenterInReferenceBody'])),
                  finePointAndMapObservationCenterAgree=True, mapPoleObservationCenterMaximumSeparationM=float(spread),
                  poleRobustWeightFraction=float(pairs.loc[pairs.semanticName=='pole','robustWeight'].sum()/pairs.robustWeight.sum()),
                  referenceObjective=float(costs.loc[costs.poseReference1Solution2==1,'cost'].sum()),
                  fittedObjective=float(costs.loc[costs.poseReference1Solution2==2,'cost'].sum()),
                  recordedProductionSourceHashesUnchanged=True, originalRunInputHashesVerified=len(inputs), checkedNeighborhoodFrames=[585,610],
                  qualifications=['Same-drive detector/map/reference consistency; no surveyed physical pole position.',
                                  'Fine labels, exact reference motion and pole-at-map controls are offline diagnostics.',
                                  'Restoring curb support relaxes precision filtering and is not a deployable precision claim.',
                                  'Underlying physical cause of cross-scan landmark displacement is unresolved.'])
    (DEST/'validation.json').write_text(json.dumps(checks,indent=2)+'\n')
    plot(source,targets,pairs,centers,neighborhood,body,yaw)
    artifacts = []
    for p in sorted(DEST.iterdir()):
        if p.suffix in ('.m','.py','.csv','.json','.png','.pdf','.md') and p.name!='artifact_hashes.json':
            artifacts.append(dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    (DEST/'artifact_hashes.json').write_text(json.dumps(artifacts,indent=2)+'\n')
    print(json.dumps(checks,indent=2))


def plot(source,targets,pairs,centers,neighborhood,body,yaw):
    fig, axes = plt.subplots(2,2,figsize=(14,9),constrained_layout=True)
    ax=axes[0,0]
    for _,row in targets[targets['class']=='curb'].iterrows():
        tangent=np.array([-row.normalY,row.normalX]);xy=np.array([row.x,row.y]);ends=xy+np.array([[-2],[2]])*row.majorStd*tangent
        ax.plot(ends[:,0],ends[:,1],color='black',lw=2)
    curb=source[(source.production1Unfiltered2==1)&(source['class']=='curb')]
    extra=source[(source.production1Unfiltered2==2)&(source['class']=='curb')]
    ax.scatter(extra.x,extra.y,facecolors='none',edgecolors='#AAAAAA',s=35,label='Before precision filter')
    ax.scatter(curb.x,curb.y,color='#008CA8',s=35,label='Current curb at reference pose')
    pole=targets[targets['class']=='pole'].iloc[0]
    q=pairs[pairs.semanticName=='pole'].iloc[0]
    ax.scatter([pole.x],[pole.y],marker='*',s=150,color='black',label='Map pole')
    ax.scatter([q.sourceBody_1],[q.sourceBody_2],marker='o',s=60,color='#E07A10',label='Current pole at reference pose')
    rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    fitted=np.column_stack([curb.x,curb.y])@rot.T+body
    ax.scatter(fitted[:,0],fitted[:,1],marker='x',color='#C22855',s=40,label='Current curb at fitted pose')
    ax.set(xlim=(1,24),ylim=(-5,-1.5),xlabel='Forward (m)',ylabel='Left (m)',title='Short curb segment + a displaced pole')
    ax.legend(fontsize=8,ncol=2,loc='upper right')
    ax=axes[0,1]
    points=ax.scatter(centers.reference601BodyX,centers.reference601BodyY,c=centers.frame,cmap='viridis',s=28)
    ax.plot(centers.reference601BodyX,centers.reference601BodyY,color='gray',alpha=.35)
    ax.scatter([pole.x],[pole.y],marker='*',s=170,color='black',label='Map component mean')
    c601=centers[centers.frame==601].iloc[0]
    ax.scatter([c601.reference601BodyX],[c601.reference601BodyY],marker='o',s=100,facecolors='none',edgecolors='red',label='Frame 601 fine points')
    ax.set_aspect('equal',adjustable='datalim');ax.set(xlabel='Forward in reference-601 frame (m)',ylabel='Left (m)',title='Same pole component: cross-scan fine-point centers')
    fig.colorbar(points,ax=ax,label='Frame');ax.legend(fontsize=8)
    ax=axes[1,0]
    ax.plot(neighborhood.frame,neighborhood.curb,'o-',label='Curb distributions')
    ax.plot(neighborhood.frame,neighborhood.pole,'o-',label='Pole distributions')
    ax.plot(neighborhood.frame,neighborhood.sign,'o-',label='Sign distributions')
    ax.axvline(601,color='red',ls='--',lw=1);ax.set(xlabel='Frame',ylabel='Confirmed components',title='Loss and return of matching support');ax.legend(fontsize=8)
    ax=axes[1,1]
    names=['Production','Exact seed','Exact window motion','Fine pole replacement','Fine curb replacement','Restore curb support','Pole center oracle','Curb + pole oracle']
    # Read measured controls rather than using rounded display values.
    c=pd.read_csv(DEST/'controls.csv').set_index('variant');g=pd.read_csv(DEST/'geometry_controls.csv').set_index('variant');u=pd.read_csv(DEST/'support_controls.csv').set_index('variant')
    values=[c.loc['production_reproduced','errorM'],c.loc['exact_reference_seed','errorM'],c.loc['reference_window_motion','errorM'],g.loc['fine_window_pole_only_replacement','errorM'],g.loc['fine_window_curb_only_replacement','errorM'],u.loc['restore_curb_only','errorM'],g.loc['pole_center_at_map_oracle','errorM'],u.loc['restore_curb_and_map_center_pole_oracle','errorM']]
    ax.barh(names,np.array(values)*100,color=['#155B9A']*6+['#C17C21']*2);ax.invert_yaxis();ax.set(xlabel='Candidate position error (cm)',title='Offline controls; orange uses map-center oracle')
    for i,x in enumerate(values):ax.text(x*100+1,i,f'{x*100:.1f}',va='center',fontsize=8)
    ax.set_xlim(0,100)
    for ax in axes.flat:ax.grid(alpha=.18)
    fig.suptitle('Frame 601: 85.2 cm error from weak curb support and source/map geometric disagreement\nControlled diagnosis only; production settings unchanged',fontsize=13)
    for ext in ('png','pdf'):fig.savefig(DEST/f'diagnosis.{ext}',dpi=160)
    plt.close(fig)


if __name__=='__main__':main()
