"""Compare distribution descriptors on held-out frame partitions.

The frozen 0.3 m coarse detector supplies weak labels, not physical truth.
No frame identifiers or absolute XY positions enter the model. All scoring
uses the complete reference, including cells absent from candidate rows.
Run after captureAxialSequenceDescriptors. This is an exploratory experiment,
not a runtime dependency. Dependencies: NumPy and scikit-learn.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier
from threadpoolctl import threadpool_limits

parser=argparse.ArgumentParser()
parser.add_argument('--radial',action='store_true')
parser.add_argument('--refit',action='store_true')
args=parser.parse_args()
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/pole_distribution_20260925'
STUDY=ROOT/'research/pole_distribution_20260925'

def read_csv(path):
    with path.open() as stream:
        names=stream.readline().strip().split(',')
    return names,np.loadtxt(path,delimiter=',',skiprows=1,dtype=np.float32)

names,whole=read_csv(ROOT/'output/coarse_lattice_20260924/whole_pillar_descriptors_all.csv')
axis_names,axis=read_csv(OUT/'axial_all.csv')
f=whole[:,0].astype(int); ids=whole[:,1].astype(int); labels=whole[:,2]>0
keys=f*10000+ids; key_size=1172*10000
lookup=np.full(key_size,-1,np.int32);lookup[keys]=np.arange(len(keys))
axis_keys=axis[:,axis_names.index('frame')].astype(int)*10000+axis[:,axis_names.index('pillarIndices')].astype(int)
assert np.array_equal(keys,axis_keys), 'Descriptor rows must share frame and pillar ordering.'
exclude={'frame','elapsed','pillarIndices','centerX','centerY','centerZ','slopeX','slopeY','minimumZ','maximumZ','score'}
axis_columns=[j for j,name in enumerate(axis_names) if name not in exclude]
extra_names=[axis_names[j] for j in axis_columns]
extra=axis[:,axis_columns]; del axis
X=np.column_stack((whole[:,3:],extra)); X[~np.isfinite(X)]=np.nan
feature_names=names[3:]+extra_names
current=np.loadtxt(OUT/'current_cells.csv',delimiter=',',skiprows=1,dtype=int)
current_map=np.zeros(key_size,bool);current_map[current[:,0]*10000+current[:,1]]=True
current_seed=current_map[keys].astype(np.float32)
X=np.column_stack((X,current_seed)); feature_names.append('previousDetectorSeed')
# The exported reference includes positives absent from the input universe.
reference=np.loadtxt(OUT/'baseline_cells.csv',delimiter=',',skiprows=1,dtype=int)
ref_keys=reference[:,0]*10000+reference[:,1]; ref_map=np.zeros(key_size,bool);ref_map[ref_keys]=True
row=(ids-1)%100;col=(ids-1)//100
near=np.zeros(len(ids),bool)
br=(reference[:,1]-1)%100;bc=(reference[:,1]-1)//100;reference_neighbors=[]
for dr in [-1,0,1]:
    for dc in [-1,0,1]:
        valid=(row+dr>=0)&(row+dr<100)&(col+dc>=0)&(col+dc<100)
        near|=valid&ref_map[keys+dr+100*dc]
        valid_ref=(br+dr>=0)&(br+dr<100)&(bc+dc>=0)&(bc+dc<100)
        rows=lookup[ref_keys+dr+100*dc];rows[~valid_ref]=-1
        reference_neighbors.append(rows)
reference_neighbors=np.array(reference_neighbors).T
reference_neighbors[reference_neighbors<0]=len(ids)
reference_exact=lookup[ref_keys];reference_exact[reference_exact<0]=len(ids)

def score(prediction,split,exact=False):
    chosen=prediction&split[f]
    pr=float(np.sum(chosen&(labels if exact else near))/max(1,np.sum(chosen)))
    selected_ref=split[reference[:,0]]
    predicted=np.r_[chosen,False]
    hits=predicted[reference_exact] if exact else predicted[reference_neighbors].any(1)
    re=float(hits[selected_ref].mean())
    return {'precision':pr,'recall':re,'f1':2*pr*re/max(pr+re,1e-12),'detected_cells':int(chosen.sum()),'reference_cells':int(selected_ref.sum())}

frames=np.arange(1172)
train=frames%4==1; validation=frames%4==3; test=frames%2==0
models=[('whole_only',np.arange(31)),('whole_and_height',np.array(list(range(31))+[31+j for j,n in enumerate(extra_names) if n.startswith('whole')])),('whole_and_axis',np.arange(X.shape[1]-1)),('axis_with_seed',np.arange(X.shape[1]))]
if args.radial:
    radial_names,radial=read_csv(OUT/'radial_all.csv')
    assert np.array_equal(radial[:,0],f) and np.array_equal(radial[:,1],ids)
    begin=X.shape[1]; X=np.column_stack((X,radial[:,3:])); del radial
    feature_names += ['radial_'+name for name in radial_names[3:]]
    X[~np.isfinite(X)]=np.nan
    models=[('whole_and_radial',np.array(list(range(31))+list(range(begin,X.shape[1])))),('axis_and_radial',np.arange(X.shape[1]))]
if args.refit:
    assert args.radial, 'Refit uses the radial feature set.'
    prior=json.loads((STUDY/'radial_ablation.json').read_text())
    frozen_threshold=next(item['threshold'] for item in prior if item['name']=='whole_and_radial')
    models=models[:1]
    train=frames%2==1
results=[]
for name,columns in models:
    with threadpool_limits(limits=4):
        model=HistGradientBoostingClassifier(max_iter=250,max_leaf_nodes=31,min_samples_leaf=25,l2_regularization=5,random_state=42,class_weight={False:1,True:10},early_stopping=False)
        model.fit(X[train[f]][:,columns],labels[train[f]])
        probabilities=model.predict_proba(X[:,columns])[:,1]
    candidates=[]
    for threshold in np.arange(.2,.91,.025):
        metrics=score(probabilities>=threshold,validation)
        candidates.append((min(metrics['precision'],metrics['recall']),float(threshold),metrics))
    _,threshold,validation_score=max(candidates,key=lambda item:item[:2])
    if args.refit: threshold=frozen_threshold
    record={'name':name,'threshold':threshold,'features':[feature_names[j] for j in columns]}
    partitions=[('train',train),('former_validation_in_training' if args.refit else 'validation',validation),('test',test)]
    record.update({partition:{'exact':score(probabilities>=threshold,mask,True),'one_cell':score(probabilities>=threshold,mask)} for partition,mask in partitions})
    record['validation_reused_for_refit']=args.refit
    print(json.dumps(record),flush=True);results.append(record)
    np.savez(OUT/f"{name}{'_refit' if args.refit else ''}_predictions.npz",frame=f,pillar=ids,probability=probabilities,threshold=threshold)
    with (STUDY/('radial_refit.json' if args.refit else ('radial_ablation.json' if args.radial else 'descriptor_ablation.json'))).open('w') as stream:json.dump(results,stream,indent=2)
