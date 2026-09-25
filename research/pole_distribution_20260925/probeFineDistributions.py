"""Diagnostic pole-shape discrimination against the separate fine reference.

This reference is not hand-labelled truth. Frozen frame partitions expose
whether richer point distributions predict it; no model is deployed here.
"""
import json
from pathlib import Path
import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier
from threadpoolctl import threadpool_limits

ROOT=Path(__file__).resolve().parents[2]; OUT=ROOT/'output/pole_distribution_20260925'; STUDY=ROOT/'research/pole_distribution_20260925'

def read(path):
    with path.open() as h:names=h.readline().strip().split(',')
    return names,np.loadtxt(path,delimiter=',',skiprows=1,dtype=np.float32)

names,whole=read(ROOT/'output/coarse_lattice_20260924/whole_pillar_descriptors_all.csv')
keep=(whole[:,0].astype(int)-1)%10==0; whole=whole[keep]; f=whole[:,0].astype(int);ids=whole[:,1].astype(int);key=f*10000+ids;size=1172*10000
an,axis=read(OUT/'axial_all.csv');axis=axis[keep]
rn,radial=read(OUT/'radial_all.csv');radial=radial[keep]
exclude={'frame','elapsed','pillarIndices','centerX','centerY','centerZ','slopeX','slopeY','minimumZ','maximumZ','score'}
columns=[j for j,n in enumerate(an) if n not in exclude]
X=np.column_stack((whole[:,3:],axis[:,columns],radial[:,3:]));X[~np.isfinite(X)]=np.nan
features=names[3:]+['axis_'+an[j] for j in columns]+['radial_'+n for n in rn[3:]]
lookup=np.full(size,-1,np.int32);lookup[key]=np.arange(len(key))
ref=np.loadtxt(OUT/'fine_cells.csv',delimiter=',',skiprows=1,dtype=int);rk=ref[:,0]*10000+ref[:,1];refmap=np.zeros(size,bool);refmap[rk]=True;y=refmap[key]
r=(ids-1)%100;c=(ids-1)//100;br=(ref[:,1]-1)%100;bc=(ref[:,1]-1)//100;near=np.zeros(len(ids),bool);ni=[]
for dr in [-1,0,1]:
    for dc in [-1,0,1]:
        valid=(r+dr>=0)&(r+dr<100)&(c+dc>=0)&(c+dc<100);near|=valid&refmap[key+dr+dc*100]
        valid=(br+dr>=0)&(br+dr<100)&(bc+dc>=0)&(bc+dc<100);ind=lookup[rk+dr+dc*100];ind[~valid]=-1;ni.append(ind)
ni=np.array(ni).T;ni[ni<0]=len(ids);exact=lookup[rk];exact[exact<0]=len(ids)

def score(prediction,frame_mask,strict=False):
    pp=prediction & frame_mask[f];pr=float((pp&(y if strict else near)).sum()/max(1,pp.sum()))
    p=np.r_[pp,False];hit=p[exact] if strict else p[ni].any(1);re=float(hit[frame_mask[ref[:,0]]].mean())
    return dict(precision=pr,recall=re,f1=2*pr*re/max(pr+re,1e-9),detected_cells=int(pp.sum()),reference_cells=int(frame_mask[ref[:,0]].sum()))

frames=np.arange(1172);train=((frames-1)//10)%3==0;val=((frames-1)//10)%3==1;test=((frames-1)//10)%3==2
results=[]
for name,cols in [('whole',np.arange(31)),('whole_axis',np.arange(31+len(columns))),('whole_radial',np.r_[np.arange(31),np.arange(31+len(columns),X.shape[1])]),('all_distributions',np.arange(X.shape[1]))]:
    with threadpool_limits(limits=4):
        model=HistGradientBoostingClassifier(max_iter=180,max_leaf_nodes=15,min_samples_leaf=15,l2_regularization=5,random_state=42,class_weight={False:1,True:10},early_stopping=False)
        model.fit(X[train[f]][:,cols],y[train[f]]);p=model.predict_proba(X[:,cols])[:,1]
    trials=[]
    for threshold in np.arange(.1,.91,.025):
        s=score(p>=threshold,val);trials.append((min(s['precision'],s['recall']),float(threshold)))
    _,threshold=max(trials)
    result=dict(name=name,threshold=threshold,features=[features[j] for j in cols])
    result.update({n:dict(exact=score(p>=threshold,mask,True),one_cell=score(p>=threshold,mask)) for n,mask in [('train',train),('validation',val),('test',test)]})
    print(json.dumps(result),flush=True);results.append(result)
    np.savez(OUT/f'fine_{name}_predictions.npz',frame=f,pillar=ids,probability=p,threshold=threshold)
    with (STUDY/'fine_distribution_ablation.json').open('w') as h:json.dump(results,h,indent=2)
