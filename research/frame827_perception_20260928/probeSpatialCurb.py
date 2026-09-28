"""Test signed neighboring pillar statistics on a purged training prefix."""
from pathlib import Path
import os
os.environ['OMP_NUM_THREADS']='2'
import json,pickle
import numpy as np,pandas as pd
from sklearn.ensemble import HistGradientBoostingClassifier
ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent;OUT=ROOT/'output/frame827_perception_20260928'
FIELDS=['ground_heightMap','ground_roughnessMap','energy_relativeHeightMeters','energy_heightStepMeters','energy_totalBase','raw_planeRms','raw_stepOwnMiddleFraction','raw_ownerContextQ50','raw_zQ50']
OFFSETS=[(dx,dy) for dx in [-1,0,1] for dy in [-1,0,1] if dx or dy]
def augment(t):
 names=list(t.columns[3:]);x=t[names].to_numpy();ids=t.pillar.to_numpy().astype(int);frames=t.frame.to_numpy().astype(int)
 lookup=pd.Index(frames*10000+ids);row=(ids-1)%100;col=(ids-1)//100;pieces=[x];extra=[]
 for dx,dy in OFFSETS:
  key=frames*10000+ids+dx*100+dy;at=lookup.get_indexer(key);valid=(at>=0)&(row+dy>=0)&(row+dy<100)&(col+dx>=0)&(col+dx<100);safe=np.maximum(at,0)
  delta=x[safe][:,[names.index(f) for f in FIELDS]]-x[:,[names.index(f) for f in FIELDS]];delta[~valid]=0
  pieces.extend([valid[:,None],delta]);tag=f'neighbor_{dx+1}_{dy+1}';extra += [tag+'_present']+[tag+'_'+f for f in FIELDS]
 return np.floor(np.column_stack(pieces)*1e8+.5)/1e8,names+extra

def main():
 t=pd.read_csv(ROOT/'output/semantic_precision_20260927/mississippi_curb_features.csv');X,names=augment(t);frame=t.frame.to_numpy();label=t.finePointCount.to_numpy();y=label>0;dev=frame<=780;fold=np.minimum(4,((frame-1)*5/780).astype(int));oof=np.full(len(t),-np.inf)
 def fit(mask):
  w=np.ones(mask.sum());z=y[mask];w[z]=np.sqrt(1+np.minimum(label[mask][z],100)/10);w[z]/=w[z].sum();w[~z]/=(~z).sum();w/=w.mean()
  return HistGradientBoostingClassifier(max_iter=250,max_depth=6,max_leaf_nodes=31,min_samples_leaf=25,learning_rate=.05,l2_regularization=2,early_stopping=False,random_state=928).fit(X[mask],y[mask],sample_weight=w)
 for k in range(5):
  test=dev&(fold==k);fs=frame[test];train=dev&((frame<fs.min()-20)|(frame>fs.max()+20));m=fit(train);oof[test]=m.predict_proba(X[test])[:,1];print('Fold',k,'done',flush=True)
 order=np.flatnonzero(dev);order=order[np.argsort(-oof[order],kind='stable')];count=np.arange(1,len(order)+1);false=np.cumsum(~y[order]);covered=np.cumsum(label[order]);last=np.r_[oof[order][:-1]!=oof[order][1:],True];choices=np.flatnonzero((false/count<=.05)&(count>=30)&last);at=choices[np.argmax(covered[choices])];threshold=float(oof[order[at]])
 choice=dict(threshold=threshold,selected=int(count[at]),false=int(false[at]),covered=int(covered[at]),fields=FIELDS,offsets=OFFSETS)
 m=fit(dev);score=m.predict_proba(X)[:,1];t[['frame','pillar','finePointCount']].assign(score=score,oof=oof).to_csv(OUT/'spatial_predictions.csv',index=False)
 with (OUT/'spatial.pickle').open('wb') as f:pickle.dump(dict(model=m,names=names,choice=choice),f)
 results={}
 for name,mask in [('prefixOOF',dev),('suffix',~dev),('frame827',frame==827)]:
  v=oof if name=='prefixOOF' else score;s=mask&(v>=threshold);results[name]=dict(selected=int(s.sum()),false=int((s&~y).sum()),covered=int(label[s].sum()))
 (DEST/'spatial_probe.json').write_text(json.dumps(dict(choice=choice,results=results),indent=2)+'\n');print(json.dumps(results),flush=True)
if __name__=='__main__':main()
