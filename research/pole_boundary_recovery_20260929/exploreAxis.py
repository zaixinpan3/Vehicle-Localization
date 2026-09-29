"""Evaluate owner-independent shaft evidence using purged prefix folds."""
from pathlib import Path
import os,json,pickle
os.environ['OMP_NUM_THREADS']='1'
import numpy as np
import pandas as pd
from sklearn.ensemble import HistGradientBoostingClassifier
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent;OUT=ROOT/'output/pole_boundary_recovery_20260929'
x=pd.read_csv(ROOT/'output/pole_geometry_20260927/mississippi_features.csv')
b=pd.read_csv(ROOT/'research/pole_selective_recovery_20260928/specialist_expected.csv')
# Independent shaft statistics deliberately exclude owner coordinates/moments.
names=list(x.columns[:x.columns.get_loc('ownerCount')]);names=[n for n in names if n not in ['coreMinimumZ','coreMaximumZ']]
X=np.floor(np.nan_to_num(x[names].to_numpy(float))*1e8+.5)/1e8
h=pd.MultiIndex.from_frame(x[['frame','hypothesis']]);_,hi,hg=np.unique(h,return_index=True,return_inverse=True)
u=pd.MultiIndex.from_frame(x[['frame','pillar']]);_,ui,ug=np.unique(u,return_index=True,return_inverse=True)
f=x.frame.to_numpy();label=x.finePointCount.to_numpy();target=label[ui];base=b.set_index(['frame','pillar']).loc[u[ui],'selected'].to_numpy()
baseOOF=np.zeros(len(ui),bool);np.logical_or.at(baseOOF,ug,np.load(ROOT/'output/pole_selective_recovery_20260928/oof_d5_l10_w1.npy')>=.8747229048074372)
folds=np.minimum(4,((f[hi]-1)*5/780).astype(int));prefix=f[hi]<=780
hy=np.zeros(len(hi),int);np.maximum.at(hy,hg,label);hs=np.bincount(hg,weights=label>0)/np.bincount(hg)
# Require direct ownership of a substantial, vertically extended subset.
guard=(x.ownerCount>=5)&(x.ownerHeight>=.8)&(x.ownerSupportFraction>=.3)&(x.ownerFraction>=.15)&(x.axisOwnerDistance<=.1)
results=[]
for definition in ['any','all']:
 for depth in [3,5]:
  y=(hy>0) if definition=='any' else hs==1
  def fit(tr):
   m=HistGradientBoostingClassifier(max_iter=200,max_depth=depth,min_samples_leaf=15,learning_rate=.05,l2_regularization=2,early_stopping=False,random_state=929)
   w=np.where(y[tr],.5/max(y[tr].sum(),1),.5/max((~y[tr]).sum(),1));w/=w.mean();m.fit(X[hi][tr],y[tr],sample_weight=w);return m
  oof=np.full(len(hi),-np.inf)
  for fold in range(5):
   te=prefix&(folds==fold);ff=f[hi][te];tr=prefix&((f[hi]<ff.min()-20)|(f[hi]>ff.max()+20));m=fit(tr);oof[te]=m.predict_proba(X[hi][te])[:,1]
  own=np.full(len(ui),-np.inf);np.maximum.at(own,ug,np.where(guard,oof[hg],-np.inf));prefixOwn=f[ui]<=780
  choices=[]
  for t in np.unique(own[np.isfinite(own)]):
   add=prefixOwn&~baseOOF&(own>=t);n=add.sum();bad=(target[add]==0).sum();keep=prefixOwn&(baseOOF|(own>=t));fp=(target[keep]==0).sum()
   if n>=10 and bad/n<=.1 and fp/keep.sum()<=.08:choices.append((int(target[add].sum()),-int(bad),float(t),int(n)))
  if not choices:print('no operating point',definition,depth,flush=True);continue
  chosen=max(choices);threshold=chosen[2];m=fit(prefix);p=m.predict_proba(X)[:,1]
  row=guard&(p>=threshold);own=np.zeros(len(ui),bool);np.logical_or.at(own,ug,row);new=base|own
  metrics=[]
  for part,sel in [('prefix',f[ui]<=780),('suffix',f[ui]>780),('all',f[ui]>0)]:
   keep=sel&new;added=sel&own&~base;metrics.append(dict(part=part,selected=int(keep.sum()),empty=int((target[keep]==0).sum()),covered=int(target[keep].sum()),added=int(added.sum()),addedEmpty=int((target[added]==0).sum())))
  r=dict(definition=definition,depth=depth,threshold=threshold,oofAddedPoints=chosen[0],oofAddedEmpty=-chosen[1],oofAdded=chosen[3],target856=p[(f==856)&x.pillar.isin([7944,7945])].tolist(),targetSelected=row[(f==856)&x.pillar.isin([7944,7945])].tolist(),metrics=metrics)
  results.append(r);print(json.dumps(r),flush=True)
  with (OUT/f'axis_{definition}_{depth}.pickle').open('wb') as stream:pickle.dump(dict(model=m,names=names,result=r),stream)
  np.save(OUT/f'axis_{definition}_{depth}_oof.npy',oof[hg]);np.save(OUT/f'axis_{definition}_{depth}_scores.npy',p)
(P/'axis_candidates.json').write_text(json.dumps(results,indent=2)+'\n')
