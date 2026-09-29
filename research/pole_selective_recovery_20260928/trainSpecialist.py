"""Fit a Mississippi-specific pole discriminator, selecting on purged prefix OOF."""
from pathlib import Path
import os,sys,json,pickle
os.environ['OMP_NUM_THREADS']='1'
import numpy as np
import pandas as pd
from sklearn.ensemble import HistGradientBoostingClassifier
P=Path(__file__).resolve().parent;ROOT=P.parents[1];OUT=ROOT/'output/pole_selective_recovery_20260928';OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(ROOT/'research/pole_geometry_20260927'))
from trainGeometryModel import load,COMPACT
rows,den,names,X,ds,frames,labels,y,ix,groups,dev=load(True,True)
use=ds=='Mississippi';X=X[use];frames=frames[use];labels=labels[use];y=y[use];rows=[r for r,k in zip(rows,use) if k]
keys=np.array([f"{r['frame']}:{r['pillar']}" for r in rows]);_,ix,groups=np.unique(keys,return_index=True,return_inverse=True)
fields=COMPACT+names[names.index('moment001'):];X=X[:,[names.index(n) for n in fields]]
prefix=frames<=780;folds=np.minimum(4,((frames-1)*5/780).astype(int));target=labels[ix];ownerPrefix=frames[ix]<=780
results=[];bundles=[]
for depth,leaf,power in [(3,20,.5),(5,20,.5),(5,10,1),(7,20,1),(3,10,1),(5,20,1)]:
    def factory():return HistGradientBoostingClassifier(max_iter=250,max_depth=depth,max_leaf_nodes=31,min_samples_leaf=leaf,learning_rate=.05,l2_regularization=2,early_stopping=False,random_state=928)
    def weights(mask):
        yy=y[mask];w=np.ones(mask.sum());w[yy]=(1+labels[mask][yy]/20)**power;w[yy]*=.5/w[yy].sum();w[~yy]=.5/(~yy).sum();w/=np.bincount(groups[mask],minlength=len(ix))[groups[mask]];return w/w.mean()
    oof=np.full(len(y),-np.inf)
    for fold in range(5):
        test=prefix&(folds==fold);ff=frames[test];train=prefix&((frames<ff.min()-20)|(frames>ff.max()+20))
        model=factory();model.fit(X[train],y[train],sample_weight=weights(train));oof[test]=model.predict_proba(X[test])[:,1]
    score=np.full(len(ix),-np.inf);np.maximum.at(score,groups,oof)
    order=np.flatnonzero(ownerPrefix);order=order[np.argsort(-score[order])];values=score[order];bad=np.cumsum(target[order]==0);count=np.arange(1,len(order)+1);coverage=np.cumsum(target[order])
    valid=(bad/count<=.075)&np.r_[values[:-1]!=values[1:],True]&(count>=100)
    best=np.flatnonzero(valid)[np.argmax(coverage[valid])];threshold=float(values[best])
    result=dict(depth=depth,leaf=leaf,power=power,threshold=threshold,selected=int(count[best]),empty=int(bad[best]),covered=int(coverage[best]),falseFraction=float(bad[best]/count[best]))
    results.append(result);print('OOF',result,flush=True)
    model=factory();model.fit(X[prefix],y[prefix],sample_weight=weights(prefix));bundles.append(model)
    np.save(OUT/f'oof_d{depth}_l{leaf}_w{power}.npy',oof)
idx=max(range(len(results)),key=lambda k:(results[k]['covered'],-results[k]['empty'],-results[k]['depth']));chosen=results[idx];model=bundles[idx]
(P/'specialist_selection.json').write_text(json.dumps(dict(chosen=chosen,alternatives=results,seed=928,split='Mississippi1:780, five contiguous folds,20-frame purge; maximize covered points under7.5% OOF reference-empty selections'),indent=2)+'\n')
with (OUT/'specialist.pickle').open('wb') as f:pickle.dump(dict(model=model,names=fields),f)
score=model.predict_proba(X)[:,1];ownerScore=np.full(len(ix),-np.inf);np.maximum.at(ownerScore,groups,score)
threshold=chosen['threshold'];pred=pd.DataFrame([dict(frame=int(rows[j]['frame']),pillar=int(rows[j]['pillar']),finePointCount=int(target[k]),score=float(ownerScore[k]),selected=bool(ownerScore[k]>=threshold)) for k,j in enumerate(ix)])
pred.to_csv(P/'specialist_expected.csv',index=False)
metrics=[]
for part,sel in [('all',np.ones(len(pred),bool)),('prefix',pred.frame<=780),('suffix',pred.frame>780)]:
    a=pred[sel&pred.selected];total=sum(int(r['finePoints']) for r in den if r['dataset']=='Mississippi' and (part=='all' or (int(r['frame'])<=780)==(part=='prefix')))
    metrics.append(dict(part=part,selected=len(a),empty=int((a.finePointCount==0).sum()),covered=int(a.finePointCount.sum()),fine=total,falseFraction=float((a.finePointCount==0).mean()),coverage=float(a.finePointCount.sum()/total)))
pd.DataFrame(metrics).to_csv(P/'specialist_metrics.csv',index=False);print('CHOSEN',chosen);print(pd.DataFrame(metrics));print(pred[(pred.frame==857)&pred.selected])
used=sorted({int(r['feature_idx']) for p in model._predictors for r in p[0].nodes if not r['is_leaf']});remap={v:i+1 for i,v in enumerate(used)}
a=dict(schemaVersion=1,featureScale=1e8,featureNames=[fields[i] for i in used],decisionThreshold=threshold,initialLogit=float(model._baseline_prediction[0,0]),roots=[],feature=[],threshold=[],left=[],right=[],value=[],leaf=[],model='mississippiPoleSpecialist928',training='Mississippi <=780;5blocks20framepurge;seed928;fine labels offline only')
for predictors in model._predictors:
    n=predictors[0].nodes;offset=len(a['feature']);a['roots'].append(offset+1);a['feature'] += [remap[int(r['feature_idx'])] if not r['is_leaf'] else 1 for r in n];a['threshold']+=n['num_threshold'].tolist();a['left']+=(n['left'].astype(int)+offset+1).tolist();a['right']+=(n['right'].astype(int)+offset+1).tolist();a['value']+=n['value'].tolist();a['leaf']+=n['is_leaf'].astype(bool).tolist()
(OUT/'model.json').write_text(json.dumps(a,separators=(',',':'))+'\n')
np.savetxt(OUT/'inference_inputs.csv',X[:,used],delimiter=',');np.savetxt(OUT/'inference_expected.csv',score,delimiter=',')
