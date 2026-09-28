"""Calibrate whole-pillar ranks by sensing distance on prefix folds."""
from probeCurbContinuation import ROOT,P,OLD,OUT
import numpy as np,pandas as pd,json

a=pd.read_csv(OLD/'curb_predictions.csv');a=a[a.dataset=='mississippi'];b=pd.read_csv(OUT/'classifier_predictions.csv');ranges=pd.read_csv(OLD/'mississippi_curb_features.csv',usecols=['frame','pillar','range']);t=a.merge(b,on=['frame','pillar','finePointCount'],suffixes=('_old','_new'),validate='one_to_one').merge(ranges,on=['frame','pillar'],validate='one_to_one');dev=t.frame.to_numpy()<=780;y=t.finePointCount.to_numpy();distance=t['range'].to_numpy();best=None;records=[]
for model in ['old','new']:
 score=t['oof_'+model].to_numpy()
 for edges in [[10,20],[8,16,24],[10,15,20,25]]:
  band=np.searchsorted(edges,distance,side='right');thresholds=[];sel=np.zeros(len(t),bool)
  for i in range(len(edges)+1):
   ix=np.flatnonzero(dev&(band==i));order=ix[np.argsort(-score[ix],kind='stable')];n=np.arange(1,len(order)+1);fp=np.cumsum(y[order]==0);points=np.cumsum(y[order]);last=np.r_[score[order][:-1]!=score[order][1:],True];valid=(n>=30)&(fp/n<=.05)&last
   if not valid.any():threshold=1.
   else:
    choices=np.flatnonzero(valid);at=choices[np.argmax(points[choices])];threshold=float(score[order[at]])
   thresholds.append(threshold);sel|=dev&(band==i)&(score>=threshold)
  rec=dict(model=model,edges=edges,thresholds=thresholds,selected=int(sel.sum()),false=int((sel&(y==0)).sum()),coveredPoints=int(y[sel].sum()));records.append(rec)
  if best is None or rec['coveredPoints']>best['coveredPoints']:best=rec
print('PREFIX',json.dumps(best),flush=True);score=t['score_'+best['model']].to_numpy();band=np.searchsorted(best['edges'],distance,side='right');take=score>=np.array(best['thresholds'])[band]
den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];result={}
for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',t.frame.to_numpy()==500,den.frame==500)]:
 s=take&mask;n=int(s.sum());false=int((s&(y==0)).sum());pts=int(y[s].sum());total=int(den.finePointCountInRoi[dmask].sum());result[name]=dict(selected=n,false=false,falseFraction=false/n,coveredPoints=pts,referencePoints=total,pointCoverage=pts/total)
print('VALIDATION',json.dumps(result),flush=True);(P/'range_calibration.json').write_text(json.dumps(dict(prefixCap=.05,prefixChoice=best,validation=result,alternatives=records),indent=2)+'\n');t[['frame','pillar','finePointCount']].assign(accepted=take).to_csv(OUT/'range_predictions.csv',index=False)
