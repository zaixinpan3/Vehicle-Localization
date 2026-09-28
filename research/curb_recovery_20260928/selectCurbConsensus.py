"""Select a conservative two-ranker recovery without mask expansion."""
from probeCurbContinuation import ROOT,P,OLD,OUT
import numpy as np,pandas as pd,json

a=pd.read_csv(OLD/'curb_predictions.csv');a=a[a.dataset=='mississippi'];b=pd.read_csv(OUT/'classifier_predictions.csv');t=a.merge(b,on=['frame','pillar','finePointCount'],suffixes=('_old','_new'),validate='one_to_one')
oldThreshold=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];dev=t.frame.to_numpy()<=780;y=t.finePointCount.to_numpy();old=t.oof_old.to_numpy();new=t.oof_new.to_numpy();base=dev&(old>=oldThreshold);records=[];best=None
for floor in [.2,.3,.4,.5,.6,.7,.75]:
 eligible=dev&~base&(old>=floor);ix=np.flatnonzero(eligible);order=ix[np.argsort(-new[ix],kind='stable')];n=int(base.sum())+np.arange(1,len(order)+1);fp=int((base&(y==0)).sum())+np.cumsum(y[order]==0);pts=int(y[base].sum())+np.cumsum(y[order]);last=np.r_[new[order][:-1]!=new[order][1:],True];valid=(fp/n<=.055)&last
 if not valid.any():continue
 choices=np.flatnonzero(valid);at=choices[np.argmax(pts[choices])];rec=dict(oldMinimumScore=floor,newMinimumScore=float(new[order[at]]),selected=int(n[at]),false=int(fp[at]),coveredPoints=int(pts[at]),falseFraction=float(fp[at]/n[at]));records.append(rec)
 if best is None or rec['coveredPoints']>best['coveredPoints']:best=rec
print('PREFIX',json.dumps(best),flush=True);assert best
old=t.score_old.to_numpy();new=t.score_new.to_numpy();base=old>=oldThreshold;take=base|((old>=best['oldMinimumScore'])&(new>=best['newMinimumScore']))
den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];validation={}
for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',t.frame.to_numpy()==500,den.frame==500)]:
 use=mask&take;n=int(use.sum());false=int((use&(y==0)).sum());covered=int(y[use].sum());total=int(den.finePointCountInRoi[dmask].sum());validation[name]=dict(selected=n,false=false,falseFraction=false/n,coveredPoints=covered,referencePoints=total,pointCoverage=covered/total)
print('VALIDATION',json.dumps(validation),flush=True)
(P/'consensus_selection.json').write_text(json.dumps(dict(prefixCap=.055,prefixChoice=best,validation=validation,alternatives=records),indent=2)+'\n');t[['frame','pillar','finePointCount']].assign(oldAccepted=base,accepted=take).to_csv(OUT/'consensus_predictions.csv',index=False)
