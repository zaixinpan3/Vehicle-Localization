"""Select bounded, direction-aware curb continuation from prefix OOF scores."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent;OLD=ROOT/'output/semantic_precision_20260927';OUT=ROOT/'output/curb_recovery_20260928'

def support(t,scores,threshold,radius,normalTolerance,angleCos):
 result=np.zeros(len(t),int);bilateral=np.zeros(len(t),bool)
 row=(t.pillar.to_numpy()-1)%100;col=(t.pillar.to_numpy()-1)//100
 theta=t.energy_linearityThetaRadians.to_numpy();z=t.ground_heightMap.to_numpy()
 for indices in t.groupby('frame',sort=False).indices.values():
  seed=indices[scores[indices]>=threshold]
  if not len(seed):continue
  dx=.6*(col[seed][None,:]-col[indices][:,None]);dy=.6*(row[seed][None,:]-row[indices][:,None]);distance=np.hypot(dx,dy)
  tangent=dx*np.cos(theta[indices,None])+dy*np.sin(theta[indices,None]);normal=-dx*np.sin(theta[indices,None])+dy*np.cos(theta[indices,None])
  compatible=(distance>0)&(distance<=radius+1e-9)&(abs(normal)<=normalTolerance)&(abs(np.cos(theta[indices,None]-theta[seed][None,:]))>=angleCos)&(abs(z[indices,None]-z[seed][None,:])<=.15+.05*distance)
  result[indices]=compatible.sum(axis=1);bilateral[indices]=np.any(compatible&(tangent>0),axis=1)&np.any(compatible&(tangent<0),axis=1)
 return result,bilateral

def main():
 table=pd.read_csv(OLD/'mississippi_curb_features.csv');pred=pd.read_csv(OLD/'curb_predictions.csv');pred=pred[pred.dataset=='mississippi'];t=table.merge(pred[['frame','pillar','score','oof']],on=['frame','pillar'],validate='one_to_one')
 threshold=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];dev=t.frame.to_numpy()<=780;y=t.finePointCount.to_numpy();base=dev&(t.oof.to_numpy()>=threshold)
 # Fixed margin below the requested 10% limit, selected before suffix evaluation.
 cap=.06;records=[];best=None
 for radius in [1.2,1.8,2.4]:
  for normal in [.3,.45,.6]:
   count,both=support(t,t.oof.to_numpy(),threshold,radius,normal,np.cos(np.pi/4))
   for mode in ['one','two','bilateral']:
    eligibility=(count>=(1 if mode=='one' else 2))
    if mode=='bilateral':eligibility &= both
    for low in [.4,.5,.6,.65,.7,.75,.8]:
     take=base | (dev&eligibility&(t.oof.to_numpy()>=low));n=take.sum();fp=(take&(y==0)).sum();covered=y[take].sum()
     rec=dict(radius=radius,normalTolerance=normal,minimumScore=low,mode=mode,selected=int(n),false=int(fp),covered=int(covered),falseFraction=float(fp/n));records.append(rec)
     if fp/n<=cap and (best is None or covered>best['covered']):best=rec
 print('PREFIX',json.dumps(best),flush=True);assert best
 count,both=support(t,t.score.to_numpy(),threshold,best['radius'],best['normalTolerance'],np.cos(np.pi/4));eligible=count>=(1 if best['mode']=='one' else 2)
 if best['mode']=='bilateral':eligible &= both
 old=t.score.to_numpy()>=threshold;take=old | (eligible&(t.score.to_numpy()>=best['minimumScore']))
 den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];validation={}
 for name,mask,denmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',t.frame.to_numpy()==500,den.frame==500)]:
  sel=mask&take;n=int(sel.sum());fp=int((sel&(y==0)).sum());points=int(y[sel].sum());total=int(den.finePointCountInRoi[denmask].sum());validation[name]=dict(selected=n,false=fp,falseFraction=fp/n,coveredPoints=points,referencePoints=total,pointCoverage=points/total)
 print('VALIDATION',json.dumps(validation),flush=True)
 (P/'continuation_search.json').write_text(json.dumps(dict(cap=cap,prefixChoice=best,validation=validation,alternatives=records),indent=2)+'\n')
 t[['frame','pillar','finePointCount']].assign(oldAccepted=old,accepted=take).to_csv(OUT/'continuation_predictions.csv',index=False)
if __name__=='__main__':main()
