"""Select monotone curb recovery from two distribution ranks and local support."""
from probeCurbContinuation import ROOT,P,OLD,OUT,support
import numpy as np,pandas as pd,json

def main():
 t=pd.read_csv(OLD/'mississippi_curb_features.csv');a=pd.read_csv(OLD/'curb_predictions.csv');a=a[a.dataset=='mississippi'];b=pd.read_csv(OUT/'classifier_predictions.csv');t=t.merge(a[['frame','pillar','score','oof']],on=['frame','pillar'],validate='one_to_one').merge(b[['frame','pillar','score','oof']],on=['frame','pillar'],suffixes=('_old','_new'),validate='one_to_one')
 model=json.loads((OUT/'mississippi_curb_model.json').read_text());oldThreshold=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];newThreshold=model['decisionThreshold'];dev=t.frame.to_numpy()<=780;labels=t.finePointCount.to_numpy();old=t.oof_old.to_numpy();new=t.oof_new.to_numpy();base=dev&(old>=oldThreshold);best=None;records=[];cap=.055
 for radius in [1.2,1.8,2.4]:
  for normal in [.3,.45,.6]:
   count,both=support(t,new,newThreshold,radius,normal,np.cos(np.pi/4))
   for floor in [.3,.5,.65,.75]:
    for weak in [.45,.55,.65,.75]:
     for minimumSupport in [1,2]:
      # One pass; newly recovered cells never become recursive seeds.
      recovery=(old>=floor)&(new>=weak)&(count>=minimumSupport);take=base|(dev&recovery);n=int(take.sum());false=int((take&(labels==0)).sum());covered=int(labels[take].sum());rec=dict(radius=radius,normalTolerance=normal,oldMinimumScore=floor,newMinimumScore=weak,minimumSupport=minimumSupport,selected=n,false=false,coveredPoints=covered,falseFraction=false/n)
      records.append(rec)
      if false/n<=cap and (best is None or covered>best['coveredPoints']):best=rec
 assert best;print('PREFIX CHOICE',json.dumps(best),flush=True)
 count,_=support(t,t.score_new.to_numpy(),newThreshold,best['radius'],best['normalTolerance'],np.cos(np.pi/4));old=t.score_old.to_numpy();new=t.score_new.to_numpy();base=old>=oldThreshold;take=base|((old>=best['oldMinimumScore'])&(new>=best['newMinimumScore'])&(count>=best['minimumSupport']))
 den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];validation={}
 for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',t.frame.to_numpy()==500,den.frame==500)]:
  chosen=mask&take;n=int(chosen.sum());f=int((chosen&(labels==0)).sum());pts=int(labels[chosen].sum());total=int(den.finePointCountInRoi[dmask].sum());validation[name]=dict(selected=n,false=f,falseFraction=f/n,coveredPoints=pts,referencePoints=total,pointCoverage=pts/total)
 print('VALIDATION',json.dumps(validation),flush=True)
 settings={k:best[k] for k in ['radius','normalTolerance','oldMinimumScore','newMinimumScore','minimumSupport']};settings.update(seedThreshold=newThreshold,angleCos=float(np.cos(np.pi/4)),heightTolerance=.15,heightRangeSlope=.05)
 (OUT/'recovery_parameters.json').write_text(json.dumps(settings,indent=2)+'\n');(P/'recovery_selection.json').write_text(json.dumps(dict(prefixCap=cap,prefixChoice=best,validation=validation,alternatives=records),indent=2)+'\n')
 t[['frame','pillar','finePointCount']].assign(accepted=take,oldAccepted=base).to_csv(OUT/'recovery_predictions.csv',index=False)
if __name__=='__main__':main()
