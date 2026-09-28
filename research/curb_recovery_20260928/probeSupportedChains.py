"""Require two distribution ranks plus slender connected curb support."""
from probeCurbChains import ROOT,P,OLD,OUT,chain_stats,choose_mask
import numpy as np,pandas as pd,json

def main():
 a=pd.read_csv(OLD/'curb_predictions.csv');a=a[a.dataset=='mississippi'];b=pd.read_csv(OUT/'classifier_predictions.csv');t=a.merge(b,on=['frame','pillar','finePointCount'],suffixes=('_old','_new'),validate='one_to_one');strong=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];y=t.finePointCount.to_numpy();dev=t.frame.to_numpy()<=780;old=t.oof_old.to_numpy();new=t.oof_new.to_numpy();base=old>=strong;best=None;records=[]
 for weak in [.5,.55,.6,.65,.7]:
  for secondary in [.4,.55,.65,.75]:
   adjusted=np.where(base|((old>=weak)&(new>=secondary)),np.where(base,strong,weak),-np.inf);stats=chain_stats(t,adjusted,strong,weak)
   for width in [.15,.3,.45]:
    for length in [1.8,3.]:
     for seeds in [2,3]:
      for fraction in [.2,.4]:
       r=dict(weak=weak,secondary=secondary,width=width,length=length,seeds=seeds,seedFraction=fraction);take=dev&choose_mask(stats,base,r);n=int(take.sum());f=int((take&(y==0)).sum());pts=int(y[take].sum());rec={**r,'selected':n,'false':f,'falseFraction':f/n,'coveredPoints':pts};records.append(rec)
       if f/n<=.06 and (best is None or pts>best['coveredPoints']):best=rec
 print('PREFIX',json.dumps(best),flush=True);assert best
 old=t.score_old.to_numpy();new=t.score_new.to_numpy();base=old>=strong;adjusted=np.where(base|((old>=best['weak'])&(new>=best['secondary'])),np.where(base,strong,best['weak']),-np.inf);stats=chain_stats(t,adjusted,strong,best['weak']);take=choose_mask(stats,base,best)
 den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];result={}
 for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',t.frame.to_numpy()==500,den.frame==500)]:
  s=take&mask;n=int(s.sum());f=int((s&(y==0)).sum());pts=int(y[s].sum());total=int(den.finePointCountInRoi[dmask].sum());result[name]=dict(selected=n,false=f,falseFraction=f/n,coveredPoints=pts,referencePoints=total,pointCoverage=pts/total)
 print('VALIDATION',json.dumps(result),flush=True);(P/'supported_chain_selection.json').write_text(json.dumps(dict(prefixCap=.06,prefixChoice=best,validation=result,alternatives=records),indent=2)+'\n');t[['frame','pillar','finePointCount']].assign(accepted=take).to_csv(OUT/'supported_chain_predictions.csv',index=False)
if __name__=='__main__':main()
