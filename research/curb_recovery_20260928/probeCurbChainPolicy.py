"""Trade isolated-cell confidence against supported chain recovery."""
from probeCurbChains import ROOT,P,OLD,OUT,chain_stats,choose_mask
import numpy as np,pandas as pd,json

def main():
 t=pd.read_csv(OLD/'curb_predictions.csv');t=t[t.dataset=='mississippi'].reset_index(drop=True);strong=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];oof=t.oof.to_numpy();dev=t.frame.to_numpy()<=780;y=t.finePointCount.to_numpy();best=None;records=[]
 for weak in [.45,.55,.6,.65,.7,.75]:
  stats=chain_stats(t,oof,strong,weak)
  for isolated in [.875,.9,.925,.95]:
   for width in [.15,.3,.45]:
    for length in [1.8,3.]:
     for seeds in [2,3]:
      for fraction in [.2,.4]:
       r=dict(weak=weak,isolated=isolated,width=width,length=length,seeds=seeds,seedFraction=fraction);take=dev&choose_mask(stats,oof>=isolated,r);n=int(take.sum());false=int((take&(y==0)).sum());pts=int(y[take].sum());rec={**r,'selected':n,'false':false,'falseFraction':false/n,'coveredPoints':pts};records.append(rec)
       if false/n<=.055 and (best is None or pts>best['coveredPoints']):best=rec
 print('PREFIX',json.dumps(best),flush=True);assert best
 score=t.score.to_numpy();stats=chain_stats(t,score,strong,best['weak']);take=choose_mask(stats,score>=best['isolated'],best)
 den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];result={}
 for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',t.frame.to_numpy()==500,den.frame==500)]:
  selected=take&mask;n=int(selected.sum());f=int((selected&(y==0)).sum());pts=int(y[selected].sum());total=int(den.finePointCountInRoi[dmask].sum());result[name]=dict(selected=n,false=f,falseFraction=f/n,coveredPoints=pts,referencePoints=total,pointCoverage=pts/total)
 print('VALIDATION',json.dumps(result),flush=True);(P/'chain_policy.json').write_text(json.dumps(dict(prefixCap=.055,prefixChoice=best,validation=result,alternatives=records),indent=2)+'\n');t[['frame','pillar','finePointCount']].assign(accepted=take).to_csv(OUT/'chain_policy_predictions.csv',index=False)
if __name__=='__main__':main()
