"""Separate ordinary acceptance from stronger anchors for chain recovery."""
from probeCurbChains import ROOT,P,OLD,OUT,chain_stats,choose_mask
import numpy as np,pandas as pd,json

def main():
 t=pd.read_csv(OLD/'curb_predictions.csv');t=t[t.dataset=='mississippi'].reset_index(drop=True);threshold=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];oof=t.oof.to_numpy();dev=t.frame.to_numpy()<=780;y=t.finePointCount.to_numpy();best=None;records=[]
 for weak in [.45,.55,.6,.65,.7,.75]:
  for anchor in [.9,.925,.95,.975]:
   stats=chain_stats(t,oof,anchor,weak)
   for width in [.15,.3,.45]:
    for length in [1.8,3.,4.2]:
     for seeds in [2,3]:
      for fraction in [.2,.4,.6]:
       r=dict(weak=weak,anchor=anchor,width=width,length=length,seeds=seeds,seedFraction=fraction);take=dev&choose_mask(stats,oof>=threshold,r);n=int(take.sum());f=int((take&(y==0)).sum());pts=int(y[take].sum());rec={**r,'selected':n,'false':f,'falseFraction':f/n,'coveredPoints':pts};records.append(rec)
       if f/n<=.06 and (best is None or pts>best['coveredPoints']):best=rec
 print('PREFIX',json.dumps(best),flush=True);assert best
 score=t.score.to_numpy();stats=chain_stats(t,score,best['anchor'],best['weak']);take=choose_mask(stats,score>=threshold,best)
 den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];result={}
 for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',t.frame.to_numpy()==500,den.frame==500)]:
  s=take&mask;n=int(s.sum());f=int((s&(y==0)).sum());pts=int(y[s].sum());total=int(den.finePointCountInRoi[dmask].sum());result[name]=dict(selected=n,false=f,falseFraction=f/n,coveredPoints=pts,referencePoints=total,pointCoverage=pts/total)
 print('VALIDATION',json.dumps(result),flush=True);(P/'strong_anchor_selection.json').write_text(json.dumps(dict(prefixCap=.06,prefixChoice=best,validation=result,alternatives=records),indent=2)+'\n');t[['frame','pillar','finePointCount']].assign(accepted=take).to_csv(OUT/'strong_anchor_predictions.csv',index=False)
if __name__=='__main__':main()
