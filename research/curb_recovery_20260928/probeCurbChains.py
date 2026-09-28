"""Grow confidence-connected slender curb chains; no point or fine-grid lookup."""
from probeCurbContinuation import ROOT,P,OLD,OUT
import numpy as np,pandas as pd,json
from scipy.ndimage import label

def chain_stats(t,scores,strongThreshold,weak):
 n=len(t);frame=t.frame.to_numpy().astype(int)-1;p=t.pillar.to_numpy().astype(int)-1;row=p%100;col=p//100;index=frame*10000+row*100+col
 raster=np.zeros((1170,100,100),bool);raster.ravel()[index[scores>=weak]]=True;structure=np.zeros((3,3,3),int);structure[1,:,:]=1;groups,count=label(raster,structure);gid=groups.ravel()[index];use=gid>0;g=gid[use];c=np.bincount(g,minlength=count+1);cx=np.bincount(g,weights=col[use]*.6,minlength=count+1)/np.maximum(c,1);cy=np.bincount(g,weights=row[use]*.6,minlength=count+1)/np.maximum(c,1);xx=np.bincount(g,weights=(col[use]*.6)**2,minlength=count+1)/np.maximum(c,1)-cx**2;yy=np.bincount(g,weights=(row[use]*.6)**2,minlength=count+1)/np.maximum(c,1)-cy**2;xy=np.bincount(g,weights=(row[use]*.6)*(col[use]*.6),minlength=count+1)/np.maximum(c,1)-cx*cy;spread=np.hypot(xx-yy,2*xy);lo=np.maximum(0,(xx+yy-spread)/2);hi=np.maximum(0,(xx+yy+spread)/2);seeds=np.bincount(gid[scores>=strongThreshold],minlength=count+1);width=np.sqrt(lo);length=2*np.sqrt(3*hi)+.6;anisotropy=spread/np.maximum(xx+yy,1e-12)
 return dict(gid=gid,count=c,seeds=seeds,width=width,length=length,anisotropy=anisotropy)

def choose_mask(stats,base,r):
 qualifies=(stats['seeds']>=r['seeds'])&(stats['seeds']/np.maximum(stats['count'],1)>=r['seedFraction'])&(stats['width']<=r['width'])&(stats['length']>=r['length'])&(stats['anisotropy']>=.9);qualifies[0]=False;return base|qualifies[stats['gid']]

def main():
 t=pd.read_csv(OLD/'curb_predictions.csv');t=t[t.dataset=='mississippi'].reset_index(drop=True);strong=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];dev=t.frame.to_numpy()<=780;y=t.finePointCount.to_numpy();oof=t.oof.to_numpy();base=oof>=strong;records=[];best=None
 for weak in [.45,.55,.6,.65,.7,.75]:
  stats=chain_stats(t,oof,strong,weak)
  for width in [.15,.3,.45]:
   for length in [1.8,3.,4.2]:
    for seeds in [2,3]:
     for fraction in [.2,.4,.6]:
      r=dict(weak=weak,width=width,length=length,seeds=seeds,seedFraction=fraction);take=dev&choose_mask(stats,base,r);n=int(take.sum());fp=int((take&(y==0)).sum());pts=int(y[take].sum());rec={**r,'selected':n,'false':fp,'falseFraction':fp/n,'coveredPoints':pts};records.append(rec)
      if fp/n<=.06 and (best is None or pts>best['coveredPoints']):best=rec
 print('PREFIX',json.dumps(best),flush=True);assert best
 stats=chain_stats(t,t.score.to_numpy(),strong,best['weak']);take=choose_mask(stats,t.score.to_numpy()>=strong,best)
 den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];result={}
 for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',t.frame.to_numpy()==500,den.frame==500)]:
  s=take&mask;n=int(s.sum());false=int((s&(y==0)).sum());pts=int(y[s].sum());total=int(den.finePointCountInRoi[dmask].sum());result[name]=dict(selected=n,false=false,falseFraction=false/n,coveredPoints=pts,referencePoints=total,pointCoverage=pts/total)
 print('VALIDATION',json.dumps(result),flush=True);(P/'chain_selection.json').write_text(json.dumps(dict(prefixCap=.06,prefixChoice=best,validation=result,alternatives=records),indent=2)+'\n');t[['frame','pillar','finePointCount']].assign(accepted=take).to_csv(OUT/'chain_predictions.csv',index=False)
if __name__=='__main__':main()
