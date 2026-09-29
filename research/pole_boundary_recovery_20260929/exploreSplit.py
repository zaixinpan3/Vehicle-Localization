"""Rejected split-owner rescues lacking the clutter-subset requirement."""
import pandas as pd,numpy as np,itertools
x=pd.read_csv('output/pole_geometry_20260927/mississippi_features.csv');b=pd.read_csv('research/pole_selective_recovery_20260928/specialist_expected.csv');p=np.loadtxt('output/pole_selective_recovery_20260928/inference_expected.csv',delimiter=',');o=np.load('output/pole_selective_recovery_20260928/oof_d5_l10_w1.npy');u=pd.MultiIndex.from_frame(x[['frame','pillar']]);_,ix,g=np.unique(u,return_index=True,return_inverse=True);y=x.finePointCount.to_numpy()[ix];frames=x.frame.to_numpy()[ix];base=np.zeros(len(ix),bool);np.logical_or.at(base,g,p>=.8747229048074372)
baseo=np.zeros(len(ix),bool);np.logical_or.at(baseo,g,o>=.8747229048074372)
fixed=(x.supportHeight>=1.5)&(x.longestHeight>=.9*x.supportHeight)&(x.isolation>=.95)&(x.minimumQuarterCount>=3)&(x.ownerCount>=5)&(x.ownerHeight>=.8)&(x.ownerSupportFraction>=.3)&(x.tilt<=5)&(x.ownerFraction>=.2)

h=pd.MultiIndex.from_frame(x[['frame','hypothesis']]);_,hi,hg=np.unique(h,return_index=True,return_inverse=True);score=np.zeros(len(hi));np.maximum.at(score,hg,p);oscore=np.zeros(len(hi));np.maximum.at(oscore,hg,o)
res=[]
for rms,step,frac,gap,asp,share,fracown in itertools.product([.08,.1],[.025,.05],[.9],[.2,.3],[1.5,2,3],[.2,.3],[.3,.5]):
 own=(x.ownerCount>=5)&(x.ownerHeight>=1)&(x.ownerFraction>=share)&(x.ownerSupportFraction>=fracown)&(x.axisOwnerDistance<=.1)
 count=np.bincount(hg,weights=own,minlength=len(hi));physical=(x.supportHeight>=1.5)&(x.longestHeight>=.9*x.supportHeight)&(x.isolation>=.95)&(x.minimumQuarterCount>=3)&(x.tilt<=5)&(x.radialRms<=rms)&(x.quarterCenterStep<=step)&(x.fraction015>=frac)&(x.maximumGap<=gap)&(x.aspect<=asp)
 rule=physical&own&(count[hg]>=2)
 added=np.zeros(len(ix),bool);np.logical_or.at(added,g,rule&(score[hg]>=.2));add=added&~base;new=added|base
 oo=np.zeros(len(ix),bool);np.logical_or.at(oo,g,rule&(oscore[hg]>=.2));oa=oo&~baseo&(frames<=780);ok=(oo|baseo)&(frames<=780)
 res.append(dict(rms=rms,step=step,gap=gap,asp=asp,share=share,fracown=fracown,oofAdd=int(oa.sum()),oofBad=int((y[oa]==0).sum()),oofPoints=int(y[oa].sum()),add=int(add.sum()),bad=int((y[add]==0).sum()),suffixFP=float((y[new&(frames>780)]==0).mean()),target=int(new[(frames==856)&np.isin(x.pillar.to_numpy()[ix],[7944,7945])].sum())))
pd.DataFrame(res).to_csv('research/pole_boundary_recovery_20260929/split_sweep.csv',index=False)
