"""Select shared-model retention using prefix OOF specialist support only."""
from pathlib import Path
import numpy as np,pandas as pd,json
P=Path(__file__).resolve().parent;ROOT=P.parents[1];OUT=ROOT/'output/pole_selective_recovery_20260928'
r=pd.read_csv(ROOT/'output/pole_geometry_20260927/mississippi_features.csv');n=len(r)
old=np.loadtxt(ROOT/'output/pole_geometry_20260927/stable_moments_replay_expected.csv')[:n]
new=np.loadtxt(OUT/'inference_expected.csv');assert len(new)==n
baseOOF=np.load(ROOT/'output/pole_geometry_20260927/stable_moments_compact_d5_w1.0_oof.npy')[:n]
spec=json.loads((P/'specialist_selection.json').read_text())['chosen'];newOOF=np.load(OUT/f"oof_d{spec['depth']}_l{spec['leaf']}_w{spec['power']}.npy")
# Filename integer power is deliberately the Python repr from training.
keys=list(zip(r.frame,r.pillar));idx=pd.MultiIndex.from_tuples(keys);codes,unique=pd.factorize(idx);first=np.unique(codes,return_index=True)[1];labels=r.finePointCount.to_numpy()[first];frame=r.frame.to_numpy()[first]
def owners(mask):
 a=np.zeros(len(first),bool);np.logical_or.at(a,codes,mask);return a
def measure(mask,part):
 a=owners(mask)&part;return dict(selected=int(a.sum()),empty=int((labels[a]==0).sum()),covered=int(labels[a].sum()),falseFraction=float((labels[a]==0).sum()/max(a.sum(),1)))
rows=[]
for floor in np.arange(0,1.001,.025):
 sel=(newOOF>=spec['threshold'])|((baseOOF>=.8701683227450074)&(newOOF>=floor));m=measure(sel,frame<=780)
 rows.append(dict(floor=float(floor),**m))
# Same pooled OOF precision budget as deployed calibration.
valid=[x for x in rows if x['falseFraction']<=.075];best=max(valid,key=lambda x:(x['covered'],-x['empty'],x['floor']))
(P/'consensus_selection.json').write_text(json.dumps(dict(chosen=best,alternatives=rows,method='max prefix OOF coverage under7.5% false; specialist strong OR shared strong with specialist floor'),indent=2)+'\n')
sel=(new>=spec['threshold'])|((old>=.8701683227450074)&(new>=best['floor']));pred=r.iloc[first][['frame','pillar','finePointCount']].copy();pred['selected']=owners(sel);pred.to_csv(ROOT/'output/pole_selective_recovery_20260928/consensus_expected.csv',index=False)
print(best);print('all',measure(sel,np.ones(len(first),bool)));print('suffix',measure(sel,frame>780));print(pred[(pred.frame==857)&pred.selected])
