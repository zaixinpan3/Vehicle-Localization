#!/usr/bin/env python3
"""Conditional mass identification with documented torque and wheel acceleration."""
import json
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from prepare import ROOT,DEST,OUT
RADIUS=np.mean(json.loads((ROOT/'config/mncavWheelSpeedCalibration.json').read_text())['effectiveRadiusM'])

def load():
    rows=[]
    for s in pd.read_csv(DEST/'drive_summary.csv').itertuples():
        f=pd.read_csv(OUT/s.drive/'uniform.csv');f['drive']=s.drive;f['split']=s.split
        mask=(f.time>.6)&(f.time<f.time.iloc[-1]-.6)&(f.speed>3)&(abs(f.r)<.15)&np.isfinite(f.acceleration)
        rows.append(f.loc[mask].iloc[::5])
    return pd.concat(rows,ignore_index=True)

def fit(f,mode):
    if mode=='traction_only':f=f[(f.brake<30)&(f.axle>100)]
    force=f.axle.to_numpy()/RADIUS; brake=f.brake.to_numpy()/RADIUS
    accel=f.acceleration.to_numpy();v=f.speed.to_numpy()
    # Force units; offset is common rolling resistance/grade/torque bias.
    if mode=='free_brake_gain':
        fun=lambda p:(force-p[3]*brake-p[0]*accel-p[1]*v*v-p[2])/300
        x=[2273.,.4,100.,1.];lo=[500,0,-2000,0];hi=[6000,10,2000,2]
    else:
        fun=lambda p:(force-brake-p[0]*accel-p[1]*v*v-p[2])/300
        x=[2273.,.4,100.];lo=[500,0,-2000];hi=[6000,10,2000]
    result=least_squares(fun,x,bounds=(lo,hi),x_scale=np.array(hi)-lo,loss='soft_l1',max_nfev=150)
    p=list(result.x)
    if len(p)==3:p.append(1.)
    return np.array(p),len(f)

def main():
    f=load();f.to_csv(OUT/'longitudinal_samples.csv',index=False)
    train=f[f.split=='train'];rows=[];params={};boot=[];rng=np.random.default_rng(20260925)
    for mode in ['net_torque','free_brake_gain','traction_only']:
        p,n=fit(train,mode);params[mode]=dict(massKg=p[0],dragNPerMps2=p[1],offsetN=p[2],brakeGain=p[3],trainingSamples=n)
        for split,s in f.groupby('split'):
            for subset,mask in [('all',np.ones(len(s),bool)),('traction',(s.brake<30)&(s.axle>100))]:
                z=s.loc[mask];pred=(z.axle/RADIUS-p[3]*z.brake/RADIUS-p[1]*z.speed**2-p[2])/p[0]
                e=pred-z.acceleration
                rows.append(dict(model=mode,split=split,subset=subset,samples=len(z),accelerationRmseMps2=np.sqrt(np.mean(e**2)),biasMps2=np.mean(e)))
        drives=train.drive.unique()
        for k in range(150):
            sample=pd.concat([train[train.drive==d] for d in rng.choice(drives,len(drives),replace=True)])
            q,_=fit(sample,mode);boot.append(dict(model=mode,replicate=k,massKg=q[0],brakeGain=q[3]))
        print(mode,params[mode],flush=True)
    pd.DataFrame(rows).to_csv(DEST/'longitudinal_metrics.csv',index=False)
    pd.DataFrame(boot).to_csv(DEST/'mass_bootstrap.csv',index=False)
    (DEST/'longitudinal_models.json').write_text(json.dumps(dict(radiusM=RADIUS,candidates=params,seed=20260925,bootstrapUnit='whole May training recording'),indent=2)+'\n')
if __name__=='__main__':main()
