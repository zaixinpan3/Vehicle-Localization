#!/usr/bin/env python3
"""Explore sideslip-free identification without changing production parameters.

Run with uv run --offline --with numpy --with scipy --with pandas
--with matplotlib --with numba python <this file>.
"""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
DEST=Path(__file__).resolve().parent
PREVIOUS=ROOT/'research/mncav_identification_20260925'
sys.path.insert(0,str(PREVIOUS))
import identifyMncav as base

L=base.LF+base.LR
GYRO_BIAS=float(pd.read_csv(PREVIOUS/'drive_diagnostics.csv').fittedDbwGyroBiasRadps.iloc[0])
OUTPUT_X=-json.loads((ROOT/'config/mncavMotionOutputPoint.json').read_text())['forwardOffsetM']


def load(drive):
    f=base.load_drive(drive)
    folder='calibration_sensors' if drive=='12-11-24' else 'sensors'
    ins=pd.read_csv(ROOT/'output/mncav_wheel_only_20260916'/folder/'inspva.csv')
    dt=f.attrs['dt'];window=int(round(.51/dt))|1
    f['speed']=savgol_filter(np.hypot(ins.north_velocity,ins.east_velocity),window,3)
    f['measured_r']=savgol_filter(f.raw_r-GYRO_BIAS,window,3)
    f['measured_ay']=savgol_filter(-f.raw_ay,window,3)
    f['valid']=(f.ins_quality & (f.time>=.6) & (f.time<=f.time.iloc[-1]-.6) &
                (f.speed>=5) & (np.abs(f.measured_ay)<=4))
    f.attrs['steeringSpline']=CubicSpline(f.time,f.steer,extrapolate=False)
    return f


def integral_rows(f,mask,horizon):
    ids=np.flatnonzero(mask); groups=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    rows=[];step=int(round(horizon/f.attrs['dt']))
    for group in groups:
        for k in range(0,len(group)-step,step):
            s=f.iloc[group[k:k+step+1]]
            t=s.time.to_numpy()
            # Shared endpoints between adjacent integral windows are correlated.
            z=s.steer-L*s.measured_r/s.speed
            x=np.array([np.trapezoid(z,t),np.trapezoid(s.measured_ay,t),t[-1]-t[0]])
            rows.append((x,s.measured_r.iloc[-1]-s.measured_r.iloc[0]))
    return np.array([r[0] for r in rows]),np.array([r[1] for r in rows])


def beta_less_fit(f,mask,horizon):
    x,y=integral_rows(f,mask,horizon)
    result=least_squares(lambda p:(x@p-y)/.003,[25.,0.,0.],
        bounds=([.01,-5,-2],[200,5,2]),x_scale=[25,1,.1],loss='soft_l1',max_nfev=300)
    singular=np.linalg.svd(x/np.linalg.norm(x,axis=0),compute_uv=False)
    return result.x,dict(integralRows=len(y),designCondition=float(singular[0]/singular[-1]),
                         success=bool(result.success))


def beta_less_scores(p,f,part,horizon):
    mask=base.partition(f,part);x,y=integral_rows(f,mask,horizon)
    errors=[];forced_errors=[]
    for s in base.prediction_segments(f,mask):
        derivative=p[0]*(s.steer-L*s.measured_r/s.speed)+p[1]*s.measured_ay+p[2]
        prediction=s.measured_r.iloc[0]+cumulative_trapezoid(derivative,s.time,initial=0)
        errors.extend((prediction-s.measured_r)[50:])
        # A distinct predictor feeds back predicted yaw and takes measured
        # acceleration as an input. It does not use subsequent measured yaw.
        forced=np.zeros(len(s));forced[0]=s.measured_r.iloc[0]
        t=s.time.to_numpy();v=s.speed.to_numpy();d=s.steer.to_numpy();ay=s.measured_ay.to_numpy()
        for k in range(len(s)-1):
            rate=p[0]*L/((v[k]+v[k+1])/2)
            drive=p[0]*(d[k]+d[k+1])/2+p[1]*(ay[k]+ay[k+1])/2+p[2]
            decay=np.exp(-rate*(t[k+1]-t[k]))
            forced[k+1]=decay*forced[k]+(1-decay)*drive/rate
        forced_errors.extend(forced[50:]-s.measured_r.to_numpy()[50:])
    return dict(drive=f.attrs['drive'],partition=part,integralRows=len(y),
        deltaYawRateRmseRadps=float(np.sqrt(np.mean((x@p-y)**2))),
        conditionalYawRmseRadps=float(np.sqrt(np.mean(np.array(errors)**2))),
        accelerationDrivenYawRmseRadps=float(np.sqrt(np.mean(np.array(forced_errors)**2))))


def yaw_predictions(p,f,mask):
    rows=[]
    for s in base.prediction_segments(f,mask):
        t=s.time.to_numpy();v=s.speed.to_numpy()
        # Smooth interpolation avoids artificial minima at linear-interpolation
        # knots while optimizing the steering time shift.
        delta=f.attrs['steeringSpline'](t+p[6])-p[3]
        states=base.simulate_array(t,v,delta,np.array([0.,s.measured_r.iloc[0]]),np.exp(p[:3]))
        qf,qr,j=np.exp(p[:3])
        af=delta-(states[:,0]+base.LF*states[:,1])/v
        ar=-(states[:,0]-base.LR*states[:,1])/v
        predicted_ay=qf*af+qr*ar
        rows.append(dict(time=t[50:],r=states[50:,1],vy=states[50:,0]+OUTPUT_X*states[50:,1],
            ay=predicted_ay[50:],measured_r=s.measured_r.to_numpy()[50:],
            reference_vy=s.vy.to_numpy()[50:],measured_ay=s.measured_ay.to_numpy()[50:]))
    return rows


def yaw_fit(f,mask,start,active):
    def residual(z):
        p=start.copy();p[active]=z
        return np.concatenate([(r['r']-r['measured_r'])/.003 for r in yaw_predictions(p,f,mask)])
    result=least_squares(residual,start[active],bounds=(base.LOW[active],base.HIGH[active]),
        loss='soft_l1',x_scale=base.SCALES[active],max_nfev=300,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    p=start.copy();p[active]=result.x
    return p,result


def yaw_scores(p,f,part):
    rows=yaw_predictions(p,f,base.partition(f,part))
    re=np.concatenate([r['r']-r['measured_r'] for r in rows])
    ve=np.concatenate([r['vy']-r['reference_vy'] for r in rows])
    ae=np.concatenate([r['ay']-r['measured_ay'] for r in rows])
    return dict(drive=f.attrs['drive'],partition=part,samples=len(re),
        yawRmseRadps=float(np.sqrt(np.mean(re**2))),vyRmseMps=float(np.sqrt(np.mean(ve**2))),
        vyBiasMps=float(ve.mean()),uncalibratedAyRmseMps2=float(np.sqrt(np.mean(ae**2))))


def algebra_check():
    rng=np.random.default_rng(20260925);largest=0.
    for _ in range(100):
        qf,qr,j=rng.uniform(10,120),rng.uniform(10,120),rng.uniform(.5,5)
        v,vy,r,delta=rng.uniform(5,25),rng.normal(0,.2),rng.normal(0,.1),rng.normal(0,.03)
        ay=qf*(delta-(vy+base.LF*r)/v)+qr*(-(vy-base.LR*r)/v)
        rdot=base.flow(vy,r,v,delta,qf,qr,j)[1]
        A=L*qf*qr/(j*(qf+qr));B=(base.LF*qf-base.LR*qr)/(j*(qf+qr))
        largest=max(largest,abs(rdot-A*(delta-L*r/v)-B*ay))
        qf2=A*j/(base.LF-j*B);qr2=A*j/(base.LR+j*B)
        assert np.allclose([qf,qr],[qf2,qr2],atol=1e-10)
    assert largest<1e-12
    return largest


def main():
    error=algebra_check()
    cal=load('12-11-24');mask=base.partition(cal,'fit')
    beta={};beta_parameters=[];beta_scores=[]
    for horizon in [.1,.2,.5,1.]:
        p,info=beta_less_fit(cal,mask,horizon);name='integral_'+str(horizon);beta[name]=(p,horizon)
        beta_parameters.append(dict(model=name,A=p[0],B=p[1],constant=p[2],horizonSeconds=horizon,**info))
        for part in ['fit','holdout']:
            beta_scores.append(dict(model=name,**beta_less_scores(p,cal,part,horizon)))
    # Holdout selects one integral duration; no evaluation-drive optimization.
    selected_beta=min([r for r in beta_scores if r['partition']=='holdout'],key=lambda r:r['conditionalYawRmseRadps'])['model']
    models={'nominal':base.NOMINAL.copy()};parameters=[];scores=[];multistarts=[]
    rng=np.random.default_rng(20260925)
    for name,active in [('yaw_physical',[0,1,2]),('yaw_alignment',[0,1,2,3,6])]:
        trials=[]
        for k in range(8):
            start=base.NOMINAL.copy()
            if k:
                start[:3]=rng.uniform(base.LOW[:3]+.05,base.HIGH[:3]-.05)
                if 3 in active:start[3]+=rng.uniform(-.001,.001)
                if 6 in active:start[6]=rng.uniform(-.08,.08)
            trial,optimization=yaw_fit(cal,mask,start,active);trials.append((trial,optimization))
            multistarts.append(dict(model=name,start=k,cost=optimization.cost,success=bool(optimization.success),
                CfOverMass=np.exp(trial[0]),CrOverMass=np.exp(trial[1]),IzOverMass=np.exp(trial[2]),
                roadSteeringZeroRad=trial[3],timeShiftSeconds=trial[6]))
        p,result=min(trials,key=lambda trial:trial[1].cost);models[name]=p
        parameters.append(dict(model=name,CfOverMass=np.exp(p[0]),CrOverMass=np.exp(p[1]),
            IzOverMass=np.exp(p[2]),steeringWheelZeroDeg=np.rad2deg(p[3]*base.RATIO),
            timeShiftSeconds=p[6],success=bool(result.success),cost=result.cost,
            boundsReached=str([k for k in active if min(p[k]-base.LOW[k],base.HIGH[k]-p[k])<1e-4*(base.HIGH[k]-base.LOW[k])])) )
    for name,p in models.items():
        for part in ['fit','holdout']:scores.append(dict(model=name,**yaw_scores(p,cal,part)))
    selected_yaw=min([r for r in scores if r['partition']=='holdout'],key=lambda r:r['yawRmseRadps'])['model']
    print('Selected before evaluation:',selected_beta,selected_yaw,flush=True)
    # Profile the fixed inertia ratio while refitting stiffness and alignment.
    profiles=[]
    for j in [.5,.8,1.2,1.8,np.exp(base.NOMINAL[2]),3.5,4.8,6.]:
        trials=[]
        for starting in [models['yaw_alignment'],models['yaw_physical'],base.NOMINAL]:
            start=starting.copy();start[2]=np.log(j)
            trials.append(yaw_fit(cal,mask,start,[0,1,3,6]))
        p,result=min(trials,key=lambda trial:trial[1].cost)
        profiles.append(dict(IzOverMass=j,conditionalIzKgM2=j*base.MASS,CfOverMass=np.exp(p[0]),
            CrOverMass=np.exp(p[1]),cost=result.cost,success=bool(result.success),
            fitYawRmseRadps=yaw_scores(p,cal,'fit')['yawRmseRadps'],
            holdoutYawRmseRadps=yaw_scores(p,cal,'holdout')['yawRmseRadps']))
    evaluation=load('12-09-31')
    for name,(p,horizon) in beta.items():beta_scores.append(dict(model=name,**beta_less_scores(p,evaluation,'evaluation',horizon)))
    for name,p in models.items():scores.append(dict(model=name,**yaw_scores(p,evaluation,'evaluation')))
    pd.DataFrame(beta_parameters).to_csv(DEST/'beta_less_parameters.csv',index=False)
    pd.DataFrame(beta_scores).to_csv(DEST/'beta_less_metrics.csv',index=False)
    pd.DataFrame(parameters).to_csv(DEST/'yaw_parameters.csv',index=False)
    pd.DataFrame(scores).to_csv(DEST/'yaw_prediction_metrics.csv',index=False)
    pd.DataFrame(profiles).to_csv(DEST/'inertia_profile.csv',index=False)
    pd.DataFrame(multistarts).to_csv(DEST/'multistart.csv',index=False)
    # A reduced relation has a continuum of physical solutions even for x_s=0.
    p,horizon=beta[selected_beta];A,B,_=p;family=[]
    for j in np.linspace(.4,6,29):
        if base.LF-j*B>0 and base.LR+j*B>0:
            qf=A*j/(base.LF-j*B);qr=A*j/(base.LR+j*B)
            family.append(dict(IzOverMass=j,CfOverMass=qf,CrOverMass=qr,
                conditionalIzKgM2=j*base.MASS,conditionalCfNPerRad=qf*base.MASS,
                conditionalCrNPerRad=qr*base.MASS))
    pd.DataFrame(family).to_csv(DEST/'conditional_parameter_family.csv',index=False)
    summary=dict(selectedIntegral=selected_beta,selectedYaw=selected_yaw,productionParametersChanged=False,
        gyroBiasRadps=GYRO_BIAS,fixedOutputPointX=OUTPUT_X,algebraCases=100,algebraMaxError=error,
        candidateVectors={k:v.tolist() for k,v in models.items()},randomSeed=20260925,
        sourceCommit='c49895662fcf4211da707cec763d92f6bb2039cd',
        betaLessInterpretation='Conditional reconstruction driven by measured steering, yaw and acceleration; not free-run steering-only prediction',
        yawInterpretation='Steering-only dynamic prediction with measured speed, zero initial vy, measured initial yaw and 1 s burn-in per <=8 s window')
    (DEST/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    paths=[PREVIOUS/'identifyMncav.py',PREVIOUS/'drive_diagnostics.csv',PREVIOUS/'source_hashes.csv',
           PREVIOUS/'raw_bag_audit.json',Path(__file__),ROOT/'config/mncavMotionOutputPoint.json']
    pd.DataFrame([dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                  for p in paths]).to_csv(DEST/'source_hashes.csv',index=False)
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    profiles=pd.DataFrame(profiles)
    axes[0].plot(profiles.conditionalIzKgM2,profiles.fitYawRmseRadps,'o-',label='Fit')
    axes[0].plot(profiles.conditionalIzKgM2,profiles.holdoutYawRmseRadps,'o-',label='Holdout')
    axes[0].set_xlabel('Fixed conditional yaw inertia (kg m²)');axes[0].set_ylabel('Yaw RMSE (rad/s)');axes[0].legend()
    for name in models:
        row=next(r for r in scores if r['model']==name and r['partition']=='evaluation')
        axes[1].scatter(row['yawRmseRadps'],row['vyRmseMps'],label=name,s=60)
    axes[1].set_xlabel('Evaluation yaw RMSE (rad/s)');axes[1].set_ylabel('Evaluation lateral velocity RMSE (m/s)');axes[1].legend()
    for ax in axes:ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(DEST/'identification_diagnostics.png',dpi=160);plt.close(fig)
    assert all(row['success'] for row in parameters+profiles.to_dict('records')+beta_parameters)
    print(pd.DataFrame(parameters).to_string(index=False),flush=True)
    print(pd.DataFrame(scores).to_string(index=False),flush=True)
    print(pd.DataFrame(beta_scores).to_string(index=False),flush=True)


if __name__=='__main__':main()
