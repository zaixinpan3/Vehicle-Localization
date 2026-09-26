#!/usr/bin/env python3
"""Export a conditional physical candidate, comparison tables and diagnostics."""
import json,sys
from importlib.metadata import version
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from prepare import ROOT,DEST,OUT

def main():
    models=json.loads((DEST/'lateral_models.json').read_text());long=json.loads((DEST/'longitudinal_models.json').read_text())
    metrics=pd.read_csv(DEST/'lateral_metrics.csv');mass=long['candidates']['net_torque']['massKg']
    joint=models['candidates']['bicycle_joint_vy'];qf,qr,j=np.exp(joint[:3])
    boot=pd.read_csv(DEST/'mass_bootstrap.csv');interval=boot[boot.model=='net_torque'].massKg.quantile([.025,.975]).tolist()
    candidate=dict(status='Research candidate for simulation; not a verified production observer configuration',
      physical=dict(massKg=mass,wheelbaseM=3.089,lfM=1.374605,lrM=1.714395,
        frontAxleCorneringStiffnessNPerRad=mass*qf,rearAxleCorneringStiffnessNPerRad=mass*qr,yawInertiaKgM2=mass*j,
        steeringRatio=joint[4],steeringWheelZeroRad=joint[3],steeringWheelZeroDeg=float(np.rad2deg(joint[3]))),
      normalizedDynamics=dict(CfOverMass=qf,CrOverMass=qr,IzOverMass=j),
      alternativeAtPriorMass2273Kg=dict(Cf=qf*2273,Cr=qr*2273,Iz=j*2273),
      massBootstrapPercentile95Kg=interval,
      assumptions=['Manufacturer CAN torque scaling is correct for the recorded firmware.',
       'Rear wheel speed uses frozen effective-radius calibration from June 11-24.',
       'Longitudinal fit assumes net axle minus brake torque, one constant force offset and quadratic drag.',
       'CG split and wheelbase remain priors, not newly identified.',
       'Lateral velocity is expressed at the frozen effective output point x=-2.359798894 m.',
       'Linear planar bicycle; no estimated bank, tire saturation, load transfer or mass variation.',
       'Bootstrap interval conditions on this model and excludes systematic torque/radius/grade uncertainty.'],
      empiricalYawModel=models['candidates'][models['selected']],
      empiricalFeatureOrder='For u=v*steeringWheelRad/16.2: lowpass(u,tau), lowpass(u*v^2/100,tau), lowpass(v/10,tau); tau varies fastest. Standardize, prepend 1, dot coefficients.',
      integration='50 Hz inputs; four implicit-midpoint substeps per sample for physical candidates',
      empiricalStateInitialization='Each <=8 s evaluation window initializes each lowpass state to its first input value; 1 s burn-in.',
      preprocessing='Centered 25-sample degree-3 Savitzky-Golay smoothing at 50 Hz. Results are offline, not zero-latency online validation.',
      productionParametersChanged=False)
    (DEST/'identifiedMncavModel.json').write_text(json.dumps(candidate,indent=2)+'\n')
    aggregate=metrics.groupby(['model','split'])[['yawRmseRadps','vyRmseMps','vyBiasMps']].mean().reset_index()
    aggregate.to_csv(DEST/'comparison.csv',index=False)
    names=['nominal','previous_yaw','bicycle_joint_vy',models['selected']]
    fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for i,split in enumerate(['validation','june_evaluation']):
        values=[aggregate[(aggregate.model==n)&(aggregate.split==split)].yawRmseRadps.iloc[0] for n in names]
        axs[0,0].bar(np.arange(4)+i*.35,values,.35,label=split)
    axs[0,0].set(xticks=np.arange(4)+.175,xticklabels=['Nominal','Earlier yaw fit','Joint physical','Filter bank'],ylabel='Yaw-rate RMSE [rad/s]',title='Independent-recording prediction');axs[0,0].legend()
    for i,split in enumerate(['june_holdout','june_evaluation']):
        ns=['nominal','bicycle_ratio_lag','bicycle_joint_vy'];values=[aggregate[(aggregate.model==n)&(aggregate.split==split)].vyRmseMps.iloc[0] for n in ns]
        axs[0,1].bar(np.arange(3)+i*.35,values,.35,label=split)
    axs[0,1].set(xticks=np.arange(3)+.175,xticklabels=['Nominal','Yaw-only physical','Joint physical'],ylabel='Lateral velocity RMSE [m/s]',title='Yaw quality does not ensure lateral accuracy');axs[0,1].legend()
    order=['net_torque','free_brake_gain','traction_only']
    axs[1,0].boxplot([boot[boot.model==n].massKg for n in order],tick_labels=['Net torque','Brake gain free','Traction only'],showfliers=False)
    axs[1,0].axhline(2273,ls='--',c='gray',label='Existing mass prior');axs[1,0].set(ylabel='Conditional mass [kg]',title='150 whole-drive bootstrap fits per method');axs[1,0].legend()
    f=pd.read_csv(OUT/'lateral_predictions.csv');f=f[(f.model==models['selected'])&(f.split=='june_evaluation')]
    # Insert gaps between windows so no lines bridge unscored intervals.
    t=f.time.to_numpy();reference=f.referenceYaw.to_numpy().copy();prediction=f.predictedYaw.to_numpy().copy();gaps=np.r_[False,np.diff(t)>.03]
    reference[gaps]=np.nan;prediction[gaps]=np.nan
    axs[1,1].plot(t,reference,label='Bias-corrected DBW yaw',lw=1);axs[1,1].plot(t,prediction,label='Selected filter bank',lw=1)
    axs[1,1].set(xlabel='Ouster native time [s]',ylabel='Yaw rate [rad/s]',title='June 09-31 transfer evaluation');axs[1,1].legend()
    fig.savefig(DEST/'diagnostics.png',dpi=170);plt.close(fig)
    (DEST/'environment.json').write_text(json.dumps(dict(python=sys.version,packages={n:version(n) for n in ['numpy','scipy','pandas','matplotlib','numba','rosbags']}),indent=2)+'\n')
    print(json.dumps(candidate['physical'],indent=2))
if __name__=='__main__':main()
