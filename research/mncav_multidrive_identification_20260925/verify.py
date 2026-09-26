#!/usr/bin/env python3
"""Independent metric, protocol, input-hash and numerical integration checks."""
import hashlib,io,json,subprocess
import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp
from extract import decode_torque
from lateral import midpoint,physical,segments
from prepare import ROOT,DEST,OUT

def main():
    checked=0
    for report in json.loads((DEST/'extraction_manifest.json').read_text()):
        folder=OUT/ROOT.joinpath(report['bag']).stem
        for name,expected in report['exportSha256'].items():
            assert hashlib.sha256((folder/(name+'.csv')).read_bytes()).hexdigest()==expected;checked+=1
    # Compile the actual pinned manufacturer bitfields, not a restated decoder.
    cpp=OUT/'verify_decoder.cpp'
    cpp.write_text(r'''#include "dispatch.h"
#include <cstring>
#include <cstdio>
using namespace dbw_fca_can;
int main(){
 for(unsigned k=0;k<32768;k++){
  uint64_t raw=k;MsgReportThrottleInfo msg;std::memcpy(&msg,&raw,8);
  std::printf("%u %d\n",k,(int)msg.axle_torque);
 }
 for(unsigned k=0;k<4096;k++){
  uint64_t raw=k|((uint64_t)(4095-k)<<12);MsgReportBrakeInfo msg;std::memcpy(&msg,&raw,8);
  std::printf("%u %u %u\n",k,msg.brake_torque_request,msg.brake_torque_actual);
 }
}''')
    executable=OUT/'verify_decoder'
    subprocess.run(['g++','-O2',str(cpp),'-o',str(executable)],check=True)
    lines=subprocess.check_output([str(executable)],text=True).splitlines()
    for line in lines[:32768]:
        k,val=map(int,line.split());actual=decode_torque(0x75,k.to_bytes(8,'little'))[0]
        assert (np.isnan(actual) if val==-16384 else actual==val*1.5625)
    for line in lines[32768:]:
        k,req,act=map(int,line.split());actual=decode_torque(0x74,(k|((4095-k)<<12)).to_bytes(8,'little'))
        assert np.allclose(actual,[np.nan if req==4095 else req*3,np.nan if act==4095 else act*3],equal_nan=True)
    pred=pd.read_csv(OUT/'lateral_predictions.csv');metrics=pd.read_csv(DEST/'lateral_metrics.csv');error=0.
    for row in metrics.itertuples():
        g=pred[(pred.model==row.model)&(pred.split==row.split)&(pred.drive==row.drive)]
        assert len(g)==row.samples
        e=g.predictedYaw-g.referenceYaw
        error=max(error,abs(np.sqrt(np.mean(e**2))-row.yawRmseRadps))
        if np.isfinite(row.vyRmseMps):
            ev=g.predictedVy-g.referenceVy;error=max(error,abs(np.sqrt(np.mean(ev**2))-row.vyRmseMps))
    assert error<1e-12
    assert np.max(abs(pred.predictedYaw))<2
    assert np.nanmax(abs(pred.predictedVy))<10
    samples=pd.read_csv(OUT/'longitudinal_samples.csv')
    long_models=json.loads((DEST/'longitudinal_models.json').read_text())
    long_metrics=pd.read_csv(DEST/'longitudinal_metrics.csv');long_error=0.
    for row in long_metrics.itertuples():
        f=samples[samples.split==row.split]
        if row.subset=='traction':f=f[(f.brake<30)&(f.axle>100)]
        assert len(f)==row.samples
        p=long_models['candidates'][row.model];radius=long_models['radiusM']
        predicted=(f.axle/radius-p['brakeGain']*f.brake/radius-p['dragNPerMps2']*f.speed**2-p['offsetN'])/p['massKg']
        long_error=max(long_error,abs(np.sqrt(np.mean((predicted-f.acceleration)**2))-row.accelerationRmseMps2))
    assert long_error<1e-12
    # Direct high-accuracy ODE check on the joint fit and one real input segment.
    model=json.loads((DEST/'lateral_models.json').read_text());p=np.array(model['candidates']['bicycle_joint_vy'])
    s=next(s for s in segments() if s['split']=='june_train');qf,qr,j=np.exp(p[:3]);lf=1.374605;lr=1.714395
    def ode(t,x):
        v=np.interp(t,s['t'],s['v']);d=(np.interp(t,s['t'],s['sw'])-p[3])/p[4]
        af=d-(x[0]+lf*x[1])/v;ar=-(x[0]-lr*x[1])/v
        return [qf*af+qr*ar-v*x[1],(lf*qf*af-lr*qr*ar)/j]
    reference=solve_ivp(ode,[s['t'][0],s['t'][-1]],[0,s['r'][0]],t_eval=s['t'],rtol=1e-10,atol=1e-12,max_step=.002).y.T
    integration_error=np.max(abs(reference-physical(p,s)),axis=0)
    assert integration_error[0]<.0003 and integration_error[1]<.0001, integration_error
    # Selected filter bank is independent of future measured outputs.
    from lateral import features
    x=features(s);copy={k:(v.copy() if isinstance(v,np.ndarray) else v) for k,v in s.items()};copy['r'][1:]=999;copy['vy'][:]=999
    assert np.array_equal(x,features(copy))
    # Truncating future inputs must not change any already emitted feature.
    short={k:(v[:100] if isinstance(v,np.ndarray) else v) for k,v in s.items()}
    assert np.array_equal(features(short),x[:100])
    result=dict(exportHashesChecked=checked,manufacturerDecoderCases=len(lines),lateralMetricRows=len(metrics),
      maxMetricDifference=error,longitudinalMetricRows=len(long_metrics),maxLongitudinalMetricDifference=long_error,finalTrajectoriesBelowNumericalGuard=True,midpointVsSolveIvpMaxError=integration_error.tolist(),filterOutputLeakageCheck='passed',filterCausalRecurrenceCheck='passed',
      limitations='Offline input preprocessing uses centered Savitzky-Golay smoothing; causal recurrence is not an end-to-end online latency test.')
    (DEST/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
if __name__=='__main__':main()
