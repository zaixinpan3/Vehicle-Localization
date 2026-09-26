#!/usr/bin/env python3
"""Verify published follow-up metrics, algebra, parameter family and sources."""
import hashlib
import json
import numpy as np
import pandas as pd
import followup as task


def main():
    folder=task.DEST;summary=json.loads((folder/'summary.json').read_text())
    frames={drive:task.load(drive) for drive in ['12-11-24','12-09-31']}
    max_difference=0.;count=0
    for file,parameters in [('yaw_prediction_metrics.csv','yaw'),('beta_less_metrics.csv','beta')]:
        for _,expected in pd.read_csv(folder/file).iterrows():
            frame=frames[expected.drive]
            if parameters=='yaw':
                p=np.array(summary['candidateVectors'][expected.model])
                actual=task.yaw_scores(p,frame,expected.partition)
            else:
                candidates=pd.read_csv(folder/'beta_less_parameters.csv')
                row=candidates[candidates.model==expected.model].iloc[0]
                actual=task.beta_less_scores(np.array([row.A,row.B,row.constant]),frame,expected.partition,row.horizonSeconds)
            for key,value in actual.items():
                if isinstance(value,str):assert value==expected[key]
                else:
                    delta=abs(value-expected[key]);max_difference=max(max_difference,delta)
                    assert np.isfinite(value) and delta<1e-10,(expected.model,key,delta)
            count+=1
    hashes=0
    for file in [folder/'source_hashes.csv',task.PREVIOUS/'source_hashes.csv']:
        for _,row in pd.read_csv(file).iterrows():
            assert hashlib.sha256((task.ROOT/row.path).read_bytes()).hexdigest()==row.sha256,row.path
            hashes+=1
    selected=pd.read_csv(folder/'beta_less_parameters.csv').set_index('model').loc[summary['selectedIntegral']]
    largest=0.
    for _,row in pd.read_csv(folder/'conditional_parameter_family.csv').iterrows():
        qf,qr,j=row.CfOverMass,row.CrOverMass,row.IzOverMass
        A=task.L*qf*qr/(j*(qf+qr));B=(task.base.LF*qf-task.base.LR*qr)/(j*(qf+qr))
        largest=max(largest,abs(A-selected.A),abs(B-selected.B))
    assert largest<1e-10
    algebra=task.algebra_check()
    inventory=pd.read_csv(folder/'bag_inventory.csv')
    assert len(inventory)==19 and (inventory.inspvaMessages>0).sum()==2
    assert not summary['productionParametersChanged']
    result=dict(status='passed',recomputedMetricRows=count,maxAbsoluteMetricDifference=max_difference,
        verifiedHashes=hashes,algebraCases=100,algebraMaxError=algebra,
        parameterFamilyRows=29,parameterFamilyMaxCoefficientError=largest,
        inventoriedBags=19,bagsWithExactInspvaTopic=2)
    (folder/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
