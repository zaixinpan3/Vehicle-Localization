#!/usr/bin/env python3
"""Recompute published prediction scores and verify input/artifact integrity."""
import hashlib
import json

import numpy as np
import pandas as pd

import identifyMncav as experiment


def main():
    dest=experiment.DEST
    summary=json.loads((dest/'summary.json').read_text())
    expected=pd.read_csv(dest/'prediction_metrics.csv')
    frames={drive:experiment.load_drive(drive) for drive in ['12-11-24','12-09-31']}
    max_difference=0.
    for _,row in expected.iterrows():
        p=np.array(summary['candidateVectors'][row.model])
        actual=experiment.score(row.model,p,frames[row.drive],row.partition)
        for key,value in actual.items():
            if isinstance(value,str):
                assert value==row[key]
            else:
                difference=abs(value-row[key])
                assert difference<1e-11,(row.model,key,difference)
                max_difference=max(max_difference,difference)
    inputs=pd.read_csv(dest/'source_hashes.csv')
    for _,row in inputs.iterrows():
        source=experiment.ROOT/row.path
        assert hashlib.sha256(source.read_bytes()).hexdigest()==row.sha256,row.path
    candidates=pd.read_csv(dest/'candidate_parameters.csv')
    assert candidates.success.all()
    assert np.isfinite(candidates[['CfOverMass','CrOverMass','IzOverMass']]).all().all()
    assert (candidates[['CfOverMass','CrOverMass','IzOverMass']]>0).all().all()
    assert not summary['productionParametersChanged']
    assert not summary['absoluteMassIdentifiable']
    for frame in frames.values():
        assert frame.loc[frame.valid,'ins_quality'].all()
        assert frame.loc[frame.valid,'vx'].min()>=5
    audit=json.loads((dest/'raw_bag_audit.json').read_text())
    assert len(audit)==2
    assert all(check['maxAbsoluteDifference']<1e-10 for drive in audit for check in drive['exportChecks'].values())
    result=dict(status='passed',recomputedScoreRows=len(expected),
        maximumAbsoluteMetricDifference=max_difference,verifiedInputHashes=len(inputs),
        rawBagExportChecks=6,finitePositiveCandidates=len(candidates),
        qualityAndSpeedMasksPassed=True,syntheticRecovery=summary['syntheticRecovery'])
    (dest/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
