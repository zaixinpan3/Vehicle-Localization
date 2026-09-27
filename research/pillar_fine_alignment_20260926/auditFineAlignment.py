"""Independently audit exported counts, fixed targets, sources and timing pairs."""
from pathlib import Path
import csv
import hashlib
import json

ROOT=Path(__file__).resolve().parents[2]
FOLDER=Path(__file__).resolve().parent


def rows(name):
    with (FOLDER/name).open() as f:
        return list(csv.DictReader(f))


def main():
    audit={}
    for label, old, new, expected in [
        ('Mississippi','baseline_frames.csv','final_full_frames.csv',1170),
        ('Downtown','baseline_downtown_frames.csv','final_downtown_frames.csv',24),
    ]:
        before,after=rows(old),rows(new)
        assert len(before)==len(after)==expected
        for x,y in zip(before,after):
            for key in ['frame','finePointCount','finePointCountInRoi','finePointCountOutsideRoi','finePillarCount']:
                assert x[key]==y[key],(label,key,x['frame'])
            v={key:int(y[key]) for key in ['finePointCountInRoi','coveredFinePointCount','finePillarCount',
                                          'coveredFinePillarCount','candidatePillarCount','extraPillarCount']}
            assert v['candidatePillarCount']==v['coveredFinePillarCount']+v['extraPillarCount']
            assert v['coveredFinePointCount']<=v['finePointCountInRoi']
            assert v['coveredFinePillarCount']<=v['finePillarCount']
            assert y['nonPoleComponentsUnchanged']=='1'
            if v['finePointCountInRoi']:
                assert abs(float(y['pointCoverage'])-v['coveredFinePointCount']/v['finePointCountInRoi'])<1e-12
        audit[label]=dict(frames=expected,fixedReferenceDenominators=True,countIdentities=True,
                          exportedRatios=True,nonPoleChecks=True)
    frozen=json.loads((FOLDER/'final_implementation_freeze.json').read_text())
    assert all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==h for name,h in frozen['sha256'].items())
    audit['frozenProductionSourcesMatch']=True
    timing=rows('paired_runtime.csv')
    assert len(timing)==288
    for i in range(0,len(timing),2):
        a,b=timing[i:i+2]
        assert all(a[key]==b[key] for key in ['dataset','frame','repeat']) and a['variant']!=b['variant']
    audit['pairedTimingCalls']=288
    (FOLDER/'arithmetic_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    print(json.dumps(audit,indent=2))


if __name__=='__main__':
    main()
