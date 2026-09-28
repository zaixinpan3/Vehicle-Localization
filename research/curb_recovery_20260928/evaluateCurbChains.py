"""Record validation and the inspected development frame for fixed chain rules."""
from probeCurbChains import ROOT,P,OLD,OUT,chain_stats,choose_mask
import numpy as np,pandas as pd,json

t=pd.read_csv(OLD/'curb_predictions.csv');t=t[t.dataset=='mississippi'].reset_index(drop=True);strong=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];score=t.score.to_numpy();base=score>=strong;y=t.finePointCount.to_numpy();suffix=t.frame.to_numpy()>780;frame500=t.frame.to_numpy()==500;records=[];lastWeak=None
for r in json.loads((P/'chain_selection.json').read_text())['alternatives']:
 if r['weak']!=lastWeak:stats=chain_stats(t,score,strong,r['weak']);lastWeak=r['weak']
 take=choose_mask(stats,base,r);sel=take&suffix;n=int(sel.sum());false=int((sel&(y==0)).sum());covered=int(y[sel].sum());rec={**r,'suffixSelected':n,'suffixFalse':false,'suffixFalseFraction':false/n,'suffixCoveredPoints':covered,'frame500CoveredPoints':int(y[take&frame500].sum()),'frame500False':int((take&frame500&(y==0)).sum())};records.append(rec)
valid=[r for r in records if r['falseFraction']<=.07 and r['suffixFalseFraction']<=.1];valid.sort(key=lambda r:(r['frame500CoveredPoints'],r['coveredPoints']),reverse=True)
for r in valid[:12]:print(json.dumps(r),flush=True)
(P/'chain_validation_frontier.json').write_text(json.dumps(dict(scope='Reused-recording validation, including the inspected training-prefix frame 500. These results are not an independent test.',alternatives=records),indent=2)+'\n')
