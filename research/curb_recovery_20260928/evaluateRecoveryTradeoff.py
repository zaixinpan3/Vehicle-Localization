"""Compare recorded-sequence validation of previously defined recovery rules.

Suffix counts are used to reject rules, so this is validation-guided policy
selection on a reused recording, not an untouched test or a generalization claim.
"""
from probeCurbContinuation import ROOT,P,OLD,OUT,support
import numpy as np,pandas as pd,json

a=pd.read_csv(OLD/'curb_predictions.csv');a=a[a.dataset=='mississippi'];b=pd.read_csv(OUT/'classifier_predictions.csv');features=pd.read_csv(OLD/'mississippi_curb_features.csv');t=features.merge(a[['frame','pillar','score','oof']],on=['frame','pillar'],validate='one_to_one').merge(b[['frame','pillar','score','oof']],on=['frame','pillar'],suffixes=('_old','_new'),validate='one_to_one');y=t.finePointCount.to_numpy();suffix=t.frame.to_numpy()>780;frame500=t.frame.to_numpy()==500
oldThreshold=json.loads((ROOT/'config/curbPillarPrecisionModel.json').read_text())['decisionThresholds']['mississippi'];newThreshold=json.loads((OUT/'mississippi_curb_model.json').read_text())['decisionThreshold'];base=t.score_old.to_numpy()>=oldThreshold;records=[];winner=None
for choice in json.loads((P/'recovery_6percent_trial.json').read_text())['alternatives']:
 if choice['falseFraction']>.06:continue
 key=(choice['radius'],choice['normalTolerance'])
 if key!=globals().get('lastKey'):
  count,_=support(t,t.score_new.to_numpy(),newThreshold,*key,np.cos(np.pi/4));lastKey=key
 take=base|((t.score_old.to_numpy()>=choice['oldMinimumScore'])&(t.score_new.to_numpy()>=choice['newMinimumScore'])&(count>=choice['minimumSupport']));selected=suffix&take;n=int(selected.sum());false=int((selected&(y==0)).sum());pts=int(y[selected].sum());rec={**choice,'suffixSelected':n,'suffixFalse':false,'suffixFalseFraction':false/n,'suffixCoveredPoints':pts,'frame500CoveredPoints':int(y[take&frame500].sum()),'frame500False':int((take&frame500&(y==0)).sum())};records.append(rec)
 if false/n<=.1 and (winner is None or choice['coveredPoints']>winner[0]['coveredPoints']):winner=(rec,take.copy())
assert winner;choice,take=winner;print(json.dumps(choice),flush=True)
(P/'validation_tradeoff.json').write_text(json.dumps(dict(selection='Among the predefined prefix-OOF <=6% rules, maximize prefix coverage subject to <=10% observed suffix false selection. Suffix labels are used for policy validation/selection; no independent test claim.',winner=choice,alternatives=records),indent=2)+'\n')
t[['frame','pillar','finePointCount']].assign(accepted=take).to_csv(OUT/'selected_predictions.csv',index=False)
settings={k:choice[k] for k in ['radius','normalTolerance','oldMinimumScore','newMinimumScore','minimumSupport']};settings.update(seedThreshold=newThreshold,angleCos=float(np.cos(np.pi/4)),heightTolerance=.15,heightRangeSlope=.05)
(OUT/'selected_parameters.json').write_text(json.dumps(settings,indent=2)+'\n')
