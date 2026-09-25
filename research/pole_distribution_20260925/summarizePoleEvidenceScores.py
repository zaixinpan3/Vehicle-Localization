"""Descriptive, same-drive ranking of accepted candidates against fine masks."""
import json
from pathlib import Path
import numpy as np
from sklearn.metrics import average_precision_score, roc_auc_score

folder=Path(__file__).resolve().parent
rows=np.genfromtxt(folder/'pole_evidence_scores.csv',delimiter=',',names=True)
y=rows['fine']>0
report={'candidate_cells':len(rows),'fine_confirmed_cells':int(y.sum()),'frames':len(np.unique(rows['frame'])),
        'reference':'Existing offline fine detector; not manual physical ground truth',
        'scope':'Exploratory same-drive ranking; score chosen after inspecting these distributions',
        'scores':{}}
for name in ['oldProbability','distributionScore']:
    score=rows[name]; top_count=round(len(rows)*.2)
    cutoff=np.partition(score,-top_count)[-top_count]; above=score>cutoff; tied=score==cutoff
    expected_positive=float(y[above].sum()+(top_count-above.sum())*y[tied].mean())
    report['scores'][name]={'roc_auc':float(roc_auc_score(y,score)),
        'average_precision':float(average_precision_score(y,score)),
        'top_twenty_percent_count':top_count,
        'fine_fraction_in_top_twenty_percent':expected_positive/top_count,
        'top_twenty_percent_tie_policy':'Expected precision under uniform selection within cutoff ties',
        'cutoff_tied_cells':int(tied.sum()),
        'median_confirmed':float(np.median(score[y])),
        'median_unconfirmed':float(np.median(score[~y]))}
(folder/'evidence_ranking.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
