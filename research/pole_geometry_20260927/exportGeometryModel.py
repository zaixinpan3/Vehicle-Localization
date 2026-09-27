"""Export compact tree arrays and independent numerical replay fixtures."""
from pathlib import Path
import json,pickle,sys
import numpy as np
from trainGeometryModel import load,P,OUT

def main():
    moments='--moments' in sys.argv;stable='--stable' in sys.argv;prefix=('stable_' if stable else '')+('moments_' if moments else '')
    frozen=json.loads((P/f'{prefix}model_freeze.json').read_text());winner=frozen['winner']
    bundle=pickle.load((OUT/(winner['model']+'.pickle')).open('rb'));model=bundle['model']
    used=sorted({int(row['feature_idx']) for predictors in model._predictors for row in predictors[0].nodes if not row['is_leaf']})
    remap={value:index+1 for index,value in enumerate(used)}
    result=dict(schemaVersion=1,featureNames=[bundle['names'][i] for i in used],decisionThreshold=winner['pareto']['threshold'],
        initialLogit=float(model._baseline_prediction[0,0]),roots=[],feature=[],threshold=[],left=[],right=[],value=[],leaf=[],
        model=winner['model'],training='Mississippi <=780 and Downtown <=360; seed 927; original fine pole points are offline labels only')
    if stable:result['featureScale']=1e8
    for predictors in model._predictors:
        assert len(predictors)==1;n=predictors[0].nodes;offset=len(result['feature']);result['roots'].append(offset+1)
        result['feature']+=[remap[int(row['feature_idx'])] if not row['is_leaf'] else 1 for row in n];result['threshold']+=n['num_threshold'].tolist()
        result['left']+=(n['left'].astype(int)+offset+1).tolist();result['right']+=(n['right'].astype(int)+offset+1).tolist()
        result['value']+=n['value'].tolist();result['leaf']+=n['is_leaf'].astype(bool).tolist()
    dest=OUT/f'{prefix}candidate_model.json';dest.write_text(json.dumps(result,separators=(',',':'))+'\n')
    _,_,names,X,*_=load(moments,stable);features=X[:,[names.index(n) for n in bundle['names']]]
    np.savetxt(OUT/f'{prefix}replay_inputs.csv',features[:,used],delimiter=',');np.savetxt(OUT/f'{prefix}replay_expected.csv',model.predict_proba(features)[:,1],delimiter=',')
    print(json.dumps(dict(file=str(dest),rows=len(features),features=len(used),trees=len(result['roots']))))

if __name__=='__main__':main()
