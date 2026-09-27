"""Export the frozen histogram-boosting model for toolbox-free MATLAB replay."""
from pathlib import Path
import json,pickle,sys
import numpy as np
from trainContextPrecision import expanded

P=Path(__file__).resolve().parent;OUT=P.parents[1]/'output/pole_precision_20260927'

def main():
    whole='--whole' in sys.argv;prefix='whole' if whole else 'context'
    freeze=json.loads((P/f'{prefix}_model_freeze.json').read_text())
    name=freeze['winner']['model']
    with (OUT/f'{name}.pickle').open('rb') as f:bundle=pickle.load(f)
    model=bundle['model'];trees=[]
    for predictors in model._predictors:
        assert len(predictors)==1
        n=predictors[0].nodes
        trees.append(dict(feature=(n['feature_idx'].astype(int)+1).tolist(),threshold=n['num_threshold'].tolist(),
            left=(n['left'].astype(int)+1).tolist(),right=(n['right'].astype(int)+1).tolist(),
            value=n['value'].tolist(),leaf=n['is_leaf'].astype(bool).tolist()))
    result=dict(featureNames=bundle['names'],threshold=bundle['threshold'],
        initialLogit=float(model._baseline_prediction[0,0]),trees=trees,model=name,
        status=f'Research candidate; validation limitations in {prefix}_model_validation.json')
    (P/f'{prefix}_candidate_model.json').write_text(json.dumps(result,separators=(',',':'))+'\n')
    _,_,names,X,*_=expanded(whole);features=X[:,[names.index(n) for n in bundle['names']]]
    scores=model.predict_proba(features)[:,1]
    # Full replay matrix has no labels or identifiers; it is a generated cache.
    np.savetxt(OUT/f'{prefix}_model_replay_inputs.csv',features,delimiter=',')
    np.savetxt(OUT/f'{prefix}_model_replay_expected.csv',scores,delimiter=',')
    print(json.dumps(dict(model=name,rows=len(features),features=features.shape[1],trees=len(trees))))

if __name__=='__main__':main()
