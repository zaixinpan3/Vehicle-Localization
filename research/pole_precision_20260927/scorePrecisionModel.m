function score=scorePrecisionModel(features,model)
% scorePrecisionModel: Portable research inference without ML toolboxes.
% Feature ordering and zero-imputation match the frozen Python export.
    x=double(features);x(~isfinite(x))=0;
    assert(size(x,2)==numel(model.featureNames),'Feature schema mismatch.');
    logit=repmat(model.initialLogit,size(x,1),1);row=(1:size(x,1)).';
    for k=1:numel(model.trees)
        t=model.trees(k);node=ones(size(row));pending=~t.leaf(node);
        while any(pending)
            i=find(pending);at=node(i);f=t.feature(at);threshold=t.threshold(at);
            left=x(sub2ind(size(x),i,f))<=threshold;
            next=t.right(at);next(left)=t.left(at(left));node(i)=next;
            pending=~t.leaf(node);
        end
        logit=logit+t.value(node);
    end
    score=1./(1+exp(-logit));
end
