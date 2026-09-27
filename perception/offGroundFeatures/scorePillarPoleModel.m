function score=scorePillarPoleModel(features,model)
% scorePillarPoleModel: Evaluate flattened distribution trees without toolboxes.
% All trees advance together; allocation scales with candidates, not raw points.
% Scores are reference-agreement ranks, not calibrated physical probabilities.
    x=double(features);x(~isfinite(x))=0;n=size(x,1);
    if isfield(model,'featureScale'),x=floor(x*model.featureScale+.5)/model.featureScale;end
    assert(size(x,2)==numel(model.featureNames),'perception:PoleFeatureSchema','Pole feature schema mismatch.');
    if n==0,score=zeros(0,1);return;end
    node=repmat(model.roots(:).',n,1);rows=repmat((1:n).',1,numel(model.roots));
    pending=~reshape(model.leaf(node),size(node));
    while any(pending,'all')
        at=node(pending);at=at(:);f=model.feature(at);sourceRows=rows(pending);
        value=x(sourceRows(:)+(f-1)*n);left=value(:)<=model.threshold(at);
        next=model.right(at);next(left)=model.left(at(left));node(pending)=next;
        pending=~reshape(model.leaf(node),size(node));
    end
    score=1./(1+exp(-(model.initialLogit+sum(reshape(model.value(node),size(node)),2))));
end
