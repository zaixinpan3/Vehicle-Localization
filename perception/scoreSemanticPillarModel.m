function score=scoreSemanticPillarModel(features,model,threshold)
% scoreSemanticPillarModel: Toolbox-free distribution agreement classifiers.
% Forest votes use float32 inputs, matching the fitted extra-tree predictor.
% A conservative facade vote also requires agreement of nearby distributions
% in the offline training bank. The bank contains no frame/point/pillar IDs.
% Scores rank reference agreement; they are not physical class probabilities.
    if ~isfield(model,'kind') || string(model.kind)~="forestNeighborConsensus"
        score=scorePillarPoleModel(features,model);return;
    end
    x=double(features);x(~isfinite(x))=0;
    x=floor(x*model.featureScale+.5)/model.featureScale;n=size(x,1);
    assert(size(x,2)==numel(model.featureNames),'perception:SemanticFeatureSchema', ...
        'Semantic precision feature schema mismatch.');
    score=zeros(n,1);if n==0,return;end
    treeX=single(x);node=repmat(model.roots(:).',n,1);rows=repmat((1:n).',1,numel(model.roots));
    pending=~reshape(model.leaf(node),size(node));
    while any(pending,'all')
        at=node(pending);at=at(:);f=model.feature(at);sourceRows=rows(pending);
        value=double(treeX(sourceRows(:)+(f-1)*n));left=value(:)<=model.threshold(at);
        next=model.right(at);next(left)=model.left(at(left));node(pending)=next;
        pending=~reshape(model.leaf(node),size(node));
    end
    score=mean(reshape(model.value(node),size(node)),2);
    % Skip the neighbor query for candidates already rejected by the forest.
    query=find(score>=threshold);if isempty(query),return;end
    scaled=(x(query,:)-model.neighborCenter(:).')./model.neighborScale(:).';
    bank=model.neighborVectors;bankNorm=sum(bank.^2,2).';
    % Bound temporary storage on dense scenes; base MATLAB supplies mink.
    for start=1:64:numel(query)
        at=start:min(start+63,numel(query));q=scaled(at,:);
        distance=max(0,sum(q.^2,2)+bankNorm-2*(q*bank.'));
        [~,nearest]=mink(distance,model.neighborCount,2);
        agreement=mean(reshape(double(model.neighborPositive(nearest)),size(nearest)),2);
        score(query(at(agreement<model.neighborThreshold)))=0;
    end
end
