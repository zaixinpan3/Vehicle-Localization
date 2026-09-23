function result=matchLocalProbabilityCloud(map,source,seed,cfg,positionAid)
% matchLocalProbabilityCloud Crop a semantic map and retain global pair indices.
% The source is the confirmed coarse horizon, already in current body axes.
    if nargin<5,positionAid=[];end
    c=map.components;keep=sum((c.mean(:,1:2)-seed(1:2)).^2,2)<=cfg.localMapRadius^2;
    fixed=map;fixed.components.mean=c.mean(keep,:);
    fixed.components.covariance=c.covariance(:,:,keep);
    fixed.components.numComponents=nnz(keep);
    for name=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(c,name),fixed.components.(name)=c.(name)(keep,:);end
    end
    timer=tic;result=registerSemanticProbabilityCloud(fixed,source,seed,cfg,positionAid);
    result.matchingSeconds=toc(timer);
    if isfield(result,'correspondences')
        indices=find(keep);result.correspondences.globalTarget=indices(result.correspondences.target);
    end
end
