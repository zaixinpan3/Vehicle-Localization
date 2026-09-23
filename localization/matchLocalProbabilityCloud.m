function result=matchLocalProbabilityCloud(map,source,seed,cfg,positionAid,additionalSeeds)
% matchLocalProbabilityCloud Crop a semantic map and retain global pair indices.
% The source is the confirmed coarse horizon, already in current body axes.
    if nargin<5,positionAid=[];end
    if nargin<6,additionalSeeds=zeros(0,3);end
    [fixed,indices]=selectLocalProbabilityCloud(map,seed,cfg.localMapRadius);
    timer=tic;result=registerSemanticProbabilityCloud(fixed,source,seed,cfg,positionAid,additionalSeeds);
    result.matchingSeconds=toc(timer);
    if isfield(result,'correspondences')
        result.correspondences.globalTarget=indices(result.correspondences.target);
    end
end
