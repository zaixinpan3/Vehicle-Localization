function cfg=geometryCandidateConfig(dataset,prefix)
% geometryCandidateConfig: Explicit experimental profile before promotion.
    if nargin<2,prefix='';end
    root=setupVehicleLocalization();cfg=perceptionConfig(dataset);
    modelFile=fullfile(root,'output','pole_geometry_20260927',[prefix 'candidate_model.json']);
    model=loadPillarPoleModel(modelFile);
    cfg.offGroundFeatures.pole.distributionValidation=struct('enabled',true, ...
        'modelFile',modelFile,'minimumScore',model.decisionThreshold, ...
        'minimumOwnerPoints',3,'minimumOwnerHeight',.15,'minimumLearnedRange',3);
end
