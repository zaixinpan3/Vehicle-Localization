function cfg=baselineMatchingConfig()
% baselineMatchingConfig Restore the pre-study solver settings explicitly.
    cfg=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')),'baseline_config.json')));
    cfg.method=string(cfg.method);cfg.heightMode=string(cfg.heightMode);cfg.heightTranslation=NaN;
    cfg.maximumPoseCorrection=cfg.maximumPoseCorrection(:).';cfg.pyramid.pointClasses=string(cfg.pyramid.pointClasses(:).');
    cfg.relativeHeight.candidateModel=string(cfg.relativeHeight.candidateModel);
    cfg.relativeHeight.semanticNames=string(cfg.relativeHeight.semanticNames(:).');
end
