function cfg=pillarPoleDistributionConfig()
% pillarPoleDistributionConfig: Joint raw-point statistics for 0.6 m poles.
% The numeric model is trained offline on fixed fine-reference pillar labels.
% Runtime reads only current-frame whole-pillar distributions and this model.
% Disabling this stage restores the preceding independent geometric vetoes.
% Reference-agreement scores are ranks, not calibrated physical probabilities.
    cfg=struct('enabled',true, ...
        'modelFile',fullfile(fileparts(mfilename('fullpath')),'polePillarDistributionModel.json'), ...
        'minimumScore',.1488746496646483,'minimumOwnerPoints',3,'minimumOwnerHeight',.15, ...
        'minimumLearnedRange',3);
end
