function cfg=pillarPoleDistributionConfig()
% pillarPoleDistributionConfig: Joint raw-point statistics for 0.6 m poles.
% The numeric model is trained offline on fixed fine-reference pillar labels.
% Runtime reads only current-frame whole-pillar distributions and this model.
% Disabling this stage restores the preceding independent geometric vetoes.
% Reference-agreement scores are ranks, not calibrated physical probabilities.
% Precision is prioritized: the threshold targets <=5% prefix OOF false
% selections to leave margin for the <=10% regression objective. Coverage
% may fall below 80%. See research/pole_precision_priority_20260927.
    cfg=struct('enabled',true, ...
        'modelFile',fullfile(fileparts(mfilename('fullpath')),'polePillarDistributionModel.json'), ...
        'minimumScore',.9371209151674477,'minimumOwnerPoints',3,'minimumOwnerHeight',.15, ...
        'minimumLearnedRange',3);
end
