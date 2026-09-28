function cfg=pillarPoleDistributionConfig(profile)
% pillarPoleDistributionConfig: Joint raw-point statistics for 0.6 m poles.
% The numeric model is trained offline on fixed fine-reference pillar labels.
% Runtime reads only current-frame whole-pillar distributions and this model.
% Disabling this stage restores the preceding independent geometric vetoes.
% Reference-agreement scores are ranks, not calibrated physical probabilities.
% The shared threshold targets <=5% prefix OOF reference-empty selections.
% Mississippi uses its own <=7.5% OOF budget, below the 10% requested limit;
% this recovers support otherwise lost to the stricter Downtown operating point.
% See research/frame827_perception_20260928 for the precision/coverage tradeoff.
    if nargin<1,profile="";end
    cfg=struct('enabled',true, ...
        'modelFile',fullfile(fileparts(mfilename('fullpath')),'polePillarDistributionModel.json'), ...
        'minimumScore',.9371209151674477,'minimumOwnerPoints',3,'minimumOwnerHeight',.15, ...
        'minimumLearnedRange',3);
    if any(lower(string(profile))==["mississippi","missisipi"])
        cfg.minimumScore=.8701683227450074;
    end
end
