function cfg=pillarPoleDistributionConfig(profile)
% pillarPoleDistributionConfig: Joint raw-point statistics for 0.6 m poles.
% The numeric model is trained offline on fixed fine-reference pillar labels.
% Runtime reads only current-frame whole-pillar distributions and this model.
% Disabling this stage restores the preceding independent geometric vetoes.
% Reference-agreement scores are ranks, not calibrated physical probabilities.
% The shared model/profile remains unchanged for Downtown. Mississippi uses
% a specialist fitted only on its prefix, with the same whole-pillar feature
% schema and a <=7.5% purged-OOF reference-empty operating point. No frame IDs,
% reference points or extra model are used at runtime. See
% research/pole_selective_recovery_20260928 for validation and suffix limits.
    if nargin<1,profile="";end
    cfg=struct('enabled',true, ...
        'modelFile',fullfile(fileparts(mfilename('fullpath')),'polePillarDistributionModel.json'), ...
        'minimumScore',.9371209151674477,'minimumOwnerPoints',3,'minimumOwnerHeight',.15, ...
        'minimumLearnedRange',3);
    if any(lower(string(profile))==["mississippi","missisipi"])
        cfg.modelFile=fullfile(fileparts(mfilename('fullpath')),'mississippiPolePillarDistributionModel.json');
        cfg.minimumScore=.8747229048074372;
        % Preserve a clearly separated continuous shaft when most owner
        % returns belong to other-height clutter, outside the learned mix.
        cfg.minorityShaftProtection=struct('minimumHeight',2.5, ...
            'minimumContinuousFraction',.9,'maximumRadialRms',.04, ...
            'minimumIsolation',.95,'maximumOwnerSupportFraction',.2, ...
            'minimumOwnerHeight',2,'minimumQuarterCount',4, ...
            'maximumQuarterCenterStep',.08,'minimumCoreFraction',.9);
    end
end
