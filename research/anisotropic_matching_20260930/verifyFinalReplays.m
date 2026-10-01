function verifyFinalReplays()
% verifyFinalReplays Assert unchanged default and exact continuous-mode output.
    dest=fileparts(mfilename('fullpath'));out='output/anisotropic_matching_20260930';
    baseline=readtable('research/source_shape_matching_20260929/final_raw.csv');
    default=readtable(fullfile(dest,'default_verified.csv'));
    initial=readtable(fullfile(dest,'full_covariance.csv'));
    final=readtable(fullfile(dest,'continuous_verified.csv'));
    pose={'x','y','psi'};
    defaultDifference=max(abs(default{:,pose}-baseline{:,pose}),[],'all');
    continuousDifference=max(abs(final{:,pose}-initial{:,pose}),[],'all');
    assert(height(final)==1170 && height(default)==1170);
    assert(defaultDifference<1e-7 && continuousDifference<1e-7);
    assert(isequal(default.accepted,baseline.accepted)&&isequal(default.directional,baseline.directional));
    assert(isequal(final.reason,initial.reason)&&isequal(final.rank,initial.rank));
    report=table(defaultDifference,continuousDifference,height(final), ...
        VariableNames={'maximumDefaultPoseDifference','maximumContinuousPoseDifference','frames'});
    writetable(report,fullfile(dest,'replay_verification.csv'));disp(report);
    save(fullfile(out,'verification.mat'),'report');
end
