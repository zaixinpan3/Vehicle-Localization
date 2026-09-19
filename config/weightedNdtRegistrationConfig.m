function cfg=weightedNdtRegistrationConfig()
% weightedNdtRegistrationConfig D2D-NDT overlap with the stored semantic GMM.
% Experimental opt-in; see research/weighted_ndt_20260919/README.md for results.
% Map mixtureWeight already contains map stability. Never multiply it by an
% additional repeatability factor. Search is local SE(2), not relocalization.
    cfg=struct();
    cfg.method="weightedNdt";
    cfg.maximumIterationsPerScale=40;
    cfg.maximumPoseCorrection=[3 3 deg2rad(12)];
    cfg.yawLeverArm=10;
    cfg.gradientTolerance=1e-7;
    cfg.stepTolerance=1e-7;
    cfg.minimumSimilarity=.15;
    cfg.minimumComponents=3;
    cfg.minimumScaledCurvature=1e-5;
    cfg.heightMode="xy";
    cfg.heightTranslation=NaN;
    cfg.heightStandardDeviation=.20;
    cfg.tiltStandardDeviation=deg2rad(.5);
    % Background density defines the coverage diagnostic only. It is not in
    % the overlap objective and is not another stability weight. Objective
    % scaling conditions the optimizer; reported curvature stays unscaled.
    cfg.ndt=struct('backgroundDensity',1e-5,'noiseStandardDeviation',.10, ...
        'objectiveScale',1000, ...
        'heightNoiseStandardDeviation',.10,'minimumInlierProbability',.5, ...
        'minimumMatchFraction',.10,'minimumObservabilityRatio',.01,'maximumClassCorrection',.50);
end
