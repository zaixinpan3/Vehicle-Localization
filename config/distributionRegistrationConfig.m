function cfg = distributionRegistrationConfig()
% distributionRegistrationConfig: Local SE(2) D2D solver in meters/radians.
% Search limits describe the prediction basin, not global relocalization.
% Similarity/curvature gates are engineering checks, not calibrated confidence.
    cfg = struct();
    cfg.smoothingStandardDeviations = [1.0 0.35 0];
    cfg.maximumIterationsPerScale = 40;
    cfg.maximumPoseCorrection = [3 3 deg2rad(12)];
    cfg.yawLeverArm = 10;
    cfg.gradientTolerance = 1e-5;
    cfg.stepTolerance = 1e-5;
    cfg.minimumSimilarity = 0.15;
    cfg.minimumComponents = 3;
    cfg.minimumScaledCurvature = 1e-5;
    cfg.minimumCurvatureRatio = 1e-4;
    % Height is retained by default, but XYZ matching is opt-in until its
    % coarse/fine vertical sampling mismatch is calibrated on independent data.
    cfg.heightMode = "xy";
    cfg.heightTranslation = NaN; % Moving origin in map Z; never assume zero.
    % Engineering uncertainty defaults, not calibrated sensor specifications.
    cfg.heightStandardDeviation = 0.20;
    cfg.tiltStandardDeviation = deg2rad(0.5);
end
