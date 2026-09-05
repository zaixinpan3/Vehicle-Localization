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
end
