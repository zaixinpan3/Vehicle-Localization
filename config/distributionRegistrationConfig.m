function cfg = distributionRegistrationConfig()
% distributionRegistrationConfig: Local SE(2) D2D solver in meters/radians.
% Search limits describe the prediction basin, not global relocalization.
% Similarity/curvature gates are engineering checks, not calibrated confidence.
    cfg = struct();
    cfg.method = "geometricD2D";
    cfg.maximumIterationsPerScale = 40;
    cfg.maximumPoseCorrection = [3 3 deg2rad(12)];
    cfg.yawLeverArm = 10;
    cfg.gradientTolerance = 1e-5;
    cfg.stepTolerance = 1e-5;
    cfg.minimumSimilarity = 0.15;
    cfg.minimumComponents = 3;
    cfg.minimumScaledCurvature = 1e-5;
    cfg.minimumCurvatureRatio = 1e-4;
    % Pose is ALWAYS [X Y psi]. Retain full XYZ statistics but use height for
    % geometric correspondence compatibility only, opt-in with a map Z reference.
    cfg.heightMode = "xy";
    cfg.heightTranslation = NaN; % Moving origin in map Z; never assume zero.
    % Engineering uncertainty defaults, not calibrated sensor specifications.
    cfg.heightStandardDeviation = 0.20;
    cfg.tiltStandardDeviation = deg2rad(0.5);
    % Geometry-based Gaussian correspondence solver. These specify spatial
    % tolerance and observation quality, not dataset-specific feature rules.
    cfg.geometric = struct('maximumMatchDistance',2.5, ...
        'minimumLineAnisotropy',3,'noiseStandardDeviation',0.10, ...
        'robustStandardizedDistance',2.5,'minimumMatchFraction',0.10, ...
        'minimumObservabilityRatio',0.01,'maximumClassCorrection',0.50, ...
        'heightCompatibilitySigma',3);
end
