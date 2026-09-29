function cfg = distributionRegistrationConfig()
% distributionRegistrationConfig: Local SE(2) D2D solver in meters/radians.
% Search limits describe the prediction basin, not global relocalization.
% Similarity/curvature gates are engineering checks, not calibrated confidence.
    cfg = struct();
    cfg.method = "geometricD2D";
    cfg.localMapRadius = 100;
    % Canonical point clouds seed the fine solve. A matched pole can protect
    % the coarse basin; sign-only merged centroids cannot veto refinement.
    cfg.pyramid = struct('mapMergeRadius',1.5,'sourceMergeRadius',0.5, ...
        'trustRadius',0.15,'pointClasses',["pole","trafficSign"],'trustClasses',"pole");
    % Ambiguous neighboring point landmarks contribute a mean and total
    % covariance, including between-mode scatter. Temper geometry only; retain
    % stored map priors and confident associations. Two distinct unmerged pole
    % anchors restore the hard fine solve, avoiding bias from nearby aliases.
    cfg.softPointAssociation = struct('temperature',2.5,'radius',1.5, ...
        'minimumPosterior',0.9,'hardWithUnmergedAnchors',2);
    % Align reliable local line directions as well as line-normal positions.
    % Several distinct selected means must span a straight neighborhood; no
    % raw points or additional perception candidates enter this factor. The
    % angular scale is an engineering setting, not calibrated sensor noise.
    % Local minor/major scatter increases angular uncertainty on bent lines.
    cfg.lineDirection = struct('enabled',true,'radius',4,'minimumComponents',3, ...
        'minimumAnisotropy',9,'minimumSpan',2.4,'standardDeviation',deg2rad(1), ...
        'scatterScale',0.25);
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
    % Position aiding selects between LiDAR-only optima. It contributes no
    % residual, pose force or information to the conditional LiDAR solution.
    cfg.positionAid = struct('maximumStandardDeviation',2, ...
        'standardDeviationFloor',0.10,'minimumSeedSeparation',0.05, ...
        'hypothesisSeparation',0.02,'maximumSquaredInnovation',9.21);
    % Engineering uncertainty defaults, not calibrated sensor specifications.
    cfg.heightStandardDeviation = 0.20;
    cfg.tiltStandardDeviation = deg2rad(0.5);
    % Optional relative-height evidence changes association, not XY residuals.
    % A common Z offset is inferred from spatially separated curb matches.
    cfg.relativeHeight = struct('enabled',false,'minimumAnchors',4, ...
        'minimumAnchorSpan',3,'maximumAnchorDistance',0.75, ...
        'noiseStandardDeviation',0.20,'tiltStandardDeviation',deg2rad(0.5), ...
        'weight',1,'maximumPenalty',9,'candidateModel',"marginal",'semanticNames',["pole","trafficSign"]);
    % Geometry-based Gaussian correspondence solver. These specify spatial
    % tolerance and observation quality, not dataset-specific feature rules.
    % maximumClassCorrection gates the information-weighted correction norm;
    % unsupported class directions cannot independently reject an entire scan.
    cfg.geometric = struct('maximumMatchDistance',2.5, ...
        'minimumLineAnisotropy',3,'noiseStandardDeviation',0.10, ...
        'robustStandardizedDistance',2.5,'minimumMatchFraction',0.10, ...
        'minimumObservabilityRatio',0.01,'maximumClassCorrection',0.50, ...
        'heightCompatibilitySigma',3);
end
