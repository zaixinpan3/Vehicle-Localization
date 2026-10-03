function cfg = facadeSurfaceConfig(spacing)
% facadeSurfaceConfig: Offline wall geometry aligned with the tuned V12
% RobustVehicleLocalization Downtown detector (2026-10-03). The same metric
% support rules operate inside this invocation's coarse candidate envelope.
% Sparse height samples and individual residuals belong to fine perception;
% the online classifier does not invoke this stage. Reflective points
% use the joint Downtown threshold before structural evidence is computed.
%
% Input: spacing: existing XY pillar spacing in meters (0.3 or 0.6).
% Output: cfg: Hough, seed-patch and connected-plane support parameters.
    cfg = struct();
    cfg.heightResolution = 0.5;
    cfg.trafficSignIntensityThreshold = 1600;
    cfg.hough = rmfield(structuralPillarConfig(spacing), 'pole');
    cfg.hough.supportQuantile = 0.50;
    cfg.hough.houghSuppressionSize = [1 25];
    cfg.hough.houghPeakThresholdRatio = 0.25;
    cfg.hough.maxHoughPeaks = 32;
    cfg.hough.minAssignedPixels = max(5,round(10*0.3/spacing));
    cfg.hough.useNativeKernels = false;
    cfg.seed = struct('supportRadiusMeters',0.9,'patchSizeMeters',[3 3 4], ...
        'minimumVoxels',8,'minimumPoints',16,'minimumPlanarity',0.30, ...
        'maximumNormalAngleDegrees',20,'maximumNormalTiltDegrees',10, ...
        'minimumPatchHeight',2.5,'minimumColumnHeight',2.5,'minimumColumnLayers',5);
    primary = struct('enabled',true,'candidateBandMeters',0.8,'maxExtensionMeters',12, ...
        'maxPointDistanceMeters',0.08,'maxNormalAngleDeg',10,'maxNumTrials',1000, ...
        'randomSeed',0,'connectivityCellMeters',0.5,'minComponentPoints',200, ...
        'minComponentHeightMeters',3,'minComponentWidthMeters',5, ...
        'minAnchorPoints',80,'minAnchorHeightMeters',2.5,'minAnchorWidthMeters',1.5, ...
        'minAnchorColumns',max(1,round(8*0.3/spacing)), ...
        'localFitWindowMeters',8,'localFitStepMeters',4,'requireSeedPlaneValidation',true);
    panel = primary;
    panel.candidateBandMeters = 1.5;
    panel.maxPointDistanceMeters = 0.15;
    panel.connectivityCellMeters = 0.75;
    panel.minComponentPoints = 80;
    panel.minComponentHeightMeters = 4.5;
    panel.minComponentWidthMeters = 5;
    panel.minComponentAreaMeters2 = 20;
    panel.minAnchorPoints = 40;
    panel.minAnchorHeightMeters = 4;
    panel.minAnchorWidthMeters = 2.5;
    panel.minAnchorAreaMeters2 = 10;
    panel.minAnchorColumns = max(1,round(10*0.3/spacing));
    panel.parallelAnchorAngleDeg = 20;
    panel.requireAnchorsInFitWindow = false;
    panel.localSectionHeightMeters = 12;
    panel.localSectionStepMeters = 6;
    cfg.completion = primary;
    cfg.completion.tallWall = panel;
    if spacing>0.3
        % Selected on six development frames by point/pillar agreement;
        % connected-plane dimensions and residual limits remain unchanged.
        cfg.hough.fineShapeScoreNeighborhoodRadiusCells = 1;
        cfg.hough.fineShapeScoreMinSeedCount = 2;
        cfg.hough.fineShapeScoreSaturatedSeedCount = 3;
        cfg.hough.supportGammaMin = 0.4;
        cfg.hough.supportQuantile = 0.3;
        cfg.hough.maxHoughPeaks = 48;
        cfg.hough.minAssignedPixels = 3;
        cfg.seed.minimumPlanarity = 0.25;
        cfg.seed.supportRadiusMeters = 0.6;
    end
end
