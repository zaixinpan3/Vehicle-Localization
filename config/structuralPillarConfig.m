function cfg = structuralPillarConfig()
% structuralPillarConfig: Whole-pillar structural semantics in metric units.
% XYZ moments, total support and XY neighbors replace all subpillar gates.
    cfg = struct();

    % Whole-pillar point-versus-line shape scores (the voxel size comes from frameVoxelizationConfig)
    cfg.fineShapeScoreNeighborhoodRadiusCells = 2;
    cfg.fineShapeScoreWeightPower = 2.0;
    cfg.fineShapeScoreLinearityPower = 1.0;
    cfg.fineShapeScoreMinSeedCount = 3;
    cfg.fineShapeScoreSaturatedSeedCount = 5;

    % Traffic-sign channel: maximum return intensity per whole pillar
    cfg.trafficSignIntensityThreshold = 1600;

    % Facade line detection by oriented weighted Hough voting on the pillar map
    cfg.thetaResolutionDeg = 5;
    cfg.thetaPriorRangeDeg = [];
    cfg.orientationToleranceDeg = 25;
    cfg.scoreGamma = 0.05;
    cfg.supportGammaMin = 0.55;
    cfg.supportQuantile = 0.85;
    cfg.supportAbs = 0.20;
    cfg.maxManifoldWeight = 0.15;
    cfg.useLogAccumulator = true;
    cfg.facadeHoughLineExponent = 1.0;
    cfg.facadeHoughEnergyExponent = 1.0;
    cfg.facadeHoughVoteWeightQuantile = 0.95;
    cfg.houghPeakThresholdRatio = 0.70;
    cfg.houghPeakQuantizationScale = 100;
    cfg.maxHoughPeaks = 8;
    cfg.houghSuppressionSize = [25, 25];
    cfg.lineDistanceThresholdVoxels = 0.5;
    cfg.facadeLineFillGapVoxels = 15;
    cfg.facadeLineMinLengthVoxels = 18;
    cfg.splitValidationEnabled = false;
    cfg.minAssignedPixels = 10;
    cfg.minPeakLengthMeters = 0.0;
    cfg.mergeThetaTolDeg = 2.0;
    cfg.mergeRhoTolMeters = 1.0;
    cfg.maxOutputLines = 0;
    cfg.thetaClusterTolDeg = 5.0;
    cfg.maxThetaClusters = 3;
    cfg.maxLinesPerThetaCluster = 4;
    cfg.fitWeightExponent = 0.0;
    cfg.facadeMaskUseKeptPeaks = true;


    cfg.facadeRefineEnabled = false;
    cfg.facadeSupportRadiusCells = 2;
    cfg.pole = struct('minimumPoints',12,'minimumHeight',1.5, ...
        'minimumHeightStd',0.30,'maximumTiltDegrees',20, ...
        'maximumRadialStd',0.15,'minimumPointScore',0.70, ...
        'maximumLineScore',0.90,'minimumContextFraction',0.58, ...
        'maximumFootprintSpanCells',2);
end
