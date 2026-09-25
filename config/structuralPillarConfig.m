function cfg = structuralPillarConfig(spacing)
% structuralPillarConfig: Whole-pillar structural semantics in metric units.
% XYZ moments, total support and XY neighbors replace all subpillar gates.
    if nargin < 1, spacing = pillarGridConfig().voxelSize(1); end
    cfg = struct();

    % Whole-pillar point-versus-line shape scores (XY spacing comes from pillarGridConfig)
    cfg.fineShapeScoreNeighborhoodRadiusCells = 2;
    cfg.fineShapeScoreWeightPower = 2.0;
    cfg.fineShapeScoreLinearityPower = 1.0;
    cfg.fineShapeScoreMinSeedCount = 3;
    cfg.fineShapeScoreSaturatedSeedCount = 5;

    % Traffic-sign channel: maximum return intensity per whole pillar
    cfg.trafficSignIntensityThreshold = 1800;

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
    cfg.minAssignedPixels = latticeTunedValue(spacing, 10, 5);
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
    cfg.facadeSupportRadiusCells = latticeTunedValue(spacing, 2, 1);
    % Density core: the share of a pillar's returns within coreRadius of its
    % XY density peak and the height those returns span. On 0.6 m pillars the
    % core separates a shaft from what shares its pillar; the 0.3 m lattice
    % keeps its radial-scatter gate and leaves the core gate at zero.
    cfg.pole = struct('minimumPoints',12,'minimumHeight',1.5, ...
        'minimumHeightStd',0.30,'maximumTiltDegrees',20, ...
        'maximumRadialStd',latticeTunedValue(spacing,0.15,0.25), ...
        'densityPeakBinMeters',0.10,'coreRadius',0.15, ...
        'minimumCoreFraction',latticeTunedValue(spacing,0,0.55), ...
        'minimumCoreHeight',latticeTunedValue(spacing,0,1.5), ...
        'minimumPointScore',latticeTunedValue(spacing,0.70,0.60), ...
        'maximumLineScore',0.90,'minimumContextFraction',latticeTunedValue(spacing,0.58,0.52), ...
        'maximumFootprintSpanCells',2);
end
