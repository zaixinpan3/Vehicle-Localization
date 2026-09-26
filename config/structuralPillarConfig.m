function cfg = structuralPillarConfig(spacing)
% structuralPillarConfig: Whole-pillar structural semantics in metric units.
% All-point moments and raw-point support share the same XY pillar lattice.
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
    % XY density peak, the height those returns span, and their isolation
    % (core returns of the pillar and its neighbours over all returns
    % within isolationRadius of the peak). On 0.6 m pillars the core separates
    % a shaft from what shares its pillar and the isolation replaces the
    % pillar-ring context, whose 1.8 m window rejected most poles beside
    % hedges and canopies, and a lone pillar adopts the edge neighbour whose
    % peak lies within shaftCompletionDistance (the other half of the same
    % shaft); the 0.3 m lattice keeps its radial-scatter and ring-context
    % gates and leaves the core gates and the completion distance at zero.
    % Coarse cores need 12 actual returns in the metric support, including
    % a boundary partner. Their isolation replaces the spacing-dependent
    % point/line shape gates; these scores still weight the output semantics.
    cfg.pole = struct('minimumPoints',12,'minimumHeight',1.5, ...
        'minimumHeightStd',0.30,'maximumTiltDegrees',20, ...
        'maximumRadialStd',latticeTunedValue(spacing,0.15,0.25), ...
        'densityPeakBinMeters',0.10,'coreRadius',0.15,'isolationRadius',0.60, ...
        'minimumCoreFraction',latticeTunedValue(spacing,0,0.35), ...
        'minimumCoreHeight',latticeTunedValue(spacing,0,1.5), ...
        'minimumCoreIsolation',latticeTunedValue(spacing,0,0.35), ...
        'minimumCorePoints',latticeTunedValue(spacing,0,12), ...
        'shaftCompletionDistance',latticeTunedValue(spacing,0,0.30), ...
        'minimumPointScore',latticeTunedValue(spacing,0.70,0), ...
        'maximumLineScore',latticeTunedValue(spacing,0.90,1),'minimumContextFraction',latticeTunedValue(spacing,0.58,0), ...
        'maximumFootprintSpanCells',2);
    % Confidence ranks accepted pillars by continuous raw-point concentration
    % and height continuity. Detection and all-point output moments stay
    % separate: a high density peak alone cannot saturate pole confidence.
    cfg.pole.probabilityEvidence="shape";
    if spacing>0.3, cfg.pole.probabilityEvidence="distribution"; end
    cfg.pole.distribution=struct('coreRadius',0.25,'contextRadius',0.75, ...
        'minimumRobustHeight',1.5,'minimumCorePoints',12);

    % Coarse pillars retain established detections and add independently
    % supported shaft modes. The earlier subset experiment remains selectable
    % for reproducible comparisons; the offline 0.3 m branch stays unchanged.
    cfg.pole.detector="pillar";
    cfg.pole.shaft=pillarShaftConfig();
    if spacing>0.3
        cfg.pole.detector="shaft";
        cfg.pole.probabilityEvidence="shaft";
    end
    cfg.pole.subset=struct('radii',[0.06 0.10 0.15], ...
        'maximumSeeds',12,'seedSeparation',0.08,'fitRadius',0.15, ...
        'fitHeight',2.0,'heightHypotheses',4,'neighborhoodRadius',0.75, ...
        'minimumPoints',12,'minimumOwnPoints',6,'minimumOwnHeight',0.5, ...
        'minimumHeight',1.5,'minimumRobustHeight',1.2, ...
        'maximumTiltDegrees',20,'maximumGap',0.5,'rangeGapScale',0.005, ...
        'heightHalfWindow',0.5,'minimumWindowPoints',2,'minimumContrast',4,'minimumPeakContrast',1.3, ...
        'maximumRadialRms',0.10,'saturatedHeight',2.0, ...
        'saturatedPoints',20,'saturatedContrast',6);

end
