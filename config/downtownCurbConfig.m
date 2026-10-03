function cfg = downtownCurbConfig(spacing)
% downtownCurbConfig: Downtown coarse evidence and offline point-validation gates.
% Metric tolerances match the tuned joint reviewV12 reference. Cell radii
% retain their physical support on the invocation's native lattice. Point
% gates run only in offline fine validation within immutable coarse candidates.
    if nargin<1, spacing=.3; end
    assert(isscalar(spacing) && isfinite(spacing) && spacing>0);
    cfg.groundOverrides = struct('dominantBoundaryContinuationSearchRows',0, ...
        'dominantBoundaryContinuationMinTotalEnergy',.20);
    cfg.denseMinBaseEnergy = 0.75;
    cfg.denseMinResidualMeters = 0.025;
    cfg.denseMinRoughnessMeters = 0.030;
    cfg.denseNearRoadFilterEnabled = true;
    cfg.denseNearRoadRadiusCells = 2;
    cfg.boundaryColumnMinPoints = 2;
    cfg.boundaryColumnSearchRadiusCells = 3;
    cfg.boundaryColumnTrackingEnabled = true;
    cfg.boundaryMinSidePoints = 12;
    cfg.boundaryNeighborRows = 1;
    cfg.boundaryPointFilterEnabled = true;
    cfg.boundaryPointQuantile = 0.50;
    cfg.boundaryRoadAwayNeighborMarginMeters = Inf;
    cfg.boundaryRoadFacingCellMarginMeters = 0.12;
    cfg.boundaryRoadReferenceFilterEnabled = true;
    cfg.boundaryRoadReferenceRadiusCells = 16;
    cfg.boundaryRowMinCountFraction = 0.15;
    cfg.boundaryWeakSupportFilterEnabled = true;
    cfg.boundaryWeakSupportHighStepMinMeters = 0.08;
    cfg.boundaryWeakSupportMinBaseEnergy = 0.12;
    cfg.boundaryWeakSupportMinCenterEvidence = 0.75;
    cfg.boundaryWeakSupportMinLinearity = 0.10;
    cfg.boundaryWeakSupportMinRoughnessMeters = 0.010;
    cfg.boundaryYBandMeters = 0.06;
    cfg.denseNearRoadRadiusCells = max(1,round(.6/spacing));
    cfg.boundaryColumnSearchRadiusCells = max(1,round(.9/spacing));
    cfg.boundaryRoadReferenceRadiusCells = max(1,round(4.8/spacing));
    cfg.pointSupport = pointSupportParameters();
    cfg.pointSupport.minOccupiedCells = max(2,ceil(1.2/spacing));
    cfg.obstacleClearance = obstacleParameters();
    cfg.endpointExtension = endpointParameters();
end

function cfg = pointSupportParameters()
    cfg.enabled = true;
    cfg.minPoints = 4;
    cfg.minOccupiedCells = 4;
    cfg.minExtentMeters = 1.0;
end

function cfg = obstacleParameters()
    cfg.enabled = true;
    cfg.searchRadiusMeters = 0.75;
    cfg.minHeightAboveCandidateMeters = 0.4;
    cfg.maxHeightAboveCandidateMeters = 2.2;
    cfg.minElevatedPoints = 20;
    cfg.minVerticalSpanMeters = 0.5;
    cfg.heightBinMeters = 0.2;
    cfg.minOccupiedHeightBins = 3;
    cfg.minBlockedFraction = 0.5;
end

function cfg = endpointParameters()
    cfg.enabled = true;
    cfg.directions = [1,-1];
    cfg.minAnchorPoints = 30;
    cfg.minAnchorLengthMeters = 4.0;
    cfg.maxAnchorBlockedFraction = 0.10;
    cfg.fitLengthMeters = 3.0;
    cfg.minFitPoints = 12;
    cfg.fitIterations = 3;
    cfg.fitResidualMeters = 0.08;
    cfg.minFitInlierFraction = 0.80;
    cfg.maxFitRmsMeters = 0.05;
    cfg.maxAbsLateralSlope = 0.12;
    cfg.maxAbsHeightSlope = 0.15;
    cfg.maxExtensionMeters = 16.0;
    cfg.lateralToleranceMeters = 0.10;
    cfg.heightQuantiles = [0.05,0.95];
    cfg.heightPaddingMeters = 0.03;
    cfg.minTotalEnergy = 0.07;
    cfg.minBaseEnergy = 0.20;
    cfg.minHeightStepMeters = 0.025;
    cfg.minCenterEvidence = 0.90;
    cfg.maxRelativeHeightMeters = 0.25;
    cfg.maxSampleGapMeters = 0.90;
    cfg.minAddedPoints = 6;
    cfg.runSearchToleranceMeters = 0.30;
    cfg.runHeightPaddingMeters = 0.15;
    cfg.minRunPoints = 20;
    cfg.minRunLengthMeters = 2.0;
    cfg.maxRunStartDistanceMeters = 5.0;
    cfg.minRunHeightStepMeters = 0.05;
    cfg.maxRunLateralSlopeDifference = 0.05;
end
