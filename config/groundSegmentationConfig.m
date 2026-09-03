function cfg = groundSegmentationConfig()
% groundSegmentationConfig: Parameters of slope-grid ground segmentation.
% The segmentation rasterizes the robust low height of every XY cell,
% seeds the ground height from flat cells near the vehicle that agree with
% the sensor-height prior, propagates ground outward under a slope limit,
% promotes and bridges smooth flat components, fills holes, and finally
% labels points by their residual to the cell ground estimate.
%
% Input:
%   none
%
% Output:
%   cfg: struct consumed by segmentGround
    cfg = struct();

    % Sensor-height prior: the LiDAR sits about 1.44 m above the road
    cfg.groundSeedMaxZ = -1.44;
    cfg.slopeGridPriorHeight = -1.44;
    cfg.slopeGridPriorTolerance = 0.35;
    cfg.slopeGridPriorMinSeedCells = 20;

    % Raster resolution and per-cell robust low height
    cfg.slopeGridXYCellSize = [0.3, 0.3];
    cfg.slopeGridLowOutlierThreshold = 0.35;
    cfg.slopeGridMinCellPoints = 1;

    % Flat-cell gate: allowed vertical span grows with range
    cfg.slopeGridFlatSpanBase = 0.35;
    cfg.slopeGridFlatSpanRangeSlope = 0.003;
    cfg.slopeGridFlatSpanMax = 0.75;

    % Seed region near the vehicle and seed acceptance tolerance
    cfg.slopeGridSeedXLimits = [-8, 12];
    cfg.slopeGridSeedYAbsMax = 6;
    cfg.slopeGridSeedTolerance = 0.65;

    % Slope-limited near-to-far propagation with range-dependent noise
    cfg.slopeGridBaseHeightTolerance = 0.08;
    cfg.slopeGridMaxSlope = 0.25;
    cfg.slopeGridNoiseBase = 0.03;
    cfg.slopeGridNoiseRangeSlope = 0.002;
    cfg.slopeGridNoiseMax = 0.20;

    % Smooth flat-component promotion and distance-transform bridging
    cfg.slopeGridSmoothComponentPromotionEnabled = true;
    cfg.slopeGridSmoothComponentBridgeMinCells = 1;
    cfg.slopeGridSmoothComponentBridgeMaxCells = 300;
    cfg.slopeGridSmoothComponentBridgeMaxHeightRange = 0.45;
    cfg.slopeGridSmoothComponentBridgeRadiusCells = 8;
    cfg.slopeGridSmoothComponentBridgeMaxIterations = 4;
    cfg.slopeGridHoleFillWindowSize = 3;

    % Point labeling: residual tolerance grows with range, larger below ground
    cfg.slopeGridGroundToleranceBase = 0.32;
    cfg.slopeGridGroundToleranceRangeSlope = 0.004;
    cfg.slopeGridGroundToleranceMax = 0.72;
    cfg.slopeGridBelowTolerance = 1.50;
    cfg.slopeGridInterpToleranceScale = 0.60;
end
