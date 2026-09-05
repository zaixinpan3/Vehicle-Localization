function cfg = coarseSemanticProbabilityCloudConfig()
% coarseSemanticProbabilityCloudConfig: Parameters for the fast, strictly
% pillar-classified 2D semantic probability cloud. Feature decisions stay on
% the ground-cell and vertical-column rasters; selected cell geometry is
% accumulated into regularized XYZ components with an XY marginal for distribution-to-
% distribution matching.
%
% Input:
%   none
%
% Output:
%   cfg: struct consumed by perceiveCoarseProbabilityCloud and
%       buildCoarseSemanticProbabilityCloud
    voxelCfg = frameVoxelizationConfig();
    cfg = struct();
    cfg.xMin = voxelCfg.roiLimits(1);
    cfg.xMax = voxelCfg.roiLimits(2);
    cfg.yMin = voxelCfg.roiLimits(3);
    cfg.yMax = voxelCfg.roiLimits(4);
    % Three source cells per output cell preserve exact 0.3 m boundaries.
    cfg.resolution = 0.9;
    cfg.coordinateFrame = "sensorLocalXY";
    % Optional known IMU tilt: transform sufficient statistics before BEV
    % projection, while classifying in the original vehicle XY pillars.
    cfg.projectionRotation = eye(3);
    cfg.semanticNames = ["curb", "roadMarking", "pole"];

    % Pillar selectors retain the tested geometric evidence rules.
    % The accepted cell mask already applies the curb evidence tests.
    cfg.curbMinimumTotalEnergy = 0;
    cfg.curbDisabledRefinementStages = strings(0, 1);
    % Apply the retained shape criterion to complete candidate footprints,
    % not separately to each pillar of a boundary-crossing pole.
    cfg.poleMinimumFootprintScore = 0.70;

    % Probability transfers and NDT covariance safeguards.
    cfg.minimumSemanticProbability = 0.50;
    cfg.occupancySaturationPointCount = 6;
    cfg.minimumPointsPerComponent = 1;
    cfg.minCovarianceEigenvalue = 1.0e-2;
    cfg.maxCovarianceEigenvalue = 4.0;
    cfg.regularizationVariance = 1.0e-4;
    cfg.minimumConditionalHeightVariance = 1.0e-4;
    cfg.storeDiagnostics = false;
end
