function cfg = coarseSemanticProbabilityCloudConfig()
% coarseSemanticProbabilityCloudConfig: Parameters for the fast, strictly
% voxel-classified 2D semantic probability cloud. Feature decisions stay on
% the ground-cell and vertical-column rasters; selected cell geometry is
% accumulated into regularized 2D NDT components for distribution-to-
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
    cfg.semanticNames = ["curb", "roadMarking", "pole"];

    % Cell-level selectors chosen on Mississippi development frames and
    % locked before evaluation on the validation frames.
    cfg.curbMinimumTotalEnergy = 0.03;
    cfg.curbDisabledRefinementStages = [ ...
        "sameSideDuplicateSuppressionEnabled", ...
        "roadFacingShoulderRecoveryEnabled", ...
        "sameSideOutlierSuppressionEnabled"];
    cfg.poleMinimumPointScore = 0.60;
    cfg.poleMaximumLineScore = 0.75;

    % Probability transfers and NDT covariance safeguards.
    cfg.minimumSemanticProbability = 0.50;
    cfg.occupancySaturationPointCount = 6;
    cfg.minimumPointsPerComponent = 1;
    cfg.minCovarianceEigenvalue = 1.0e-2;
    cfg.maxCovarianceEigenvalue = 4.0;
    cfg.regularizationVariance = 1.0e-4;
    cfg.storeDiagnostics = false;
end
