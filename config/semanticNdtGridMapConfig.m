function cfg = semanticNdtGridMapConfig()
% semanticNdtGridMapConfig: Fixed 2D grid and covariance controls for
% projecting the coarse 3D semantic voxel tags of one frame into semantic
% NDT Gaussian cells in the sensor-local XY frame.
%
% Input:
%   none
%
% Output:
%   cfg: struct consumed by buildSemanticNdtGridMap
    voxelCfg = pillarGridConfig();
    cfg = struct();
    cfg.xMin = voxelCfg.roiLimits(1);
    cfg.xMax = voxelCfg.roiLimits(2);
    cfg.yMin = voxelCfg.roiLimits(3);
    cfg.yMax = voxelCfg.roiLimits(4);
    cfg.resolution = 1.0;
    cfg.semanticNames = ["roadSurface", "curbCandidate", "roadBoundary", ...
        "roadMarkingCandidate", "facadeCandidate", "poleCandidate", ...
        "trafficSignCandidate"];
    cfg.minPointsPerGaussian = 3;
    cfg.minCovarianceEigenvalue = 1.0e-2;
    cfg.maxCovarianceEigenvalue = 4.0;
    cfg.regularizationVariance = 1.0e-4;
    cfg.storePointAssignments = false;
    cfg.coordinateFrame = "sensorLocalXY";
end
