function cfg = frameVoxelizationConfig()
% frameVoxelizationConfig: Geometry of the canonical fine voxel grid that
% every perception stage shares. One organized LiDAR frame is cropped to a
% square region of interest around the vehicle, the near-field ego region is
% excluded, and the remaining points are binned into 0.3 m x 0.3 m x 0.5 m
% voxels whose XY footprint also defines the ground and column rasters.
%
% Input:
%   none
%
% Output:
%   cfg: struct accepted by voxelizePointCloud with fields voxelSize
%       [1 x 3], roiLimits [xMin xMax yMin yMax], exclusionHalfSize,
%       minRange, maxRange, and statisticsMode
    cfg = struct();
    cfg.voxelSize = [0.3, 0.3, 0.5];
    cfg.roiLimits = [-50, 50, -50, 50];
    cfg.exclusionHalfSize = 3.0;
    cfg.minRange = 0;
    cfg.maxRange = inf;
    cfg.statisticsMode = "countOnly";
end
