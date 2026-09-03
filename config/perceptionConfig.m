function cfg = perceptionConfig()
% perceptionConfig: Aggregate configuration of the perception module in
% pipeline order: frame voxelization, ground segmentation, the ground
% feature branch (curbs, road surface, road markings), and the non-ground
% feature branch (poles, facades, traffic signs).
%
% Input:
%   none
%
% Output:
%   cfg: struct with fields voxel, groundSegmentation, groundFeatures, and
%       offGroundFeatures, each produced by the matching stage config
    cfg = struct();
    cfg.voxel = frameVoxelizationConfig();
    cfg.groundSegmentation = groundSegmentationConfig();
    cfg.groundFeatures = groundFeatureConfig();
    cfg.offGroundFeatures = offGroundFeatureConfig();
end
