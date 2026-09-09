function cfg = fineVoxelizationConfig()
% fineVoxelizationConfig: Sparse spatial index for offline fine detection.
    cfg = pillarGridConfig();
    cfg.voxelSize = [cfg.voxelSize, 0.5];
end
