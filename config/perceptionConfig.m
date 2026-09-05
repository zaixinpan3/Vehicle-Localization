function cfg = perceptionConfig()
% perceptionConfig: Shared pillar perception and offline point refinement.
% Default mode returns a sparse semantic Gaussian cloud for localization.
% Set executionMode="offline" for candidate-only fine masks used by mapping.
% The voxel config names XY resolution and sparse height-bin resolution;
% modern modes never allocate a dense 3D volume. Legacy knobs remain for
% historical reproduction and the retained geometric feature rules.
    cfg = struct();
    cfg.executionMode = "coarseProbabilityCloud";
    cfg.voxel = frameVoxelizationConfig();
    cfg.groundSegmentation = groundSegmentationConfig();
    cfg.groundFeatures = groundFeatureConfig();
    cfg.offGroundFeatures = offGroundFeatureConfig();
    cfg.coarseProbabilityCloud = coarseSemanticProbabilityCloudConfig();
    cfg.fine = finePerceptionConfig();
end
