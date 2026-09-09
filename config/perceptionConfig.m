function cfg = perceptionConfig(dataset)
% perceptionConfig: Shared pillar perception and offline point refinement.
% cfg.featureNames selects the semantic channels for this invocation.
% Mississippi defaults to every channel except facade; Downtown selects all.
% Override featureNames with any subset, or strings(1,0) for no channels.
% Default mode returns a sparse semantic Gaussian cloud for localization.
% Set executionMode="offline" for independent detailed point masks used by mapping.
% Modern modes use voxel.voxelSize(1:2) as XY pillar spacing and ignore
% its legacy Z spacing. Every pillar retains full XYZ distribution statistics.
    if nargin < 1, dataset = "Mississippi"; end
    dataset = lower(string(dataset));
    assert(isscalar(dataset) && any(dataset == ["mississippi", "missisipi", "downtown"]), ...
        "Unknown perception dataset profile.");
    cfg = struct();
    cfg.executionMode = "coarseProbabilityCloud";
    cfg.executionBackend = "auto"; % Native kernels when built; otherwise MATLAB.
    cfg.compactGroundRaster = true; % Trim empty margins, retaining all ground and its halo.
    cfg.frameCalibration = lidarFrameCalibrationConfig();
    cfg.voxel = rmfield(frameVoxelizationConfig(),'statisticsMode');
    cfg.voxel.voxelSize=cfg.voxel.voxelSize(1:2);
    cfg.groundSegmentation = groundSegmentationConfig();
    cfg.groundFeatures = groundFeatureConfig();
    cfg.offGroundFeatures = structuralPillarConfig();
    cloudCfg = coarseSemanticProbabilityCloudConfig();
    cfg.featureNames = cloudCfg.semanticNames;
    if dataset ~= "downtown", cfg.featureNames(cfg.featureNames=="facade") = []; end
    cfg.coarseProbabilityCloud = rmfield(cloudCfg,"semanticNames");
    cfg.fine = finePerceptionConfig();
end
