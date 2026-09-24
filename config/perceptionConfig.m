function cfg = perceptionConfig(dataset, executionMode)
% perceptionConfig: Shared pillar perception and offline point refinement.
% executionMode selects the product and its lattice: coarseProbabilityCloud
% (default) returns a sparse semantic Gaussian cloud for localization from
% 0.6 m pillars; offline returns independent detailed point masks used by
% mapping from 0.3 m pillars. Every pillar-stage parameter is derived for the
% selected lattice, so change the mode here rather than after construction.
% cfg.featureNames selects the semantic channels for this invocation.
% Mississippi defaults to every channel except facade; Downtown selects all.
% Override featureNames with any subset, or strings(1,0) for no channels.
% voxel.voxelSize contains exactly two XY spacings. Pillars retain full XYZ statistics.
    if nargin < 1 || isempty(dataset), dataset = "Mississippi"; end
    if nargin < 2 || isempty(executionMode), executionMode = "coarseProbabilityCloud"; end
    dataset = lower(string(dataset));
    assert(isscalar(dataset) && any(dataset == ["mississippi", "missisipi", "downtown"]), ...
        "Unknown perception dataset profile.");
    cfg = struct();
    cfg.executionMode = string(executionMode);
    cfg.executionBackend = "auto"; % Native kernels when built; otherwise MATLAB.
    cfg.compactGroundRaster = true; % Trim empty margins, retaining all ground and its halo.
    cfg.frameCalibration = lidarFrameCalibrationConfig(dataset);
    cfg.voxel = pillarGridConfig(cfg.executionMode);
    spacing = cfg.voxel.voxelSize(1);
    cfg.groundSegmentation = groundSegmentationConfig(spacing);
    cfg.groundFeatures = groundFeatureConfig(spacing);
    cfg.offGroundFeatures = structuralPillarConfig(spacing);
    cloudCfg = coarseSemanticProbabilityCloudConfig(cfg.voxel);
    cfg.featureNames = cloudCfg.semanticNames;
    if dataset ~= "downtown", cfg.featureNames(cfg.featureNames=="facade") = []; end
    cfg.coarseProbabilityCloud = rmfield(cloudCfg,"semanticNames");
    cfg.fine = finePerceptionConfig();
end
