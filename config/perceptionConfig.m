function cfg = perceptionConfig(dataset)
% perceptionConfig: Shared pillar perception and offline point refinement.
% cfg.featureNames selects the semantic channels for this invocation.
% Mississippi defaults to every channel except facade; Downtown selects all.
% Override featureNames with any subset, or strings(1,0) for no channels.
% Default mode returns a sparse semantic Gaussian cloud for localization.
% Set executionMode="offline" for candidate-only fine masks used by mapping.
% The voxel config names XY resolution and sparse height-bin resolution;
% modern modes never allocate a dense 3D volume. Legacy knobs remain for
% historical reproduction and the retained geometric feature rules.
    if nargin < 1, dataset = "Mississippi"; end
    dataset = lower(string(dataset));
    assert(isscalar(dataset) && any(dataset == ["mississippi", "missisipi", "downtown"]), ...
        "Unknown perception dataset profile.");
    cfg = struct();
    cfg.executionMode = "coarseProbabilityCloud";
    cfg.executionBackend = "auto"; % Native kernels when built; otherwise MATLAB.
    cfg.compactGroundRaster = true; % Trim empty margins, retaining all ground and its halo.
    cfg.frameCalibration = lidarFrameCalibrationConfig();
    cfg.voxel = frameVoxelizationConfig();
    cfg.groundSegmentation = groundSegmentationConfig();
    cfg.groundFeatures = groundFeatureConfig();
    cfg.offGroundFeatures = rmfield(offGroundFeatureConfig(),"facadeDetectionEnabled");
    cloudCfg = coarseSemanticProbabilityCloudConfig();
    cfg.featureNames = cloudCfg.semanticNames;
    if dataset ~= "downtown", cfg.featureNames(cfg.featureNames=="facade") = []; end
    cfg.coarseProbabilityCloud = rmfield(cloudCfg,"semanticNames");
    cfg.fine = finePerceptionConfig();
end
