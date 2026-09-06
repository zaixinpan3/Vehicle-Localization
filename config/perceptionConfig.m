function cfg = perceptionConfig(dataset)
% perceptionConfig: Shared pillar perception and offline point refinement.
% Optional dataset="Downtown" enables facades; the default Mississippi profile
% disables facades. Traffic signs are available in both profiles.
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
    cfg.offGroundFeatures = offGroundFeatureConfig();
    cfg.coarseProbabilityCloud = coarseSemanticProbabilityCloudConfig();
    cfg.fine = finePerceptionConfig();
    cfg.offGroundFeatures.facadeDetectionEnabled = dataset == "downtown";
end
