function cfg = perceptionConfig(dataset, executionMode)
% perceptionConfig: Shared pillar perception and offline point refinement.
% executionMode selects the product and its lattice: coarseProbabilityCloud
% (default) returns a sparse semantic Gaussian cloud for localization from
% 0.6 m pillars (0.3 m for Downtown); offline returns detailed point masks used
% by mapping from 0.3 m pillars. Every pillar-stage parameter is derived for the
% selected lattice, so change the mode here rather than after construction.
% cfg.featureNames selects the semantic channels for this invocation.
% Mississippi defaults to every channel except facade; Downtown selects all.
% Downtown uses the joint reviewV12 curb, wall, pole and sign support rules,
% with no external-project or recorded-label runtime dependency.
% Carla (CARLA Town10HD semantic LiDAR, no intensity) selects curb, pole and
% facade with the CARLA-tuned values of carlaPerceptionProfile.
% Override featureNames with any subset, or strings(1,0) for no channels.
% voxel.voxelSize contains exactly two XY spacings. Pillars retain full XYZ statistics.
    if nargin < 1 || isempty(dataset), dataset = "Mississippi"; end
    if nargin < 2 || isempty(executionMode), executionMode = "coarseProbabilityCloud"; end
    dataset = lower(string(dataset));
    assert(isscalar(dataset) && any(dataset == ["mississippi", "missisipi", "downtown", "carla"]), ...
        "Unknown perception dataset profile.");
    cfg = struct();
    cfg.executionMode = string(executionMode);
    cfg.executionBackend = "auto"; % Native kernels when built; otherwise MATLAB.
    cfg.compactGroundRaster = true; % Trim empty margins, retaining all ground and its halo.
    cfg.frameCalibration = lidarFrameCalibrationConfig(dataset);
    cfg.voxel = pillarGridConfig(cfg.executionMode, dataset);
    spacing = cfg.voxel.voxelSize(1);
    cfg.groundSegmentation = groundSegmentationConfig(spacing);
    cfg.groundFeatures = groundFeatureConfig(spacing);
    cfg.offGroundFeatures = structuralPillarConfig(spacing);
    if abs(spacing-.6)<1e-12
        cfg.offGroundFeatures.pole.distributionValidation=pillarPoleDistributionConfig(dataset);
    end
    cloudCfg = coarseSemanticProbabilityCloudConfig(cfg.voxel);
    cfg.featureNames = cloudCfg.semanticNames;
    if ~any(dataset == ["downtown", "carla"]), cfg.featureNames(cfg.featureNames=="facade") = []; end
    if dataset == "carla", cfg.featureNames(cfg.featureNames=="trafficSign") = []; end
    cfg.coarseProbabilityCloud = rmfield(cloudCfg,"semanticNames");
    cfg.fine = finePerceptionConfig();
    cfg.semanticPrecision = semanticPillarPrecisionConfig(spacing,dataset);
    if dataset == "downtown"
        cfg.facadeSurface = facadeSurfaceConfig(spacing);
        cfg.downtownCurb = downtownCurbConfig(spacing);
        cfg.downtownStructure = downtownStructuralConfig(spacing);
        cfg.downtownCandidates = downtownCandidateConfig(spacing);
        cfg.offGroundFeatures.trafficSignIntensityThreshold = 1600;
        cfg.facadeSurface.trafficSignIntensityThreshold = 1600;
        if spacing<.6
            cfg.offGroundFeatures.pillarStatistics=struct('facadeMinimumPoints',1,'facadeMinimumHeight',0.10);
        end
        for field = string(fieldnames(cfg.downtownCurb.groundOverrides)).'
            cfg.groundFeatures.curb.(field) = cfg.downtownCurb.groundOverrides.(field);
        end
        cfg.semanticPrecision.enabled = false;
        cfg.semanticPrecision.classes = strings(1,0);
    end
    cfg.curbBoundary=curbBoundaryGeometryConfig();
    cfg.curbBoundary.enabled=dataset~="downtown" && cfg.executionMode=="coarseProbabilityCloud";
    if dataset == "carla", cfg = carlaPerceptionProfile(cfg); end
end
