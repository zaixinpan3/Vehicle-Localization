function [probabilityCloudMap, featureData] = buildFeatureMap(dataRoot, cfg)
% buildFeatureMap: Offline mapping entry point. Frames of the mapping drive
% are perceived one by one, their selected curb, facade, pole, and traffic-sign
% observations are registered into the global frame with the matched
% GNSS/INS poses, and the registered observations are converted into the
% sliding-window semantic temporal-stability probability-cloud map that the
% online localization queries. The map artifact is saved under dataRoot when
% cfg.mapOutputPath is nonempty.
%
% Input:
%   dataRoot: folder holding the dataset files referenced by cfg
%   cfg: optional struct from featureMapBuildConfig
%
% Output:
%   probabilityCloudMap: saved-map artifact returned by buildSlidingWindowMap
%   featureData: registered per-frame feature observations
    if nargin < 2 || isempty(cfg)
        cfg = featureMapBuildConfig();
    end
    cfg.featureNames = validatePerceptionFeatureNames(cfg.featureNames);
    matPath = fullfile(dataRoot, cfg.pointCloudMatPath);
    poseMatchCsvPath = fullfile(dataRoot, cfg.poseMatchCsvPath);
    assert(isfile(matPath), "Point-cloud MAT file not found: %s", matPath);
    assert(isfile(poseMatchCsvPath), "Pose match CSV not found: %s. Build it with matchFramePoses.", poseMatchCsvPath);

    frameIndices = double(cfg.frameIndices(:).');
    if isempty(frameIndices)
        [~, numFrames] = loadPointCloudFrame(matPath, 1);
        frameIndices = 1:numFrames;
    end
    framePoseTable = readFramePoseTable(poseMatchCsvPath, frameIndices);

    perceptionCfg = perceptionConfig();
    if isfield(cfg,'frameCalibration'), perceptionCfg.frameCalibration=validateLidarFrameCalibration(cfg.frameCalibration); end
    perceptionCfg.featureNames = cfg.featureNames;
    featureData = collectFeatureObservations(matPath, frameIndices, framePoseTable, perceptionCfg, cfg);
    probabilityCloudMap = buildSlidingWindowMap(featureData, cfg);
    probabilityCloudMap.sourceMatPath = string(matPath);
    probabilityCloudMap.poseMatchCsvPath = string(poseMatchCsvPath);

    if strlength(string(cfg.mapOutputPath)) > 0
        mapOutputPath = fullfile(dataRoot, cfg.mapOutputPath);
        mapOutputDir = fileparts(mapOutputPath);
        if strlength(string(mapOutputDir)) > 0 && ~isfolder(mapOutputDir)
            mkdir(mapOutputDir);
        end
        save(mapOutputPath, "probabilityCloudMap", "-v7.3");
        mappingSupport.logStep(cfg, "map.save", "path=%s | windows=%d | frames=%s", mapOutputPath, ...
            numel(probabilityCloudMap.batchMaps), char(mappingSupport.formatFrameIndexSet(frameIndices)));
    end
end
