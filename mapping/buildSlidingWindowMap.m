function probabilityCloudMap = buildSlidingWindowMap(featureData, cfg)
% buildSlidingWindowMap: Build the offline prior map as a sequence of
% semantic temporal-stability Gaussian support maps over overlapping frame
% windows. Each window collects the registered feature points of its frames
% into per-class BEV map inputs, caps the temporal diversity saturation by
% the window length, and fits one temporal-stability GMM map per window.
% Neighboring windows overlap so that online queries can fuse their
% responses by max-envelope without hard support discontinuities.
%
% Input:
%   featureData: struct returned by collectFeatureObservations
%   cfg: struct from featureMapBuildConfig
%
% Output:
%   probabilityCloudMap: saved-map artifact with mapType, queryFunction,
%       queryBatchFusion, frame coverage, window schedule, featureNames,
%       batchMaps (one gmmMap per window), layerSummaryTable,
%       frameSummaryTable, and config
    frameIndices = double(featureData.frameIndices(:).');
    batchFrameWindows = resolveBatchFrameWindows(frameIndices, cfg);
    batchMaps = repmat(struct("batchIndex", 0, "frameIndices", zeros(1, 0), "gmmMap", []), numel(batchFrameWindows), 1);

    for batchIdx = 1:numel(batchFrameWindows)
        batchFrameIndices = double(batchFrameWindows{batchIdx}(:).');
        batchCfg = cfg;
        batchCfg.timestampMaxBins = min(double(cfg.timestampMaxBins), double(numel(batchFrameIndices)));
        batchFeatureData = subsetFeatureDataByFrames(featureData, batchFrameIndices);
        mappingSupport.logStep(cfg, "batch.start", "idx=%d/%d | frames=%s", batchIdx, numel(batchFrameWindows), char(mappingSupport.formatFrameIndexSet(batchFrameIndices)));
        mapInput = assembleMapInput(batchFeatureData, batchCfg);
        gmmMap = [];
        if ~isempty(mapInput.buildFeatureNames)
            gmmCfg = resolveWindowMapConfig(mapInput.buildFeatureNames, batchCfg);
            mappingSupport.logStep(cfg, "batch.gmm.build", "idx=%d/%d | sourcePoints=%d | classes=%s", batchIdx, numel(batchFrameWindows), ...
                size(mapInput.points, 1), strjoin(mapInput.buildFeatureNames, ", "));
            gmmMap = buildTemporalStabilityGmmMap(mapInput.pointsXYZ, mapInput.labels, mapInput.timestamps, gmmCfg);
            if isfield(featureData,'frameCalibration'), gmmMap.frameCalibration=featureData.frameCalibration; end
            logGmmMapSummary(cfg, gmmMap);
        else
            mappingSupport.logStep(cfg, "batch.gmm.skip", "idx=%d/%d | no class met map input thresholds", batchIdx, numel(batchFrameWindows));
        end
        batchMaps(batchIdx).batchIndex = batchIdx;
        batchMaps(batchIdx).frameIndices = batchFrameIndices;
        batchMaps(batchIdx).gmmMap = gmmMap;
    end

    probabilityCloudMap = struct();
    probabilityCloudMap.mapType = "missisipiSlidingWindowIntegratedTemporalGMMProbabilityCloudMap";
    probabilityCloudMap.queryFunction = "queryTemporalStabilityGmmMap";
    probabilityCloudMap.queryBatchFusion = "maxEnvelope";
    probabilityCloudMap.spatialDimension = 3;
    probabilityCloudMap.heightModel = "conditionalGaussianGivenXY";
    probabilityCloudMap.createdAt = string(datetime("now"));
    probabilityCloudMap.sourceFrameCount = round(double(featureData.numFrames));
    probabilityCloudMap.frameIndices = frameIndices;
    probabilityCloudMap.batchFrameWindows = batchFrameWindows;
    probabilityCloudMap.batchFrameCount = round(double(cfg.batchFrameCount));
    probabilityCloudMap.batchFrameStride = round(double(cfg.batchFrameStride));
    probabilityCloudMap.transitionOverlapFrameCount = max(0, probabilityCloudMap.batchFrameCount - probabilityCloudMap.batchFrameStride);
    probabilityCloudMap.featureNames = string(cfg.featureNames(:));
    probabilityCloudMap.batchMaps = batchMaps;
    probabilityCloudMap.layerSummaryTable = buildLayerSummaryTable(batchMaps);
    probabilityCloudMap.frameSummaryTable = featureData.frameSummaryTable;
    probabilityCloudMap.config = cfg;
    if isfield(featureData,'frameCalibration'), probabilityCloudMap.frameCalibration=featureData.frameCalibration; end
end

function gmmCfg = resolveWindowMapConfig(buildFeatureNames, batchCfg)
% resolveWindowMapConfig: Configure the temporal-stability GMM builder for
% one frame window: the retained semantic classes, the logging switch, and
% the temporal diversity saturation capped by the window frame count.
    gmmCfg = temporalStabilityMapConfig();
    gmmCfg.classes = string(buildFeatureNames(:));
    gmmCfg.logEnabled = mappingSupport.isLogEnabled(batchCfg);
    gmmCfg.defaultParams.timestampMaxBins = double(batchCfg.timestampMaxBins);
    gmmCfg.defaultParams.minTimestampBins = round(double(batchCfg.minMapTimestampBinsPerClass));
    for idx = 1:numel(gmmCfg.classParams)
        gmmCfg.classParams(idx).timestampMaxBins = double(batchCfg.timestampMaxBins);
        gmmCfg.classParams(idx).minTimestampBins = round(double(batchCfg.minMapTimestampBinsPerClass));
    end
end

function batchFrameWindows = resolveBatchFrameWindows(frameIndices, cfg)
% resolveBatchFrameWindows: Resolve overlapping sliding-window frame
% batches over the covered frame range. Window starts advance by
% batchFrameStride, and the final window is clipped at the requested last frame
% so every neighboring batch pair keeps the configured transition overlap.
    frameIndices = double(frameIndices(:).');
    batchFrameCount = round(double(cfg.batchFrameCount));
    batchFrameStride = round(double(cfg.batchFrameStride));
    assert(~isempty(frameIndices) && all(diff(frameIndices) == 1), ...
        "Sliding-window batch construction requires contiguous frame indices.");
    assert(isscalar(batchFrameCount) && isfinite(batchFrameCount) && batchFrameCount >= 1, ...
        "cfg.batchFrameCount must be a positive finite integer.");
    assert(isscalar(batchFrameStride) && isfinite(batchFrameStride) && batchFrameStride >= 1, ...
        "cfg.batchFrameStride must be a positive finite integer.");
    if numel(frameIndices) <= batchFrameCount
        batchFrameWindows = {frameIndices};
        return;
    end
    firstFrame = frameIndices(1);
    lastFrame = frameIndices(end);
    startFrames = firstFrame:batchFrameStride:lastFrame;
    batchFrameWindows = cell(numel(startFrames), 1);
    for batchIdx = 1:numel(startFrames)
        batchFrameWindows{batchIdx} = startFrames(batchIdx):min(lastFrame, startFrames(batchIdx) + batchFrameCount - 1);
    end
end

function batchFeatureData = subsetFeatureDataByFrames(featureData, batchFrameIndices)
% subsetFeatureDataByFrames: Select a contiguous frame subset from the
% already extracted per-feature/per-frame global point clouds while preserving
% feature names, display names, frame poses, and count matrices.
    allFrameIndices = double(featureData.frameIndices(:).');
    [isMember, keepIdx] = ismember(double(batchFrameIndices(:).'), allFrameIndices);
    assert(all(isMember), "Batch frame window contains frames missing from featureData.");
    batchFeatureData = featureData;
    batchFeatureData.frameIndices = allFrameIndices(keepIdx);
    batchFeatureData.pointsByFeatureFrame = featureData.pointsByFeatureFrame(:, keepIdx);
    batchFeatureData.counts = featureData.counts(keepIdx, :);
    if isfield(featureData, "framePoseTable")
        batchFeatureData.framePoseTable = featureData.framePoseTable(keepIdx, :);
    end
end

function layerSummaryTable = buildLayerSummaryTable(batchMaps)
% buildLayerSummaryTable: Build a compact table of
% per-batch/per-class GMM layer counts and support amplitude summaries for
% quick inspection of a saved probability-cloud map.
    rowCount = 0;
    for batchIdx = 1:numel(batchMaps)
        if ~isempty(batchMaps(batchIdx).gmmMap) && isfield(batchMaps(batchIdx).gmmMap, "layers")
            rowCount = rowCount + numel(batchMaps(batchIdx).gmmMap.layers);
        end
    end

    batchIndexValues = zeros(rowCount, 1);
    firstFrameValues = zeros(rowCount, 1);
    lastFrameValues = zeros(rowCount, 1);
    classLabels = strings(rowCount, 1);
    pointCountValues = zeros(rowCount, 1);
    componentCountValues = zeros(rowCount, 1);
    emIterationCountValues = zeros(rowCount, 1);
    emConvergedValues = false(rowCount, 1);
    prunedComponentCountValues = zeros(rowCount, 1);
    supportMinValues = nan(rowCount, 1);
    supportMedianValues = nan(rowCount, 1);
    supportMaxValues = nan(rowCount, 1);

    rowIdx = 0;
    for batchIdx = 1:numel(batchMaps)
        if isempty(batchMaps(batchIdx).gmmMap) || ~isfield(batchMaps(batchIdx).gmmMap, "layers")
            continue;
        end
        frameIndices = double(batchMaps(batchIdx).frameIndices(:));
        for layerIdx = 1:numel(batchMaps(batchIdx).gmmMap.layers)
            rowIdx = rowIdx + 1;
            layer = batchMaps(batchIdx).gmmMap.layers(layerIdx);
            [supportMin, supportMedian, supportMax] = mappingSupport.finiteSummary(layer.componentSupportAmplitudes);
            batchIndexValues(rowIdx) = batchMaps(batchIdx).batchIndex;
            firstFrameValues(rowIdx) = frameIndices(1);
            lastFrameValues(rowIdx) = frameIndices(end);
            classLabels(rowIdx) = string(layer.classLabel);
            pointCountValues(rowIdx) = layer.pointCount;
            componentCountValues(rowIdx) = numel(layer.components);
            emIterationCountValues(rowIdx) = layer.emIterationCount;
            emConvergedValues(rowIdx) = logical(layer.emConverged);
            prunedComponentCountValues(rowIdx) = layer.integratedSupportPrunedComponentCount;
            supportMinValues(rowIdx) = supportMin;
            supportMedianValues(rowIdx) = supportMedian;
            supportMaxValues(rowIdx) = supportMax;
        end
    end

    layerSummaryTable = table(batchIndexValues, firstFrameValues, lastFrameValues, classLabels, pointCountValues, componentCountValues, ...
        emIterationCountValues, emConvergedValues, prunedComponentCountValues, supportMinValues, supportMedianValues, supportMaxValues, ...
        'VariableNames', {'batchIndex', 'firstFrame', 'lastFrame', 'classLabel', 'pointCount', 'componentCount', ...
        'emIterationCount', 'emConverged', 'integratedSupportPrunedComponentCount', 'supportMin', 'supportMedian', 'supportMax'});
end

function logGmmMapSummary(cfg, gmmMap)
% logGmmMapSummary: Print one map-level and one layer-level diagnostic
% summary after semantic temporal-stability GMM map construction completes.
    if isempty(gmmMap)
        mappingSupport.logStep(cfg, "gmm.summary", "map is empty");
        return;
    end
    mappingSupport.logStep(cfg, "gmm.summary", "mapType=%s | classes=%d | sourcePointCount=%d", ...
        char(string(gmmMap.mapType)), numel(gmmMap.layers), gmmMap.sourcePointCount);
    for layerIdx = 1:numel(gmmMap.layers)
        layer = gmmMap.layers(layerIdx);
        [minAmp, medianAmp, maxAmp] = mappingSupport.finiteSummary(layer.componentSupportAmplitudes);
        [minPatch, medianPatch, maxPatch] = mappingSupport.finiteSummary(layer.componentPatchPointCounts);
        mappingSupport.logStep(cfg, "gmm.layer", "class=%s | points=%d | components=%d | emIter=%d | converged=%d | pruned=%d | patchPoints[min/med/max]=%.4g/%.4g/%.4g | support[min/med/max]=%.4g/%.4g/%.4g", ...
            char(string(layer.classLabel)), layer.pointCount, numel(layer.components), layer.emIterationCount, logical(layer.emConverged), layer.integratedSupportPrunedComponentCount, ...
            minPatch, medianPatch, maxPatch, minAmp, medianAmp, maxAmp);
    end
end

function mapInput = assembleMapInput(featureData, cfg)
% assembleMapInput: Assemble XYZ observations and compatible BEV inputs from
% per-feature/per-frame semantic point clouds, excluding facade by
% construction and only passing classes with enough points across at least
% two frame bins to the fail-fast GMM builder.
    featureNames = string(featureData.featureNames(:));
    frameIndices = double(featureData.frameIndices(:).');
    numFeatures = numel(featureNames);
    pointsByFeature = cell(numFeatures, 1);
    timestampsByFeature = cell(numFeatures, 1);
    buildFeatureMask = false(numFeatures, 1);
    rawPointCounts = zeros(numFeatures, 1);
    buildPointCounts = zeros(numFeatures, 1);
    timestampBinCounts = zeros(numFeatures, 1);

    mappingSupport.logStep(cfg, "mapinput.start", "features=%d | maxPointsPerClass=%g | minPoints=%d | minTimestampBins=%d", ...
        numFeatures, double(cfg.maxMapPointsPerClass), round(double(cfg.minMapPointsPerClass)), round(double(cfg.minMapTimestampBinsPerClass)));
    for featureIdx = 1:numFeatures
        [featurePoints, featureTimestamps] = concatenateFeatureFrames(featureData.pointsByFeatureFrame(featureIdx, :), frameIndices);
        rawPointCounts(featureIdx) = size(featurePoints, 1);
        [featurePoints, featureTimestamps] = reduceMapFeaturePoints(featurePoints, featureTimestamps, cfg);
        pointsByFeature{featureIdx} = featurePoints;
        timestampsByFeature{featureIdx} = featureTimestamps;
        buildPointCounts(featureIdx) = size(featurePoints, 1);
        timestampBinCounts(featureIdx) = numel(unique(featureTimestamps));
        if buildPointCounts(featureIdx) >= cfg.minMapPointsPerClass && timestampBinCounts(featureIdx) >= cfg.minMapTimestampBinsPerClass
            buildFeatureMask(featureIdx) = true;
        end
        mappingSupport.logStep(cfg, "mapinput.feature", "feature=%s | rawPoints=%d | buildPoints=%d | timestampBins=%d | build=%d", ...
            char(featureNames(featureIdx)), rawPointCounts(featureIdx), buildPointCounts(featureIdx), timestampBinCounts(featureIdx), buildFeatureMask(featureIdx));
    end

    buildFeatureNames = featureNames(buildFeatureMask);
    totalBuildPointCount = sum(buildPointCounts(buildFeatureMask));
    points = zeros(totalBuildPointCount, 2);
    pointsXYZ = zeros(totalBuildPointCount, 3);
    labels = strings(totalBuildPointCount, 1);
    timestamps = zeros(totalBuildPointCount, 1);
    writeIdx = 1;
    buildFeatureIdx = find(buildFeatureMask).';
    for featureIdx = buildFeatureIdx
        featurePoints = pointsByFeature{featureIdx};
        featureTimestamps = timestampsByFeature{featureIdx};
        pointCount = size(featurePoints, 1);
        rangeIdx = writeIdx:(writeIdx + pointCount - 1);
        points(rangeIdx, :) = featurePoints(:, 1:2);
        pointsXYZ(rangeIdx, :) = featurePoints(:, 1:3);
        labels(rangeIdx, 1) = repmat(featureNames(featureIdx), pointCount, 1);
        timestamps(rangeIdx, 1) = featureTimestamps(:);
        writeIdx = writeIdx + pointCount;
    end

    mapInput = struct();
    mapInput.points = points;
    mapInput.pointsXYZ = pointsXYZ;
    mapInput.labels = labels;
    mapInput.timestamps = timestamps;
    mapInput.allFeatureNames = featureNames;
    mapInput.buildFeatureNames = buildFeatureNames;
    mapInput.pointsByFeature = pointsByFeature;
    mapInput.timestampsByFeature = timestampsByFeature;
    mapInput.rawPointCounts = rawPointCounts;
    mapInput.buildPointCounts = buildPointCounts;
    mapInput.timestampBinCounts = timestampBinCounts;
    mappingSupport.logStep(cfg, "mapinput.done", "totalBuildPoints=%d | buildClasses=%s", totalBuildPointCount, strjoin(buildFeatureNames, ", "));
end

function [featurePoints, featureTimestamps] = concatenateFeatureFrames(pointsByFrame, frameIndices)
% concatenateFeatureFrames: Concatenate one feature's per-frame point
% clouds and generate one timestamp value per retained point using the
% original frame index.
    totalCount = 0;
    for frameListIdx = 1:numel(pointsByFrame)
        totalCount = totalCount + size(pointsByFrame{frameListIdx}, 1);
    end
    featurePoints = zeros(totalCount, 3);
    featureTimestamps = zeros(totalCount, 1);
    writeIdx = 1;
    for frameListIdx = 1:numel(pointsByFrame)
        framePoints = double(pointsByFrame{frameListIdx});
        pointCount = size(framePoints, 1);
        if pointCount > 0
            rangeIdx = writeIdx:(writeIdx + pointCount - 1);
            featurePoints(rangeIdx, :) = framePoints;
            featureTimestamps(rangeIdx, 1) = frameIndices(frameListIdx);
            writeIdx = writeIdx + pointCount;
        end
    end
end

function [featurePoints, featureTimestamps] = reduceMapFeaturePoints(featurePoints, featureTimestamps, cfg)
% reduceMapFeaturePoints: Apply deterministic finite-point filtering
% and optional point-count capping before map construction.
    if isempty(featurePoints)
        featurePoints = zeros(0, 3);
        featureTimestamps = zeros(0, 1);
        return;
    end
    validMask = all(isfinite(featurePoints(:, 1:3)), 2) & isfinite(featureTimestamps(:));
    featurePoints = featurePoints(validMask, :);
    featureTimestamps = featureTimestamps(validMask);
    maxPoints = double(cfg.maxMapPointsPerClass);
    if isfinite(maxPoints) && size(featurePoints, 1) > maxPoints
        maxPoints = max(1, round(maxPoints));
        keepIdx = unique(round(linspace(1, size(featurePoints, 1), maxPoints))).';
        featurePoints = featurePoints(keepIdx, :);
        featureTimestamps = featureTimestamps(keepIdx);
    end
end
