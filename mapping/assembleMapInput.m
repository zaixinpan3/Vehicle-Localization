function mapInput = assembleMapInput(featureData, cfg)
% assembleMapInput: Assemble BEV [x y] map-builder inputs from
% per-feature/per-frame semantic point clouds, excluding facade by
% construction and only passing classes with enough points across at least
% two frame bins to the fail-fast GMM builder.
%
% Input:
%   featureData: struct produced by collectFeatureObservations
%   cfg: test configuration struct with map-input thresholds
%
% Output:
%   mapInput: struct with points, labels, timestamps, allFeatureNames,
%       buildFeatureNames, pointsByFeature, timestampsByFeature, and counts
    featureNames = string(featureData.featureNames(:));
    frameIndices = double(featureData.frameIndices(:).');
    numFeatures = numel(featureNames);
    pointsByFeature = cell(numFeatures, 1);
    timestampsByFeature = cell(numFeatures, 1);
    buildFeatureMask = false(numFeatures, 1);
    rawPointCounts = zeros(numFeatures, 1);
    buildPointCounts = zeros(numFeatures, 1);
    timestampBinCounts = zeros(numFeatures, 1);

    logStep(cfg, "mapinput.start", "features=%d | maxPointsPerClass=%g | minPoints=%d | minTimestampBins=%d", ...
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
        logStep(cfg, "mapinput.feature", "feature=%s | rawPoints=%d | buildPoints=%d | timestampBins=%d | build=%d", ...
            char(featureNames(featureIdx)), rawPointCounts(featureIdx), buildPointCounts(featureIdx), timestampBinCounts(featureIdx), buildFeatureMask(featureIdx));
    end

    buildFeatureNames = featureNames(buildFeatureMask);
    totalBuildPointCount = sum(buildPointCounts(buildFeatureMask));
    points = zeros(totalBuildPointCount, 2);
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
        labels(rangeIdx, 1) = repmat(featureNames(featureIdx), pointCount, 1);
        timestamps(rangeIdx, 1) = featureTimestamps(:);
        writeIdx = writeIdx + pointCount;
    end

    mapInput = struct();
    mapInput.points = points;
    mapInput.labels = labels;
    mapInput.timestamps = timestamps;
    mapInput.allFeatureNames = featureNames;
    mapInput.buildFeatureNames = buildFeatureNames;
    mapInput.pointsByFeature = pointsByFeature;
    mapInput.timestampsByFeature = timestampsByFeature;
    mapInput.rawPointCounts = rawPointCounts;
    mapInput.buildPointCounts = buildPointCounts;
    mapInput.timestampBinCounts = timestampBinCounts;
    logStep(cfg, "mapinput.done", "totalBuildPoints=%d | buildClasses=%s", totalBuildPointCount, strjoin(buildFeatureNames, ", "));
end

function [featurePoints, featureTimestamps] = concatenateFeatureFrames(pointsByFrame, frameIndices)
% concatenateFeatureFrames: Concatenate one feature's per-frame point
% clouds and generate one timestamp value per retained point using the
% original frame index.
%
% Input:
%   pointsByFrame: [1 x F] cell array with [Nf x 3] point arrays
%   frameIndices: [1 x F] numeric original frame indices
%
% Output:
%   featurePoints: [N x 3] concatenated finite feature points
%   featureTimestamps: [N x 1] frame index per feature point
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
%
% Input:
%   featurePoints: [N x 3] feature point coordinates
%   featureTimestamps: [N x 1] frame indices aligned with featurePoints
%   cfg: test configuration struct with maxMapPointsPerClass
%
% Output:
%   featurePoints: filtered [M x 3] feature point coordinates
%   featureTimestamps: filtered [M x 1] frame indices
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
