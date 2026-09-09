function featureData = collectFeatureObservations(matPath, frameIndices, framePoseTable, perceptionCfg, cfg)
% collectFeatureObservations: Run the perception module over a sequence of
% frames and register every semantic feature observation into the global
% map frame. For each frame the full-frame feature masks are converted to
% finite [x y z] points in the vehicle frame and transformed with the matched
% high-precision GNSS/INS pose, giving one global point cloud per feature
% class and frame, the raw material of the offline map.
%
% Input:
%   matPath: path of the MAT file holding the organized point-cloud frames
%   frameIndices: [1 x F] one-based frame indices to process
%   framePoseTable: table with one matched global pose row per frame, in
%       the same order as frameIndices (see readFramePoseTable)
%   perceptionCfg: struct from perceptionConfig
%   cfg: struct from featureMapBuildConfig with featureNames and logEnabled
%
% Output:
%   featureData: struct with featureNames, frameIndices, framePoseTable,
%       pointsByFeatureFrame {C x F} of [N x 3] global points, counts
%       [F x C], numFrames, and frameSummaryTable
    featureNames = validatePerceptionFeatureNames(cfg.featureNames);
    frameIndices = double(frameIndices(:).');
    numFeatures = numel(featureNames);
    numRequestedFrames = numel(frameIndices);
    assert(height(framePoseTable) == numRequestedFrames, ...
        "framePoseTable must contain exactly one pose row per requested frame.");
    pointsByFeatureFrame = cell(numFeatures, numRequestedFrames);
    counts = zeros(numRequestedFrames, numFeatures);
    numFrames = NaN;
    calibration=lidarFrameCalibrationConfig();
    if isfield(perceptionCfg,'frameCalibration'), calibration=validateLidarFrameCalibration(perceptionCfg.frameCalibration); end

    perceptionCfg.executionMode = "offline";
    perceptionCfg.featureNames = featureNames;
    mappingSupport.logStep(cfg, "feature.collect", "frameCount=%d | featureCount=%d", numRequestedFrames, numFeatures);
    for frameListIdx = 1:numRequestedFrames
        frameIdx = frameIndices(frameListIdx);
        [frame, currentNumFrames] = loadPointCloudFrame(matPath, frameIdx);
        if isnan(numFrames)
            numFrames = currentNumFrames;
        end
        perception = perceiveFrame(frame, perceptionCfg);
        for featureIdx = 1:numFeatures
            featureName = featureNames(featureIdx);
            featurePoints = pointsFromFeatureMask(frame, perception.featureMasks.(char(featureName)));
            featurePoints = featurePoints*calibration.rotation.'+calibration.translation;
            featurePoints = registerPointsToGlobalFrame(featurePoints, framePoseTable(frameListIdx, :));
            pointsByFeatureFrame{featureIdx, frameListIdx} = featurePoints;
            counts(frameListIdx, featureIdx) = size(featurePoints, 1);
        end
        mappingSupport.logStep(cfg, "frame.done", "frame=%d/%d | %s", frameIdx, currentNumFrames, ...
            char(formatFrameCounts(featureNames, counts(frameListIdx, :))));
    end

    featureVariableNames = string(matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(cellstr(featureNames))));
    featureData = struct();
    featureData.featureNames = featureNames;
    featureData.frameCalibration=calibration;
    featureData.frameIndices = frameIndices;
    featureData.framePoseTable = framePoseTable;
    featureData.pointsByFeatureFrame = pointsByFeatureFrame;
    featureData.counts = counts;
    featureData.numFrames = numFrames;
    featureData.frameSummaryTable = array2table([frameIndices(:), counts], ...
        "VariableNames", ["frameIdx", featureVariableNames(:).']);
end

function featurePoints = pointsFromFeatureMask(frame, featureMask)
% pointsFromFeatureMask: Extract finite [x y z] feature points from a
% full-frame logical mask aligned with an organized point-cloud frame.
%
% Input:
%   frame: organized point-cloud frame with x, y, and z fields
%   featureMask: full-frame logical mask for one semantic feature
%
% Output:
%   featurePoints: [N x 3] finite feature point coordinates
    x = double(frame.x(:));
    y = double(frame.y(:));
    z = double(frame.z(:));
    featureMask = logical(featureMask(:));
    assert(numel(featureMask) == numel(x), "featureMask must align with the frame point count.");
    validMask = featureMask & isfinite(x) & isfinite(y) & isfinite(z);
    featurePoints = [x(validMask), y(validMask), z(validMask)];
end

function countText = formatFrameCounts(featureNames, counts)
% formatFrameCounts: Format one compact feature-count status string
% for command-window progress reporting while frames are processed.
%
% Input:
%   featureNames: [1 x C] string feature names
%   counts: [1 x C] numeric point counts
%
% Output:
%   countText: string scalar containing name=count pairs
    parts = strings(1, numel(featureNames));
    for idx = 1:numel(featureNames)
        parts(idx) = sprintf("%s=%d", featureNames(idx), round(counts(idx)));
    end
    countText = strjoin(parts, " | ");
end
