function results = benchmarkCoarseProbabilityCloud(dataRoot, frameIndices)
% benchmarkCoarseProbabilityCloud: Compare the optimized voxel-only 2D NDT
% path with the legacy full perception plus dense semantic-grid construction
% on selected Mississippi frames. Report steady-state timeit measurements
% and retained MATLAB output sizes; no result file is written.
%
% Input:
%   dataRoot: optional repository data folder containing raw/
%       MissisipiPointClouds.mat
%   frameIndices: optional positive integer frame indices, default
%       [260 300 326]
%
% Output:
%   results: table of timings, speedups, output sizes, and component counts
    projectFolder = fileparts(fileparts(mfilename("fullpath")));
    run(fullfile(projectFolder, "setupVehicleLocalization.m"));
    if nargin < 1 || strlength(string(dataRoot)) == 0
        dataRoot = fullfile(projectFolder, "data");
    end
    if nargin < 2 || isempty(frameIndices)
        frameIndices = [260, 300, 326];
    end
    frameIndices = double(frameIndices(:));
    assert(all(isfinite(frameIndices)) && all(frameIndices >= 1) && ...
        all(frameIndices == floor(frameIndices)), ...
        "frameIndices must contain positive integers.");
    matPath = fullfile(dataRoot, "raw", "MissisipiPointClouds.mat");
    assert(isfile(matPath), "Mississippi point-cloud MAT file was not found.");

    cfg = perceptionConfig();
    cfg.executionMode = "legacyFull";
    numFrames = numel(frameIndices);
    fullPerceptionSeconds = zeros(numFrames, 1);
    denseProductSeconds = zeros(numFrames, 1);
    legacyTotalSeconds = zeros(numFrames, 1);
    optimizedSeconds = zeros(numFrames, 1);
    speedup = zeros(numFrames, 1);
    fullPerceptionBytes = zeros(numFrames, 1);
    denseProductBytes = zeros(numFrames, 1);
    optimizedProductBytes = zeros(numFrames, 1);
    componentCount = zeros(numFrames, 1);

    for rowIdx = 1:numFrames
        frame = loadPointCloudFrame(matPath, frameIndices(rowIdx));
        fullProduct = perceiveFrame(frame, cfg);
        denseProduct = buildSemanticVoxelGrid(fullProduct); %#ok<NASGU>
        optimizedProduct = perceiveCoarseProbabilityCloud(frame, cfg);
        fullPerceptionSeconds(rowIdx) = timeit(@() perceiveFrame(frame, cfg));
        denseProductSeconds(rowIdx) = timeit(@() buildSemanticVoxelGrid(fullProduct));
        legacyTotalSeconds(rowIdx) = ...
            fullPerceptionSeconds(rowIdx) + denseProductSeconds(rowIdx);
        optimizedSeconds(rowIdx) = ...
            timeit(@() perceiveCoarseProbabilityCloud(frame, cfg));
        speedup(rowIdx) = legacyTotalSeconds(rowIdx) ./ optimizedSeconds(rowIdx);
        fullInfo = whos("fullProduct");
        denseInfo = whos("denseProduct");
        optimizedInfo = whos("optimizedProduct");
        fullPerceptionBytes(rowIdx) = fullInfo.bytes;
        denseProductBytes(rowIdx) = denseInfo.bytes;
        optimizedProductBytes(rowIdx) = optimizedInfo.bytes;
        componentCount(rowIdx) = optimizedProduct.components.numComponents;
    end

    results = table(frameIndices, fullPerceptionSeconds, denseProductSeconds, ...
        legacyTotalSeconds, optimizedSeconds, speedup, fullPerceptionBytes, ...
        denseProductBytes, optimizedProductBytes, componentCount);
    disp(results);
end
