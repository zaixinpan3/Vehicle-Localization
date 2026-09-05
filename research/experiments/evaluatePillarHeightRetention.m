function [runtime, heightSummary] = evaluatePillarHeightRetention(outputDirectory)
% evaluatePillarHeightRetention: Inspect the cost and content of XYZ moments.
% This research probe leaves production perception and mapping unchanged.
% Timings cover moment accumulation on all retained XY pillars, excluding
% loading, grouping, classification, and registration. Candidate height
% summaries include all retained members and are not fine-feature truth.
    arguments
        outputDirectory (1, 1) string
    end
    root = fileparts(fileparts(fileparts(mfilename("fullpath"))));
    frameIndices = [260, 550, 900];
    cfg = perceptionConfig();
    runtimeRows = zeros(numel(frameIndices), 8);
    heightRows = cell(numel(frameIndices)*3, 8);
    for frameNumber = 1:numel(frameIndices)
        frameIndex = frameIndices(frameNumber);
        frame = loadPointCloudFrame(fullfile(root, "data", "raw", "MissisipiPointClouds.mat"), frameIndex);
        pillars = pillarizePointCloud(frame, cfg.voxel);
        points = double(pillars.points);
        indices = double(pillars.pointPillarLinIdx);
        numCells = prod(pillars.pillarGeometry.mapSize);
        planar = aggregatePlanarCellMoments(points, indices, numCells);
        spatial = spatialMoments(points, indices, numCells);
        meanError = max(abs(planar.mean-spatial.mean(:, 1:2)), [], "all");
        scatterError = max(abs(planar.covariance-spatial.covariance(:, [1, 2, 4])), [], "all");
        assert(isequal(planar.count, spatial.count) && meanError < 1e-12 && scatterError < 1e-12);
        % Interleave repeated timeit estimates to reduce ordering effects.
        elapsed = zeros(3, 2);
        for repetition = 1:3
            if mod(repetition, 2) == 1
                elapsed(repetition, 1) = timeit(@() aggregatePlanarCellMoments(points, indices, numCells));
                elapsed(repetition, 2) = timeit(@() spatialMoments(points, indices, numCells));
            else
                elapsed(repetition, 2) = timeit(@() spatialMoments(points, indices, numCells));
                elapsed(repetition, 1) = timeit(@() aggregatePlanarCellMoments(points, indices, numCells));
            end
        end
        timing = median(elapsed, 1)*1000;
        runtimeRows(frameNumber, :) = [frameIndex, size(points, 1), nnz(planar.count), ...
            timing, timing(2)-timing(1), meanError, scatterError];
        perception = perceiveFrame(frame, cfg);
        lower = accumarray(indices, points(:, 3), [numCells, 1], @min, NaN);
        upper = accumarray(indices, points(:, 3), [numCells, 1], @max, NaN);
        for semanticIndex = 1:3
            selected = double(perception.candidates.pillarIndices{semanticIndex});
            selected = selected(planar.count(selected) >= 4);
            verticalSd = sqrt(max(spatial.covariance(selected, 6), 0));
            horizontalSd = sqrt(max(spatial.covariance(selected, 1)+spatial.covariance(selected, 4), 0));
            row = (frameNumber-1)*3+semanticIndex;
            heightRows(row, :) = {frameIndex, perception.candidates.semanticNames(semanticIndex), ...
                numel(selected), median(planar.count(selected)), median(upper(selected)-lower(selected)), ...
                median(verticalSd), median(horizontalSd), median(verticalSd./max(horizontalSd, eps))};
        end
    end
    runtime = array2table(runtimeRows, "VariableNames", ["frameIndex", "retainedPoints", ...
        "occupiedPillars", "xyMilliseconds", "xyzMilliseconds", "additionalMilliseconds", ...
        "maximumXYMeanError", "maximumXYScatterError"]);
    heightSummary = cell2table(heightRows, "VariableNames", ["frameIndex", "semanticName", ...
        "pillarsWithAtLeastFourPoints", "medianPointCount", "medianZSpanMeters", ...
        "medianZStandardDeviationMeters", "medianHorizontalRmsMeters", "medianVerticalToHorizontalRatio"]);
    if ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end
    writetable(runtime, fullfile(outputDirectory, "moment_runtime.csv"));
    writetable(heightSummary, fullfile(outputDirectory, "candidate_height_summary.csv"));
    disp(runtime);
    disp(heightSummary);
end

function moments = spatialMoments(points, indices, numCells)
% spatialMoments: Extend the production two-pass population scatter to XYZ.
% Packed covariance order is xx, xy, xz, yy, yz, zz. No feature selection.
    count = accumarray(indices, 1, [numCells, 1], @sum, 0);
    meanXYZ = zeros(numCells, 3);
    for dimension = 1:3
        meanXYZ(:, dimension) = accumarray(indices, points(:, dimension), ...
            [numCells, 1], @sum, 0)./max(count, 1);
    end
    residual = points-meanXYZ(indices, :);
    products = [residual(:, 1).^2, residual(:, 1).*residual(:, 2), residual(:, 1).*residual(:, 3), ...
        residual(:, 2).^2, residual(:, 2).*residual(:, 3), residual(:, 3).^2];
    covariance = zeros(numCells, 6);
    for entry = 1:6
        covariance(:, entry) = accumarray(indices, products(:, entry), [numCells, 1], @sum, 0)./max(count, 1);
    end
    moments = struct("count", count, "mean", meanXYZ, "covariance", covariance);
end
