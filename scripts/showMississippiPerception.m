function result = showMississippiPerception(frameIndex, matPath)
% showMississippiPerception: Run full perception and show one recorded frame.
%   result = showMississippiPerception(260) reads Mississippi frame 260 and
%   displays all finite source points with pcshow, including points outside
%   the perception ROI. Curbs are orange-red, poles cyan, and road markings
%   yellow. Larger feature markers overlay the complete gray point cloud.
%   These are the point masks returned by perceiveFrame, not ground truth.
%
%   An optional matPath selects another extracted point-cloud MAT file.
%   The returned struct retains the frame, configuration, perception output,
%   metrics, and graphics handles for inspection in the current session.
    arguments
        frameIndex (1, 1) double {mustBeInteger, mustBePositive} = 260
        matPath (1, 1) string = ""
    end

    projectRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(projectRoot);
    setupVehicleLocalization();
    if strlength(matPath) == 0
        matPath = fullfile(projectRoot, "data", "raw", "MissisipiPointClouds.mat");
    end
    [frame, numFrames] = loadPointCloudFrame(matPath, frameIndex);
    cfg = perceptionConfig();
    timer = tic;
    perception = perceiveFrame(frame, cfg);
    elapsedSeconds = toc(timer);

    xyz = double([frame.x(:), frame.y(:), frame.z(:)]);
    finiteMask = all(isfinite(xyz), 2);
    assert(any(finiteMask), "The selected frame has no finite XYZ points.");
    featureNames = ["curb", "pole", "roadMarking"];
    featureColors = [1.0, 0.25, 0.08; 0.0, 0.85, 1.0; 1.0, 0.9, 0.05];
    featureCounts = zeros(1, 3);
    selected = false(size(xyz, 1), 3);
    for featureIndex = 1:3
        mask = perception.featureMasks.(featureNames(featureIndex));
        assert(islogical(mask) && numel(mask) == size(xyz, 1), ...
            "Feature masks must index the original frame.");
        assert(all(finiteMask(mask(:))), "A feature contains nonfinite XYZ.");
        selected(:, featureIndex) = mask(:);
        featureCounts(featureIndex) = nnz(mask);
    end

    fig = figure("Name", sprintf("Mississippi frame %d - Full perception", frameIndex), ...
        "NumberTitle", "off", "Color", [0.06, 0.06, 0.08]);
    ax = axes("Parent", fig);
    pcshow(xyz(finiteMask, :), [0.42, 0.42, 0.46], ...
        "Parent", ax, "MarkerSize", 8);
    hold(ax, "on");
    handles = gobjects(1, 3);
    for featureIndex = 1:3
        points = xyz(selected(:, featureIndex), :);
        handles(featureIndex) = scatter3(ax, points(:, 1), points(:, 2), ...
            points(:, 3), 32, featureColors(featureIndex, :), "filled");
    end
    hold(ax, "off");
    labels = compose("%s: %d points", ["Curb"; "Pole"; "Road marking"], featureCounts(:));
    legend(ax, handles, labels, "TextColor", "white", ...
        "Color", [0.1, 0.1, 0.12], "Location", "northeast");
    title(ax, {sprintf("Mississippi frame %d | full perception: %.3f s", ...
        frameIndex, elapsedSeconds), ...
        sprintf("All %d finite source points shown; gray = source cloud", nnz(finiteMask))}, ...
        "Color", "white");
    xlabel(ax, "X (m)");
    ylabel(ax, "Y (m)");
    zlabel(ax, "Z (m)");
    view(ax, -35, 55);
    axis(ax, "tight");
    drawnow;

    metrics = struct("frameIndex", frameIndex, "numFrames", numFrames, ...
        "inputPoints", size(xyz, 1), "displayedPoints", nnz(finiteMask), ...
        "retainedPoints", perception.voxelGrid.numFilteredPoints, ...
        "curbPoints", featureCounts(1), "polePoints", featureCounts(2), ...
        "roadMarkingPoints", featureCounts(3), ...
        "overlappingFeaturePoints", nnz(sum(selected, 2) > 1), ...
        "perceptionSeconds", elapsedSeconds);
    result = struct("source", matPath, "frame", frame, "config", cfg, ...
        "perception", perception, "metrics", metrics, "figure", fig, "axes", ax);
    disp(metrics);
end
