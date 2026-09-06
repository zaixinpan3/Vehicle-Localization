function result = showMississippiPerception(frameIndex, matPath, mode, cfg)
% showMississippiPerception: Run full perception and show one recorded frame.
%   result = showMississippiPerception(260) reads Mississippi frame 260 and
%   displays all finite source points with pcshow, including points outside
%   the perception ROI. The optional cfg selects invocation channels. Without
%   cfg, the MAT filename selects the Downtown or Mississippi profile.
%   Larger selected-feature markers overlay the complete gray point cloud.
%   These are the point masks returned by perceiveFrame, not ground truth.
%
%   An optional matPath selects another extracted point-cloud MAT file.
%   mode="coarseProbabilityCloud" colors membership in candidate XY pillars
%   for inspection only; those colors are not point-level feature decisions.
%   mode="offline" (default) displays points accepted by fine refinement.
%   The returned struct retains the frame, configuration, perception output,
%   metrics, and graphics handles for inspection in the current session.
    arguments
        frameIndex (1, 1) double {mustBeInteger, mustBePositive} = 260
        matPath (1, 1) string = ""
        mode (1, 1) string {mustBeMember(mode,["offline","coarseProbabilityCloud"])} = "offline"
        cfg (1, 1) struct = struct()
    end

    projectRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(projectRoot);
    setupVehicleLocalization();
    if strlength(matPath) == 0
        matPath = fullfile(projectRoot, "data", "raw", "MissisipiPointClouds.mat");
    end
    [frame, numFrames] = loadPointCloudFrame(matPath, frameIndex);
    [~,datasetName]=fileparts(matPath);
    if isempty(fieldnames(cfg))
        dataset="Mississippi";
        if contains(lower(datasetName),"downtown"), dataset="Downtown"; end
        cfg=perceptionConfig(dataset);
    end
    cfg.executionMode = mode;
    timer = tic;
    perception = perceiveFrame(frame, cfg);
    elapsedSeconds = toc(timer);

    xyz = double([frame.x(:), frame.y(:), frame.z(:)]);
    finiteMask = all(isfinite(xyz), 2);
    assert(any(finiteMask), "The selected frame has no finite XYZ points.");
    featureNames = perception.featureNames;
    paletteNames = ["curb","pole","roadMarking","facade","trafficSign"];
    palette = [1 .25 .08;0 .85 1;1 .9 .05;.3 1 .4;1 .2 .9];
    [~,colorRows]=ismember(featureNames,paletteNames);
    featureColors=palette(colorRows,:);
    numFeatures=numel(featureNames);
    featureCounts = zeros(1, numFeatures);
    selected = false(size(xyz, 1), numFeatures);
    coarse = mode == "coarseProbabilityCloud";
    if coarse
        pillars = pillarizePointCloud(frame,cfg.voxel);
    end
    for featureIndex = 1:numFeatures
        if coarse
            mask = false(size(xyz,1),1);
            semanticIndex = perception.candidates.semanticNames==featureNames(featureIndex);
            member = ismember(pillars.pointPillarLinIdx,perception.candidates.pillarIndices{semanticIndex});
            mask(double(pillars.pointIndices(member))) = true;
        else
            mask = perception.featureMasks.(featureNames(featureIndex));
        end
        assert(islogical(mask) && numel(mask) == size(xyz, 1), ...
            "Feature masks must index the original frame.");
        assert(all(finiteMask(mask(:))), "A feature contains nonfinite XYZ.");
        selected(:, featureIndex) = mask(:);
        featureCounts(featureIndex) = nnz(mask);
    end

    fig = figure("Name", sprintf("%s frame %d - %s", datasetName, frameIndex, mode), ...
        "NumberTitle", "off", "Color", [0.06, 0.06, 0.08], "Position", [100 100 1200 800]);
    ax = axes("Parent", fig);
    pcshow(xyz(finiteMask, :), [0.42, 0.42, 0.46], ...
        "Parent", ax, "MarkerSize", 8);
    hold(ax, "on");
    handles = gobjects(1, numFeatures);
    for featureIndex = 1:numFeatures
        points = xyz(selected(:, featureIndex), :);
        handles(featureIndex) = scatter3(ax, points(:, 1), points(:, 2), ...
            points(:, 3), 32, featureColors(featureIndex, :), "filled");
    end
    hold(ax, "off");
    labels = compose("%s: %d points", featureNames(:), featureCounts(:));
    if numFeatures>0
        legend(ax, handles, labels, "TextColor", "white", ...
            "Color", [0.1, 0.1, 0.12], "Location", "northeast");
    end
    detail = sprintf("All %d finite source points shown; gray = source cloud", nnz(finiteMask));
    titleLines = {sprintf("%s frame %d | %s: %.3f s", ...
        datasetName, frameIndex, mode, elapsedSeconds), detail};
    if coarse
        titleLines{end+1} = "Colors show candidate pillar membership, before point validation";
    end
    title(ax, titleLines, "Color", "white", "FontSize", 11);
    xlabel(ax, "X (m)");
    ylabel(ax, "Y (m)");
    zlabel(ax, "Z (m)");
    view(ax, -35, 55);
    axis(ax, "tight");
    drawnow;

    metrics = struct("frameIndex", frameIndex, "numFrames", numFrames, ...
        "inputPoints", size(xyz, 1), "displayedPoints", nnz(finiteMask), ...
        "retainedPoints", perception.sourceSummary.numRetainedPoints, ...
        "featureNames",featureNames,"featurePointCounts",featureCounts, ...
        "overlappingFeaturePoints", nnz(sum(selected, 2) > 1), ...
        "perceptionSeconds", elapsedSeconds);
    result = struct("source", matPath, "frame", frame, "config", cfg, ...
        "perception", perception, "metrics", metrics, "figure", fig, "axes", ax);
    disp(metrics);
end
