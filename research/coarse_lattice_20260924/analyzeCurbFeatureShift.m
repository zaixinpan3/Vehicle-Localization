function analyzeCurbFeatureShift(frames, root)
% analyzeCurbFeatureShift: Quantiles of curb feature maps on baseline curb / road cells per lattice.
    if nargin < 2, root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    store = fullfile(root, 'data', 'raw', 'MissisipiPointClouds.mat');
    cfgs = {perceptionConfig("Mississippi", "offline"), perceptionConfig()};
    names = ["heightStepMeters", "residualSlopeRadians", "curvatureResponse", "roughnessMeters", "relativeHeightMeters", "totalBase", "linearity", "total"];
    values = cell(2, 2, numel(names)); % lattice x class x feature
    for f = frames
        frame = loadPointCloudFrame(store, f);
        products = cell(1, 2);
        for k = 1:2
            cfg = cfgs{k}; cfg.coarseProbabilityCloud.storeDiagnostics = true; cfg.featureNames = "curb";
            products{k} = perceiveFrame(frame, cfg);
        end
        a = products{1}.diagnostics.ground;
        curbA = cellCentres(a.curbCellMask, a.cellOrigin, a.cellSize); roadA = cellCentres(a.roadCellMask, a.cellOrigin, a.cellSize);
        for k = 1:2
            g = products{k}.diagnostics.ground; m = g.energyMaps;
            curb = maskFromCentres(curbA, g); road = maskFromCentres(roadA, g) & ~curb;
            for j = 1:numel(names)
                v = double(m.(names(j)));
                values{k, 1, j} = [values{k, 1, j}; v(curb & isfinite(v))];
                values{k, 2, j} = [values{k, 2, j}; v(road & isfinite(v))];
            end
        end
    end
    q = [0.1 0.25 0.5 0.75 0.9];
    for j = 1:numel(names)
        fprintf('\n%-22s %-6s %-5s', names(j), 'class', 'n'); fprintf(' %7s', "q"+string(q)); fprintf('\n');
        cls = ["curb", "road"];
        for k = 1:2
            for c = 1:2
                v = values{k, c, j};
                fprintf('%-22s %-6s %5d', sprintf('%.1f m', cfgs{k}.voxel.voxelSize(1)), cls(c), numel(v)); fprintf(' %7.4f', quantile(v, q)); fprintf('\n');
            end
        end
    end
end

function xy = cellCentres(mask, origin, cellSize)
    [row, col] = find(mask); xy = origin + ([col row] - 0.5) .* cellSize;
end

function mask = maskFromCentres(xy, g)
    bins = floor((xy - g.cellOrigin) ./ g.cellSize) + 1; sz = size(g.curbCellMask);
    valid = bins(:,1) >= 1 & bins(:,1) <= sz(2) & bins(:,2) >= 1 & bins(:,2) <= sz(1);
    mask = false(sz); mask(sub2ind(sz, bins(valid,2), bins(valid,1))) = true;
end
