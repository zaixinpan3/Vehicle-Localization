function summary = compareCoarseSequences(candidateLabel, baselineLabel, frames, root)
% compareCoarseSequences: Agreement of a candidate coarse lattice with the baseline.
% Baseline cells are projected by their centers into the candidate lattice and
% compared as index sets; a one-cell (8-neighbour) tolerance is also reported.
    if nargin < 2 || isempty(baselineLabel), baselineLabel = 'baseline'; end
    if nargin < 4, root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    out = fullfile(root, 'output', 'coarse_lattice_20260924');
    base = load(fullfile(out, sprintf('%s_sequence.mat', baselineLabel)));
    cand = load(fullfile(out, sprintf('%s_sequence.mat', candidateLabel)));
    baseFrames = cellfun(@(r) r.frame, base.records); candFrames = cellfun(@(r) r.frame, cand.records);
    if nargin < 3 || isempty(frames), frames = intersect(baseFrames, candFrames); end
    channels = ["curb", "pole", "trafficSign", "road"];
    rows = [];
    for f = frames(:).'
        b = base.records{baseFrames == f}; c = cand.records{candFrames == f};
        row = struct('frame', f, 'elapsedBase', b.elapsed, 'elapsedCand', c.elapsed);
        for ch = channels
            if ch == "road"
                bIds = b.roadCells; cIds = c.roadCells;
            else
                bIds = b.pillarIndices{b.semanticNames == ch}; cIds = c.pillarIndices{c.semanticNames == ch};
            end
            bProj = projectCells(bIds, base.geometry, cand.geometry);
            cIds = double(cIds(:));
            row.(ch+"Base") = numel(bProj); row.(ch+"Cand") = numel(cIds);
            row.(ch+"Shared") = numel(intersect(bProj, cIds));
            row.(ch+"SharedTol") = nnz(ismember(cIds, dilate(bProj, cand.geometry)));
            row.(ch+"RecallTol") = nnz(ismember(bProj, dilate(cIds, cand.geometry)));
            % Nearest-centre distances (metres) inside the shared extent.
            [bx, by] = centres(bProj, cand.geometry); [cx, cy] = centres(cIds, cand.geometry);
            row.(ch+"MeanNearestCandToBase") = meanNearest([cx cy], [bx by]);
            row.(ch+"MeanNearestBaseToCand") = meanNearest([bx by], [cx cy]);
        end
        for k = 1:numel(b.cloudSemanticNames)
            name = b.cloudSemanticNames(k);
            row.(name+"ComponentsBase") = nnz(b.components.semanticId == k & inExtent(b.components.mean, cand.cloudGeometry));
            row.(name+"ComponentsCand") = nnz(c.components.semanticId == k);
            row.(name+"WeightBase") = b.layerWeights(k); row.(name+"WeightCand") = c.layerWeights(k);
            row.(name+"MeanCompDist") = meanNearest(c.components.mean(c.components.semanticId == k, :), ...
                b.components.mean(b.components.semanticId == k & inExtent(b.components.mean, cand.cloudGeometry), :));
        end
        row.groundPointsBase = b.sourceSummary.numGroundPoints; row.groundPointsCand = c.sourceSummary.numGroundPoints;
        row.retainedBase = b.sourceSummary.numRetainedPoints; row.retainedCand = c.sourceSummary.numRetainedPoints;
        rows = [rows; row]; %#ok<AGROW>
    end
    T = struct2table(rows);
    writetable(T, fullfile(out, sprintf('%s_vs_%s.csv', candidateLabel, baselineLabel)));
    summary = struct('candidate', candidateLabel, 'frames', numel(frames));
    fprintf('\n%s vs %s over %d frames\n', candidateLabel, baselineLabel, numel(frames));
    fprintf('%-12s %8s %8s %7s %7s %7s | %7s %7s %7s | %6s %6s\n', 'channel', 'baseCells', 'candCells', 'prec', 'rec', 'f1', 'precT', 'recT', 'f1T', 'd(c>b)', 'd(b>c)');
    for ch = channels
        nb = sum(T.(ch+"Base")); nc = sum(T.(ch+"Cand")); s = sum(T.(ch+"Shared"));
        st = sum(T.(ch+"SharedTol")); rt = sum(T.(ch+"RecallTol"));
        p = s/max(nc,1); r = s/max(nb,1); pt = st/max(nc,1); rr = rt/max(nb,1);
        f1 = 2*p*r/max(p+r,eps); f1t = 2*pt*rr/max(pt+rr,eps);
        dc = mean(T.(ch+"MeanNearestCandToBase"), 'omitnan'); db = mean(T.(ch+"MeanNearestBaseToCand"), 'omitnan');
        fprintf('%-12s %8d %8d %7.3f %7.3f %7.3f | %7.3f %7.3f %7.3f | %6.3f %6.3f\n', ch, nb, nc, p, r, f1, pt, rr, f1t, dc, db);
        summary.(ch) = struct('baseCells', nb, 'candCells', nc, 'precision', p, 'recall', r, 'f1', f1, ...
            'precisionTol', pt, 'recallTol', rr, 'f1Tol', f1t, 'meanNearestCandToBase', dc, 'meanNearestBaseToCand', db);
    end
    names = base.records{1}.cloudSemanticNames;
    fprintf('%-12s %9s %9s %8s %8s %8s\n', 'class', 'compBase', 'compCand', 'wBase', 'wCand', 'meanDist');
    for k = 1:numel(names)
        n = names(k);
        fprintf('%-12s %9d %9d %8.3f %8.3f %8.3f\n', n, sum(T.(n+"ComponentsBase")), sum(T.(n+"ComponentsCand")), ...
            mean(T.(n+"WeightBase")), mean(T.(n+"WeightCand")), mean(T.(n+"MeanCompDist"), 'omitnan'));
        summary.("components_"+n) = struct('base', sum(T.(n+"ComponentsBase")), 'cand', sum(T.(n+"ComponentsCand")), ...
            'weightBase', mean(T.(n+"WeightBase")), 'weightCand', mean(T.(n+"WeightCand")), 'meanDist', mean(T.(n+"MeanCompDist"), 'omitnan'));
    end
    fprintf('ground points: base %.0f cand %.0f (median per frame); retained: base %.0f cand %.0f\n', ...
        median(T.groundPointsBase), median(T.groundPointsCand), median(T.retainedBase), median(T.retainedCand));
    fprintf('elapsed median: base %.1f ms cand %.1f ms\n', 1000*median(T.elapsedBase), 1000*median(T.elapsedCand));
    summary.elapsedMedianBase = median(T.elapsedBase); summary.elapsedMedianCand = median(T.elapsedCand);
    summary.groundPointsMedianBase = median(T.groundPointsBase); summary.groundPointsMedianCand = median(T.groundPointsCand);
end

function ids = projectCells(sourceIds, sourceGeometry, targetGeometry)
    sourceIds = double(sourceIds(:));
    [row, col] = ind2sub(sourceGeometry.mapSize, sourceIds);
    centre = sourceGeometry.origin + ([col row] - 0.5).*sourceGeometry.cellSize;
    bins = floor((centre - targetGeometry.origin)./targetGeometry.cellSize) + 1;
    valid = bins(:,1) >= 1 & bins(:,1) <= targetGeometry.mapSize(2) & bins(:,2) >= 1 & bins(:,2) <= targetGeometry.mapSize(1);
    ids = unique(sub2ind(targetGeometry.mapSize, bins(valid,2), bins(valid,1)));
end

function ids = dilate(sourceIds, geometry)
    if isempty(sourceIds), ids = zeros(0,1); return; end
    [row, col] = ind2sub(geometry.mapSize, double(sourceIds(:)));
    [dr, dc] = meshgrid(-1:1, -1:1);
    rows = row + dr(:).'; cols = col + dc(:).';
    valid = rows >= 1 & rows <= geometry.mapSize(1) & cols >= 1 & cols <= geometry.mapSize(2);
    ids = unique(sub2ind(geometry.mapSize, rows(valid), cols(valid)));
end

function [x, y] = centres(ids, geometry)
    [row, col] = ind2sub(geometry.mapSize, double(ids(:)));
    x = geometry.origin(1) + (col - 0.5)*geometry.cellSize(1);
    y = geometry.origin(2) + (row - 0.5)*geometry.cellSize(2);
end

function d = meanNearest(query, reference)
    if isempty(query) || isempty(reference), d = NaN; return; end
    dist = sqrt(min((query(:,1) - reference(:,1).').^2 + (query(:,2) - reference(:,2).').^2, [], 2));
    d = mean(dist);
end

function mask = inExtent(xy, geometry)
    mask = xy(:,1) >= geometry.xMin & xy(:,1) < geometry.xMax & xy(:,2) >= geometry.yMin & xy(:,2) < geometry.yMax;
end
