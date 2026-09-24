function T = sweepPoleGates(frames, root)
% sweepPoleGates: Per-pillar pole gate statistics on the 0.6 m lattice versus baseline poles.
    if nargin < 2, root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    out = fullfile(root, 'output', 'coarse_lattice_20260924');
    base = load(fullfile(out, 'baseline_sequence.mat'), 'records', 'geometry');
    baseFrames = cellfun(@(r) r.frame, base.records);
    store = fullfile(root, 'data', 'raw', 'MissisipiPointClouds.mat');
    cfg = perceptionConfig(); cfg.coarseProbabilityCloud.storeDiagnostics = true; cfg.featureNames = ["pole", "trafficSign"];
    rows = [];
    for f = frames
        frame = loadPointCloudFrame(store, f); p = perceiveFrame(frame, cfg);
        o = p.diagnostics.offGround; maps = o.columnMaps; st = maps.statistics; gb = p.candidates.geometry;
        b = base.records{baseFrames == f}; ids = double(b.pillarIndices{b.semanticNames == "pole"});
        [row, col] = ind2sub(base.geometry.mapSize, ids); centre = base.geometry.origin + ([col row] - 0.5) .* base.geometry.cellSize;
        oBins = floor((centre - maps.origin) ./ [maps.dx maps.dy]) + 1;
        inside = oBins(:,1) >= 1 & oBins(:,1) <= maps.mapSize(2) & oBins(:,2) >= 1 & oBins(:,2) <= maps.mapSize(1);
        inExtent = centre(:,1) >= gb.origin(1) & centre(:,1) < gb.origin(1) + gb.mapSize(2)*gb.cellSize(1) & ...
            centre(:,2) >= gb.origin(2) & centre(:,2) < gb.origin(2) + gb.mapSize(1)*gb.cellSize(2);
        baseLocal = unique(sub2ind(maps.mapSize, oBins(inside,2), oBins(inside,1)));
        missingBase = nnz(inExtent) - numel(baseLocal); % baseline poles in extent but outside the cropped raster (empty)
        pid = double(st.pillarIndices); cv = st.covarianceXYZ;
        slope = cv(:,4:5) ./ max(cv(:,6), eps);
        rStd = sqrt(max(0, cv(:,1) + cv(:,3) - sum(cv(:,4:5).^2, 2) ./ max(cv(:,6), eps)));
        h = st.maximumXYZ(:,3) - st.minimumXYZ(:,3);
        r = table(repmat(f, numel(pid), 1), pid, st.count, h, sqrt(cv(:,6)), atand(vecnorm(slope, 2, 2)), rStd, ...
            double(maps.lineScore(pid)), double(maps.pointScore(pid)), double(o.candidates.contextFraction(pid)), ...
            double(o.candidates.coreMask(pid)), double(o.poleCellMask(pid)), double(ismember(pid, baseLocal)), ...
            'VariableNames', {'frame', 'cell', 'n', 'h', 'hStd', 'tilt', 'rStd', 'line', 'point', 'context', 'core', 'detected', 'isBase'});
        r = r(r.n >= 6 & r.h >= 1.0, :);
        rows = [rows; r]; %#ok<AGROW>
        if missingBase > 0, fprintf('frame %d: %d baseline pole cells have no off-ground pillar\n', f, missingBase); end
    end
    T = rows; save(fullfile(out, 'pole_gate_table.mat'), 'T');
    fprintf('pillars %d, baseline poles %d, detected %d\n', height(T), nnz(T.isBase), nnz(T.detected));
    report(T, T.detected > 0, 'current run');
    for tr = [0.15 0.18 0.20 0.22]
        for tc = [0.45 0.50 0.58 0.65]
            for tp = [0.60 0.70 0.80]
                pred = T.n >= 12 & T.h >= 1.5 & T.hStd >= 0.3 & T.tilt <= 20 & T.rStd <= tr & T.line <= 0.9 & T.context > tc & T.point >= tp;
                report(T, pred, sprintf('rStd<=%.2f context>%.2f point>=%.2f', tr, tc, tp));
            end
        end
    end
end

function report(T, pred, label)
    tp = nnz(pred & T.isBase); fp = nnz(pred & ~T.isBase); fn = nnz(~pred & T.isBase);
    p = tp / max(tp + fp, 1); r = tp / max(tp + fn, 1);
    fprintf('%-40s tp %4d fp %4d fn %4d  P %.3f R %.3f F1 %.3f\n', label, tp, fp, fn, p, r, 2*p*r/max(p+r, eps));
end
