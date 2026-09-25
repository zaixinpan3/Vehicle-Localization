function T = probePoleCoreStatistic(frames, root)
% probePoleCoreStatistic: XY density peak and its vertical extent per 0.6 m off-ground pillar.
% For every pillar with >= 12 off-ground points and >= 1.5 m height, find the
% densest XY location inside the pillar (0.1 m histogram peak refined by one
% mean shift), then measure the fraction of pillar points within 0.15 / 0.20 m
% of it and the z extent of those core points. Compared against baseline poles.
    if nargin < 2, root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    out = fullfile(root, 'output', 'coarse_lattice_20260924');
    base = load(fullfile(out, 'baseline_sequence.mat'), 'records', 'geometry'); bf = cellfun(@(r) r.frame, base.records);
    store = fullfile(root, 'data', 'raw', 'MissisipiPointClouds.mat');
    cfg = perceptionConfig(); cfg.coarseProbabilityCloud.storeDiagnostics = true; cfg.featureNames = ["pole", "trafficSign"];
    rows = [];
    for f = frames
        frame = loadPointCloudFrame(store, f);
        grid = pillarizePointCloud(frame, cfg.voxel); gIdx = segmentGround(grid, cfg.groundSegmentation);
        groundMask = false(numel(frame.x), 1); groundMask(gIdx) = true;
        off = ~groundMask(grid.pointIndices);
        xyz = grid.points(off, :); pid = double(grid.pointPillarLinIdx(off));
        p = perceiveFrame(frame, cfg); gb = p.candidates.geometry;
        detected = double(p.candidates.pillarIndices{p.candidates.semanticNames == "pole"});
        o = p.diagnostics.offGround; maps = o.columnMaps; st = maps.statistics; spid = double(st.pillarIndices);
        b = base.records{bf == f}; ids = double(b.pillarIndices{b.semanticNames == "pole"});
        [row, col] = ind2sub(base.geometry.mapSize, ids); c = base.geometry.origin + ([col row] - 0.5) .* base.geometry.cellSize;
        bins = floor((c - gb.origin) ./ gb.cellSize) + 1;
        ok = bins(:,1) >= 1 & bins(:,1) <= gb.mapSize(2) & bins(:,2) >= 1 & bins(:,2) <= gb.mapSize(1);
        basePoles = unique(sub2ind(gb.mapSize, bins(ok,2), bins(ok,1)));
        [uid, ~, g] = unique(pid); n = accumarray(g, 1);
        zmin = accumarray(g, xyz(:,3), [], @min); zmax = accumarray(g, xyz(:,3), [], @max);
        keep = find(n >= 12 & zmax - zmin >= 1.5);
        for k = keep(:).'
            pts = xyz(g == k, :);
            [r0, c0] = ind2sub(gb.mapSize, uid(k)); lo = gb.origin + ([c0 r0] - 1) .* gb.cellSize;
            hb = min(floor((pts(:,1:2) - lo) ./ 0.1) + 1, 6); hb = max(hb, 1);
            counts = accumarray(hb, 1, [6 6]);
            [~, m] = max(counts(:)); [mr, mc] = ind2sub([6 6], m);
            peak = lo + ([mr mc] - 0.5) * 0.1;
            near = hypot(pts(:,1) - peak(1), pts(:,2) - peak(2)) <= 0.15;
            if any(near), peak = mean(pts(near, 1:2), 1); end
            d = hypot(pts(:,1) - peak(1), pts(:,2) - peak(2));
            core15 = d <= 0.15; core20 = d <= 0.20;
            zc = pts(core15, 3);
            centre = lo + gb.cellSize / 2; ob = floor((centre - maps.origin) ./ [maps.dx maps.dy]) + 1;
            l = sub2ind(maps.mapSize, ob(2), ob(1)); j = find(spid == l, 1);
            cv = st.covarianceXYZ(j, :); slope = cv(4:5) ./ max(cv(6), eps);
            rStd = sqrt(max(0, cv(1) + cv(3) - sum(cv(4:5).^2) / max(cv(6), eps)));
            rows = [rows; f, uid(k), n(k), zmax(k) - zmin(k), mean(core15), mean(core20), ...
                max(zc) - min(zc), std(zc), ismember(uid(k), basePoles), ismember(uid(k), detected), ...
                rStd, sqrt(cv(6)), atand(norm(slope)), maps.lineScore(l), maps.pointScore(l), o.candidates.contextFraction(l), o.candidates.coreMask(l)]; %#ok<AGROW>
        end
    end
    T = array2table(rows, 'VariableNames', {'frame', 'cell', 'n', 'h', 'coreFrac15', 'coreFrac20', 'coreZRange', 'coreZStd', 'isBase', 'detected', 'rStd', 'hStd', 'tilt', 'line', 'point', 'context', 'core'});
    save(fullfile(out, 'pole_core_table.mat'), 'T');
    B = T(T.isBase > 0, :); N = T(T.isBase == 0, :); q = [.1 .25 .5 .75 .9];
    fprintf('pillars %d (n>=12, h>=1.5): baseline poles %d, others %d\n', height(T), height(B), height(N));
    for v = ["coreFrac15", "coreFrac20", "coreZRange", "coreZStd", "n", "h"]
        fprintf('%-11s base %s | others %s\n', v, mat2str(quantile(B.(v), q), 3), mat2str(quantile(N.(v), q), 3));
    end
    fprintf('\ncurrent detector on this population: '); report(T, T.detected > 0);
    base = T.hStd >= 0.3 & T.tilt <= 20 & T.line <= 0.9;
    fprintf('A  rStd<=0.18 & point>=0.7 (no context):          '); report(T, base & T.rStd <= 0.18 & T.point >= 0.7);
    fprintf('A+ rStd<=0.18 & point>=0.7 & context>0.58:        '); report(T, base & T.rStd <= 0.18 & T.point >= 0.7 & T.context > 0.58);
    for fr = [0.5 0.6 0.7]
        for pt = [0.5 0.6 0.7]
            fprintf('B  coreFrac15>=%.1f & coreZ>=1.5 & point>=%.1f:      ', fr, pt); report(T, base & T.coreFrac15 >= fr & T.coreZRange >= 1.5 & T.point >= pt);
            fprintf('B+ ... & context>0.58:                              '); report(T, base & T.coreFrac15 >= fr & T.coreZRange >= 1.5 & T.point >= pt & T.context > 0.58);
            fprintf('C  B or (rStd<=0.18) with point>=%.1f, context>0.58: ', pt); report(T, base & (T.coreFrac15 >= fr | T.rStd <= 0.18) & T.coreZRange >= 1.5 & T.point >= pt & T.context > 0.58);
        end
    end
    for fr = [0.5 0.6]
        fprintf('D  coreFrac15>=%.1f & coreZ>=1.5 & rStd<=0.25 & point>=0.6 & ctx>0.45: ', fr); report(T, base & T.coreFrac15 >= fr & T.coreZRange >= 1.5 & T.rStd <= 0.25 & T.point >= 0.6 & T.context > 0.45);
    end
end

function report(T, pred)
    tp = nnz(pred & T.isBase); fp = nnz(pred & ~T.isBase); fn = nnz(~pred & T.isBase);
    p = tp / max(tp + fp, 1); r = tp / max(tp + fn, 1);
    fprintf('tp %4d fp %4d fn %4d  P %.3f R %.3f F1 %.3f\n', tp, fp, fn, p, r, 2*p*r/max(p+r, eps));
end
