function out = diagnoseLatticeFrame(frameIndex, root)
% diagnoseLatticeFrame: Side-by-side coarse products of the 0.3 m and 0.6 m lattices.
% Renders masks in metric coordinates and tabulates pole gate statistics of
% every baseline (0.3 m) pole pillar projected onto the 0.6 m lattice.
    if nargin < 2, root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    folder = fullfile(root, 'output', 'coarse_lattice_20260924', 'frames'); if ~isfolder(folder), mkdir(folder); end
    frame = loadPointCloudFrame(fullfile(root, 'data', 'raw', 'MissisipiPointClouds.mat'), frameIndex);
    fineCfg = perceptionConfig("Mississippi", "offline"); fineCfg.coarseProbabilityCloud.storeDiagnostics = true;
    coarseCfg = perceptionConfig(); coarseCfg.coarseProbabilityCloud.storeDiagnostics = true;
    a = perceiveFrame(frame, fineCfg); b = perceiveFrame(frame, coarseCfg);
    products = {a, b}; labels = ["0.3 m lattice (offline stages)", "0.6 m lattice (coarse)"];
    fig = figure('Visible', 'off', 'Position', [0 0 1800 900]);
    for k = 1:2
        p = products{k}; g = p.diagnostics.ground; o = p.diagnostics.offGround;
        subplot(1, 2, k); hold on; axis equal; grid on;
        drawCells(g.stats.countMap > 0, g.cellOrigin, g.cellSize, [0.85 0.85 0.85]);
        drawCells(g.roadCellMask, g.cellOrigin, g.cellSize, [0.6 0.6 0.9]);
        drawCells(g.curbCellMask, g.cellOrigin, g.cellSize, [0.1 0.7 0.1]);
        drawCells(o.poleCellMask, o.columnMaps.origin, [o.columnMaps.dx o.columnMaps.dy], [0.9 0.1 0.1]);
        drawCells(o.trafficSignCellMask, o.columnMaps.origin, [o.columnMaps.dx o.columnMaps.dy], [0.1 0.1 0.9]);
        xlim([-30 30]); ylim([-30 30]); title(sprintf('frame %d: %s', frameIndex, labels(k)));
        xlabel('x [m]'); ylabel('y [m]');
    end
    png = fullfile(folder, sprintf('frame%04d_masks.png', frameIndex));
    exportgraphics(fig, png, 'Resolution', 110); close(fig);
    fprintf('Saved %s\n', png);
    % Pole gate table: baseline pole pillars projected into the 0.6 m lattice.
    poleTable(a, b, coarseCfg);
    out = struct('fine', a, 'coarse', b, 'png', png);
end

function drawCells(mask, origin, cellSize, color)
    [row, col] = find(mask);
    if isempty(row), return; end
    x = origin(1) + (col - 1) * cellSize(1); y = origin(2) + (row - 1) * cellSize(2);
    for k = 1:numel(x)
        rectangle('Position', [x(k) y(k) cellSize(1) cellSize(2)], 'FaceColor', color, 'EdgeColor', 'none');
    end
end

function poleTable(a, b, cfg)
    ga = a.candidates.geometry; gb = b.candidates.geometry;
    ids = double(a.candidates.pillarIndices{a.candidates.semanticNames == "pole"});
    [row, col] = ind2sub(ga.mapSize, ids);
    centre = ga.origin + ([col row] - 0.5) .* ga.cellSize;
    bins = floor((centre - gb.origin) ./ gb.cellSize) + 1;
    valid = bins(:,1) >= 1 & bins(:,1) <= gb.mapSize(2) & bins(:,2) >= 1 & bins(:,2) <= gb.mapSize(1);
    target = unique(sub2ind(gb.mapSize, bins(valid,2), bins(valid,1)));
    o = b.diagnostics.offGround; maps = o.columnMaps; stats = maps.statistics; pid = double(stats.pillarIndices);
    % The off-ground raster is cropped to its occupied bounding box: map cells by centre.
    [tr, tc] = ind2sub(gb.mapSize, target); tCentre = gb.origin + ([tc tr] - 0.5) .* gb.cellSize;
    oBins = floor((tCentre - maps.origin) ./ [maps.dx maps.dy]) + 1;
    inside = oBins(:,1) >= 1 & oBins(:,1) <= maps.mapSize(2) & oBins(:,2) >= 1 & oBins(:,2) <= maps.mapSize(1);
    local = zeros(size(target)); local(inside) = sub2ind(maps.mapSize, oBins(inside,2), oBins(inside,1));
    pole = cfg.offGroundFeatures.pole;
    detected = double(b.candidates.pillarIndices{b.candidates.semanticNames == "pole"});
    fprintf('%6s %7s %7s %5s %6s %6s %6s %6s %6s %6s %5s %5s %6s %6s\n', 'cell', 'x', 'y', 'n', 'h', 'hStd', 'tilt', 'rStd', 'line', 'point', 'core', 'foot', 'ratio', 'found');
    for t = 1:numel(target)
        c = target(t); l = local(t);
        j = find(pid == l, 1); if isempty(j) || l == 0, fprintf('%6d empty\n', c); continue; end
        cv = stats.covarianceXYZ(j, :); slope = cv(4:5) ./ max(cv(6), eps);
        rStd = sqrt(max(0, cv(1) + cv(3) - sum(cv(4:5).^2) / max(cv(6), eps)));
        h = stats.maximumXYZ(j,3) - stats.minimumXYZ(j,3);
        fprintf('%6d %7.2f %7.2f %5d %6.2f %6.2f %6.1f %6.3f %6.2f %6.2f %5d %5d %6.2f %6d\n', c, stats.meanXYZ(j,1), stats.meanXYZ(j,2), ...
            stats.count(j), h, sqrt(cv(6)), atand(norm(slope)), rStd, maps.lineScore(l), maps.pointScore(l), ...
            o.candidates.coreMask(l), o.candidates.footprintCandidateMask(l), o.candidates.contextFraction(l), ismember(c, detected));
    end
    fprintf('gates: n>=%d h>=%.2f hStd>=%.2f tilt<=%g rStd<=%.2f line<=%.2f point>=%.2f context>%.2f\n', pole.minimumPoints, ...
        pole.minimumHeight, pole.minimumHeightStd, pole.maximumTiltDegrees, pole.maximumRadialStd, pole.maximumLineScore, pole.minimumPointScore, pole.minimumContextFraction);
end
