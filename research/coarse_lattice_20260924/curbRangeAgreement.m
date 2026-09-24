function curbRangeAgreement(labels, root)
% curbRangeAgreement: Curb cell precision/recall (exact and one-cell tolerant) by range bin.
    if nargin < 2, root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    out = fullfile(root, 'output', 'coarse_lattice_20260924');
    base = load(fullfile(out, 'baseline_sequence.mat'), 'records', 'geometry'); bf = cellfun(@(r) r.frame, base.records);
    edges = [0 10 20 30.5];
    for label = string(labels)
        c = load(fullfile(out, label + "_sequence.mat"), 'records', 'geometry'); g = c.geometry; cf = cellfun(@(r) r.frame, c.records);
        tp = zeros(1, 3); fp = tp; fn = tp; tpT = tp; fpT = tp; fnT = tp;
        for k = 1:numel(c.records)
            b = base.records{bf == cf(k)}; r = c.records{k};
            bIds = project(double(b.pillarIndices{b.semanticNames == "curb"}), base.geometry, g);
            cIds = double(r.pillarIndices{r.semanticNames == "curb"});
            [bx, by] = centres(bIds, g); [cx, cy] = centres(cIds, g);
            bBin = discretize(hypot(bx, by), edges); cBin = discretize(hypot(cx, cy), edges);
            bD = dilate(bIds, g); cD = dilate(cIds, g);
            for j = 1:3
                bj = bIds(bBin == j); cj = cIds(cBin == j);
                tp(j) = tp(j) + numel(intersect(bj, cj)); fp(j) = fp(j) + nnz(~ismember(cj, bIds)); fn(j) = fn(j) + nnz(~ismember(bj, cIds));
                fpT(j) = fpT(j) + nnz(~ismember(cj, bD)); fnT(j) = fnT(j) + nnz(~ismember(bj, cD));
                tpT(j) = tpT(j) + nnz(ismember(cj, bD));
            end
        end
        fprintf('%s\n%8s %8s %8s %6s %6s | %6s %6s\n', label, 'range', 'base', 'cand', 'prec', 'rec', 'precT', 'recT');
        for j = 1:3
            nb = tp(j) + fn(j); nc = tp(j) + fp(j);
            fprintf('%3d-%3d m %8d %8d %6.3f %6.3f | %6.3f %6.3f\n', edges(j), floor(edges(j+1)), nb, nc, tp(j)/max(nc,1), tp(j)/max(nb,1), 1 - fpT(j)/max(nc,1), 1 - fnT(j)/max(nb,1));
        end
    end
end
function ids = project(sourceIds, sg, tg)
    [row, col] = ind2sub(sg.mapSize, sourceIds); centre = sg.origin + ([col row] - 0.5) .* sg.cellSize;
    bins = floor((centre - tg.origin) ./ tg.cellSize) + 1;
    valid = bins(:,1) >= 1 & bins(:,1) <= tg.mapSize(2) & bins(:,2) >= 1 & bins(:,2) <= tg.mapSize(1);
    ids = unique(sub2ind(tg.mapSize, bins(valid,2), bins(valid,1)));
end
function ids = dilate(sourceIds, g)
    if isempty(sourceIds), ids = zeros(0,1); return; end
    [row, col] = ind2sub(g.mapSize, sourceIds); [dr, dc] = meshgrid(-1:1, -1:1);
    rows = row + dr(:).'; cols = col + dc(:).';
    valid = rows >= 1 & rows <= g.mapSize(1) & cols >= 1 & cols <= g.mapSize(2);
    ids = unique(sub2ind(g.mapSize, rows(valid), cols(valid)));
end
function [x, y] = centres(ids, g)
    [row, col] = ind2sub(g.mapSize, ids); x = g.origin(1) + (col - 0.5) * g.cellSize(1); y = g.origin(2) + (row - 0.5) * g.cellSize(2);
end
