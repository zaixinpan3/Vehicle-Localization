function curbBandThickness(labels, root)
% curbBandThickness: Distribution of curb cells per x-column and road side, per sequence.
    if nargin < 2, root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    out = fullfile(root, 'output', 'coarse_lattice_20260924');
    for label = string(labels)
        s = load(fullfile(out, label + "_sequence.mat"), 'records', 'geometry'); g = s.geometry;
        counts = [];
        for k = 1:numel(s.records)
            ids = double(s.records{k}.pillarIndices{s.records{k}.semanticNames == "curb"});
            [row, col] = ind2sub(g.mapSize, ids); y = g.origin(2) + (row - 0.5) * g.cellSize(2);
            for side = [-1 1]
                sel = sign(y) == side; if ~any(sel), continue; end
                counts = [counts; accumarray(col(sel), 1)]; %#ok<AGROW>
            end
        end
        counts = counts(counts > 0);
        fprintf('%s (%.1f m): columns %d, cells/column mean %.2f, median %d, p90 %d, share>2: %.2f, metres mean %.2f\n', label, g.cellSize(1), ...
            numel(counts), mean(counts), median(counts), quantile(counts, 0.9), mean(counts > 2), mean(counts) * g.cellSize(2));
    end
end
