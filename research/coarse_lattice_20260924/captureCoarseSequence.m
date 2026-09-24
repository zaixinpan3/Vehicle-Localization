function captureCoarseSequence(label, frames, root, modify)
% captureCoarseSequence: Save compact coarse perception outputs of Mississippi frames.
% Records candidate pillar IDs, road/curb/pole/sign cell evidence, Gaussian
% components and timing for the perception code currently on the path.
    if nargin < 3 || isempty(root), root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    out = fullfile(root, 'output', 'coarse_lattice_20260924');
    fprintf('Using %s\n', which('perceiveFrame'));
    cfg = perceptionConfig();
    cfg.executionBackend = "auto";
    if nargin >= 4 && ~isempty(modify), cfg = modify(cfg); end
    store = matfile(fullfile(root, 'data', 'raw', 'MissisipiPointClouds.mat'));
    n = numel(frames); records = cell(n, 1);
    geometry = [];
    for first = 1:50:n
        ids = first:min(n, first+49);
        block = store.pointClouds(1, frames(ids));
        for k = ids
            frame = block(k-first+1);
            timer = tic; [cloud, diagnostics] = perceiveCoarseProbabilityCloud(frame, cfg); elapsed = toc(timer);
            % Diagnostics only: re-run through perceiveFrame for candidates.
            perception = perceiveFrame(frame, cfg);
            if isempty(geometry), geometry = perception.candidates.geometry; end
            r = struct('frame', frames(k), 'elapsed', elapsed);
            r.pillarIndices = perception.candidates.pillarIndices;
            r.semanticNames = perception.candidates.semanticNames;
            g = diagnostics.ground; o = diagnostics.offGround;
            r.roadCells = fullLatticeIndices(g.roadCellMask, g.pillarOffset, geometry);
            r.curbCells = fullLatticeIndices(g.curbCellMask, g.pillarOffset, geometry);
            r.curbEnergy = single(g.energyMaps.total(g.curbCellMask));
            r.curbBaseEnergy = single(g.energyMaps.totalBase(g.curbCellMask));
            r.groundCells = fullLatticeIndices(g.stats.countMap > 0, g.pillarOffset, geometry);
            [r.poleCells, r.poleProbability] = columnCells(o, "pole", geometry);
            [r.signCells, r.signProbability] = columnCells(o, "trafficSign", geometry);
            c = cloud.components;
            r.components = struct('semanticId', c.semanticId, 'count', c.count, 'mean', c.mean, ...
                'covariance', c.covariance, 'meanXYZ', c.meanXYZ, 'mixtureWeight', c.mixtureWeight, ...
                'semanticProbability', c.semanticProbability, 'occupancyProbability', c.occupancyProbability);
            r.cloudSemanticNames = cloud.semanticNames;
            r.layerWeights = [cloud.layers.mixtureWeight];
            r.sourceSummary = perception.sourceSummary;
            records{k} = r;
        end
        fprintf('%s %d/%d\n', label, ids(end), n);
    end
    cloudGeometry = cloud.geometry;
    save(fullfile(out, sprintf('%s_sequence.mat', label)), 'records', 'geometry', 'cloudGeometry', 'cfg', '-v7.3');
    fprintf('Saved %s: median %.1f ms\n', label, 1000*median(cellfun(@(r) r.elapsed, records)));
end

function ids = fullLatticeIndices(mask, offset, geometry)
    [row, col] = find(mask);
    bins = [col(:)+offset(1), row(:)+offset(2)];
    valid = bins(:,1)>=1 & bins(:,1)<=geometry.mapSize(2) & bins(:,2)>=1 & bins(:,2)<=geometry.mapSize(1);
    ids = int32(sub2ind(geometry.mapSize, bins(valid,2), bins(valid,1)));
end

function [ids, probability] = columnCells(offGround, name, geometry)
    mask = offGround.(name+"CellMask");
    [row, col] = find(mask);
    maps = offGround.columnMaps;
    centers = maps.origin + ([col(:) row(:)]-0.5).*[maps.dx maps.dy];
    bins = floor((centers-geometry.origin)./geometry.cellSize)+1;
    valid = bins(:,1)>=1 & bins(:,1)<=geometry.mapSize(2) & bins(:,2)>=1 & bins(:,2)<=geometry.mapSize(1);
    ids = int32(sub2ind(geometry.mapSize, bins(valid,2), bins(valid,1)));
    p = offGround.(name+"Probability");
    probability = single(p(mask)); probability = probability(valid);
end
