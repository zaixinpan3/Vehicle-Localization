function fine = refinePerceptionCandidates(frame, candidates, context, cfg)
% refinePerceptionCandidates: Recover and validate only coarse candidate
% members for offline mapping. Every recovered point has an explicit decision;
% an empty accepted set stays empty. No full-frame detector is rerun.
    n = numel(frame.x);
    grid = context.voxelGrid;
    ground = context.ground;
    gc = context.groundContext;
    xyz = double([frame.x(:), frame.y(:), frame.z(:)]);
    groundPoint = false(n, 1);
    groundPoint(gc.groundOriginalPointIdx) = true;
    masks = struct("groundPoint", groundPoint, "curb", false(n, 1), ...
        "roadMarking", false(n, 1), "pole", false(n, 1));
    decisions = struct();
    for k = 1:numel(candidates.semanticNames)
        name = candidates.semanticNames(k);
        inCandidate = ismember(grid.pointPillarLinIdx, candidates.pillarIndices{k});
        pointIdx = double(grid.pointIndices(inCandidate));
        if name == "pole"
            pointIdx = pointIdx(~groundPoint(pointIdx));
        else
            pointIdx = pointIdx(groundPoint(pointIdx));
        end
        points = xyz(pointIdx, :);
        accepted = false(numel(pointIdx), 1);
        switch name
            case "curb"
                [~, groundRows] = ismember(pointIdx, gc.groundOriginalPointIdx);
                cellIdx = gc.groundCellLinIdx(groundRows);
                accepted = selectCurbPointsFromCells(true(numel(pointIdx), 1), ...
                    points, cellIdx, ground.stats, ground.energyMaps, ...
                    ground.initialRoadResult.roadCellMask, cfg.groundFeatures.curb);
                accepted = thinCurbPointsToDominantBoundary(accepted, points, ...
                    cellIdx, gc.groundXYView, ground.initialRoadResult.roadSeedMask, ...
                    ground.initialRoadResult.roadCellMask, ground.energyMaps, cfg.groundFeatures.curb);
            case "roadMarking"
                [~, groundRows] = ismember(pointIdx, gc.groundOriginalPointIdx);
                reflectivity = double(gc.groundReflectivity(groundRows));
                accepted = isfinite(reflectivity) & reflectivity > candidates.groundReflectivityThreshold;
            case "pole"
                accepted = validatePolePoints(points, cfg.fine, context.offGround, context.offGroundVoxelGrid.gridConfig);
        end
        masks.(name)(pointIdx(accepted)) = true;
        decisions.(name) = struct("candidatePointIndices", pointIdx, ...
            "evaluatedPointIndices", pointIdx, "accepted", accepted, ...
            "numEvaluated", numel(pointIdx), "numAccepted", nnz(accepted));
    end
    fine = struct("featureMasks", masks, "refinement", decisions);
end

function accepted = validatePolePoints(points, cfg, offGround, geometry)
% validatePolePoints: Fit a vertical line to each connected candidate group,
% then test every member against metric tilt, height and robust radial limits.
    accepted = false(size(points, 1), 1);
    if isempty(points)
        return;
    end
    bins = floor((points(:, 1:2) - geometry.minCorner(1:2)) ./ geometry.voxelSize(1:2)) + 1;
    mapSize = geometry.dims([2, 1]);
    cellIdx = sub2ind(mapSize, bins(:, 2), bins(:, 1));
    mask = false(mapSize); mask(cellIdx) = true;
    labels = bwlabel(mask, 8);
    group = labels(cellIdx);
    for component = unique(group).'
        rows = find(group == component);
        p = points(rows, :);
        if size(p, 1) < cfg.poleMinimumPoints || (max(p(:, 3)) - min(p(:, 3))) < cfg.poleMinimumHeight
            continue;
        end
        % Fine decisions use neighboring voxel counts, never neighboring raw points.
        columns = unique(cellIdx(rows));
        maps = offGround.columnMaps;
        [r, c] = ind2sub(mapSize, columns);
        voxels = maps.voxelStatistics;
        [vr, vc] = ind2sub(mapSize, double(voxels.columnLinIdx));
        inNeighborhood = vr >= min(r)-1 & vr <= max(r)+1 & vc >= min(c)-1 & vc <= max(c)+1;
        inObject = ismember(voxels.columnLinIdx, columns);
        nz = geometry.dims(3);
        objectCounts = accumarray(double(voxels.zBin(inObject)), double(voxels.count(inObject)), [nz 1], @sum, 0);
        neighborCounts = accumarray(double(voxels.zBin(inNeighborhood)), double(voxels.count(inNeighborhood)), [nz 1], @sum, 0);
        ratio = objectCounts ./ max(neighborCounts, 1);
        qualified = objectCounts >= maps.occupiedLayerMinPoints & ratio > cfg.poleMinimumSliceRatio;
        if nnz(qualified)*geometry.voxelSize(3) < cfg.poleMinimumSupportedHeight || ...
                mean(ratio(qualified)) < cfg.poleMinimumSupportRatio || ...
                sum(objectCounts(qualified))/max(sum(neighborCounts(qualified)),1) < cfg.poleMinimumSupportRatio
            continue;
        end
        zBin = floor((p(:,3)-geometry.minCorner(3))/geometry.voxelSize(3))+1;
        supported = qualified(zBin);
        if nnz(supported) < cfg.poleMinimumPoints
            continue;
        end
        z = p(:, 3) - median(p(supported, 3));
        design = [ones(size(z)), z];
        coefficients = design(supported,:) \ p(supported, 1:2);
        if norm(coefficients(2, :)) > tand(cfg.poleMaximumTiltDegrees)
            continue;
        end
        residual = vecnorm(p(:, 1:2) - design * coefficients, 2, 2);
        mid = median(residual(supported));
        sigma = max(1.4826 * median(abs(residual(supported) - mid)), cfg.minimumResidualScale);
        limit = min(cfg.poleMaximumRadius, mid + cfg.robustScale * sigma);
        accepted(rows) = supported & residual <= limit;
    end
end
