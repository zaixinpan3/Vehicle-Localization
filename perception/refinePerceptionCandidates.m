function fine = refinePerceptionCandidates(frame, candidates, context, cfg)
% refinePerceptionCandidates: Fine point classification for offline mapping.
% Ground candidates reuse the common road raster. Structural candidates are
% independently reconstructed offline; coarse misses cannot suppress fine
% detection. Each fine candidate member receives an explicit point decision.
    n = numel(frame.x);
    grid = context.voxelGrid;
    ground = context.ground;
    gc = context.groundContext;
    xyz = double([frame.x(:), frame.y(:), frame.z(:)]);
    groundPoint = false(n, 1);
    groundPoint(gc.groundOriginalPointIdx) = true;
    masks = struct("groundPoint", groundPoint, "curb", false(n, 1), ...
        "pole", false(n, 1), ...
        "facade", false(n,1), "trafficSign", false(n,1));
    decisions = struct();
    if any(ismember(candidates.semanticNames,["pole","facade","trafficSign"])) && isfield(context,'offGroundVoxelGrid')
        [context,candidates]=prepareFineStructuralCandidates(frame,context,candidates,cfg);
    end
    for k = 1:numel(candidates.semanticNames)
        name = candidates.semanticNames(k);
        inCandidate = ismember(grid.pointPillarLinIdx, candidates.pillarIndices{k});
        pointIdx = double(grid.pointIndices(inCandidate));
        if ismember(name,["pole","facade","trafficSign"])
            pointIdx = pointIdx(~groundPoint(pointIdx));
        else
            pointIdx = pointIdx(groundPoint(pointIdx));
        end
        points = xyz(pointIdx, :);
        accepted = false(numel(pointIdx), 1);
        switch name
            case "curb"
                radius = cfg.fine.curbCandidateRadiusCells;
                support = conv2(double(ground.curbCellMask),ones(2*radius+1),'same')>0;
                support = support.';
                pointIdx = gc.groundOriginalPointIdx(support(gc.groundCellLinIdx));
                [accepted,curbDetail] = refineCurbGeometry(xyz,pointIdx,groundPoint,cfg.fine);
                [continued,evaluated,boundaries]=extendCurbBoundaries(xyz,groundPoint,pointIdx,curbDetail.boundaryPointIndices,cfg.fine);
                % Guided continuation may revisit previously rejected points.
                selected=union(pointIdx(accepted),continued);
                pointIdx=union(pointIdx,evaluated);accepted=ismember(pointIdx,selected);
                curbDetail.continuationPointIndices=continued;
                curbDetail.boundaryPointIndices=[curbDetail.boundaryPointIndices,boundaries];
                candidateMembers = ismember(grid.pointIndices,pointIdx);
                candidates.pillarIndices{k} = unique(grid.pointPillarLinIdx(candidateMembers));
            case "facade"
                accepted = validateFacadeCandidatePoints(points, context.offGround, cfg.fine);
            case "trafficSign"
                if isfield(frame,"intensity")
                    intensity = double(frame.intensity(pointIdx));
                    accepted = isfinite(intensity(:)) & intensity(:) > cfg.offGroundFeatures.trafficSignIntensityThreshold;
                end
            case "pole"
                base=true(numel(pointIdx),1);
                if isfield(candidates,'basePolePillarIndices')
                    members=ismember(grid.pointPillarLinIdx,candidates.basePolePillarIndices);
                    base=ismember(pointIdx,double(grid.pointIndices(members)));
                end
                accepted(base)=validatePolePoints(points(base,:),cfg.fine,context.offGround,context.offGroundVoxelGrid.gridConfig,context.offGroundVoxelGrid.points);
                accepted(~base)=validatePolePoints(points(~base,:),cfg.fine,context.offGround,context.offGroundVoxelGrid.gridConfig,context.offGroundVoxelGrid.points);
        end
        masks.(name)(pointIdx(accepted)) = true;
        decisions.(name) = struct("candidatePointIndices", pointIdx, ...
            "evaluatedPointIndices", pointIdx, "accepted", accepted, ...
            "numEvaluated", numel(pointIdx), "numAccepted", nnz(accepted));
        if name=="curb", decisions.curb.geometry=curbDetail; end
    end
    fine = struct("featureMasks", masks, "refinement", decisions, "candidates", candidates);
end

function accepted = validatePolePoints(points, cfg, offGround, geometry, neighborhoodPoints)
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
        % Fine voxel support first qualifies the shaft's vertical extent.
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
        % Short vertical support must also be tightly concentrated about the
        % fitted axis. Test the whole supported object before trimming points.
        radialRms = sqrt(mean(residual(supported).^2));
        % A broad, elongated transverse surface can fit an upright line but
        % represents a trunk arc or wall strip, not a compact shaft. Measure
        % all supported returns before robust point trimming.
        transverse=p(supported,1:2)-design(supported,:)*coefficients;
        spread=sort(eig(cov(transverse)));
        if spread(2)>cfg.poleWideSurfaceMinimumAxisStd^2 && ...
                spread(2)>cfg.poleWideSurfaceMinimumAspectRatio^2*max(spread(1),eps)
            continue;
        end
        supportHeight = nnz(qualified)*geometry.voxelSize(3);
        shortSupport = supportHeight < cfg.poleShortSupportHeight;
        % At the boundary, weakly separated objects also need a tight shaft.
        weakBoundary = supportHeight <= cfg.poleShortSupportHeight && ...
            mean(ratio(qualified)) < cfg.poleLowContrastSupportRatio;
        if (shortSupport || weakBoundary) && radialRms > cfg.poleShortSupportMaximumRadialRms
            continue;
        end
        % A short supported shaft must be isolated even if it dominates its
        % immediate cells. Longer weak-contrast objects use the same check.
        if (mean(ratio(qualified)) < cfg.poleLowContrastSupportRatio || ...
                supportHeight <= cfg.poleShortSupportHeight) && ...
                ~validatePoleIsolation(neighborhoodPoints, ...
                [coefficients(1,:),median(p(supported,3))],coefficients(2,:),qualified,geometry,cfg)
            continue;
        end
        mid = median(residual(supported));
        sigma = max(1.4826 * median(abs(residual(supported) - mid)), cfg.minimumResidualScale);
        limit = min(cfg.poleMaximumRadius, mid + cfg.robustScale * sigma);
        % Weak separation from neighboring returns requires each accepted
        % point to lie closer to the fitted axis; retain the narrow core.
        if mean(ratio(qualified)) < cfg.poleLowContrastSupportRatio
            limit = min(limit,cfg.poleLowContrastMaximumRadius);
        end
        % Establish continuity from actual axis-consistent returns, including
        % sparse cells that do not independently qualify for output labeling.
        connected=residual<=limit;
        connected(connected)=retainContinuousPoleSupport(p(connected,3),cfg);
        accepted(rows)=supported & connected;
    end
end

function [context,candidates]=prepareFineStructuralCandidates(frame,context,candidates,cfg)
% prepareFineStructuralCandidates: Rebuild detailed geometry exclusively offline.
% Fine detection is independent of coarse candidate recall. Existing detailed
% object support and per-point tests remain available for high-quality mapping.
    source=context.offGroundVoxelGrid;
    input=struct('x',source.points(:,1),'y',source.points(:,2),'z',source.points(:,3), ...
        'pointIndices',source.pointIndices);
    attributes=fieldnames(source.pointAttributes);
    for k=1:numel(attributes), input.(attributes{k})=source.pointAttributes.(attributes{k}); end
    base=0;
    if ~isempty(context.voxelGrid.points), base=min(context.voxelGrid.points(:,3)); end
    voxelCfg=fineVoxelizationConfig();
    voxelCfg.voxelSize=[source.pillarGeometry.cellSize,cfg.fine.poleSupportHeightResolution];
    voxelCfg.roiLimits=[]; voxelCfg.exclusionHalfSize=0;
    voxelCfg.minCorner=[source.pillarGeometry.origin,base];

    fineGrid=voxelizePointCloud(input,voxelCfg);
    fineCfg=fineStructuralConfig();
    common=intersect(fieldnames(fineCfg),fieldnames(cfg.offGroundFeatures));
    for k=1:numel(common), fineCfg.(common{k})=cfg.offGroundFeatures.(common{k}); end
    fineCfg.useNativeKernels=isfield(cfg.offGroundFeatures,'useNativeKernels') && cfg.offGroundFeatures.useNativeKernels;
    fineCfg.poleOccupiedLayerMinPoints=cfg.fine.poleSupportMinimumPoints;
    cloudCfg=coarseSemanticProbabilityCloudConfig();
    cloudCfg.semanticNames=candidates.semanticNames;
    offGround=analyzeFineStructuralCandidates(fineGrid,fineCfg,cloudCfg);
    fineCandidates=buildPerceptionCandidates(context.voxelGrid,context.ground,offGround,candidates.semanticNames);
    if cfg.fine.poleRecoveryEnabled && any(candidates.semanticNames=="pole")
        % Add only strong whole-pillar 3D evidence missed by detailed seeding.
        maps=context.offGround.columnMaps;
        detailed=offGround.columnMaps;
        vox=detailed.voxelStatistics;
        fineSupport=struct('sparseVoxelColumnLinIdx',vox.columnLinIdx, ...
            'sparseVoxelZBin',vox.zBin,'sparseVoxelCount',vox.count, ...
            'sparseMapSize',detailed.mapSize,'sparseNumZLayers',fineGrid.gridConfig.dims(3), ...
            'occupiedLayerMinPoints',detailed.occupiedLayerMinPoints);
        relaxedParams=offGround.poleParams;
        relaxedParams.coreMinRunLayerThreshold=3;
        relaxed=detectPoleCandidates(detailed,offGround.facade.mask,fineSupport,relaxedParams);
        stats=maps.statistics;
        covar=stats.covarianceXYZ;
        slope=covar(:,4:5)./max(covar(:,6),eps);
        radial=sqrt(max(0,covar(:,1)+covar(:,3)-sum(covar(:,4:5).^2,2)./max(covar(:,6),eps)));
        recover=stats.count>=cfg.fine.poleRecoveryMinimumPoints & ...
            stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3)>=cfg.fine.poleRecoveryMinimumHeight & ...
            vecnorm(slope,2,2)<=tand(cfg.fine.poleRecoveryMaximumTiltDegrees) & ...
            radial<=cfg.fine.poleRecoveryMaximumRadialStd & ...
            context.offGround.poleCellMask(double(stats.pillarIndices));
        [rows,cols]=ind2sub(maps.mapSize,double(stats.pillarIndices(recover)));
        xy=maps.origin+([cols(:) rows(:)]-0.5).*[maps.dx maps.dy];
        geometry=context.voxelGrid.pillarGeometry;
        fineBins=floor((xy-detailed.origin)./[detailed.dx detailed.dy])+1;
        fineIds=sub2ind(detailed.mapSize,fineBins(:,2),fineBins(:,1));
        % A long, tightly upright whole-pillar axis can bridge sparse detailed
        % support. Ordinary recovery retains its existing seed evidence.
        sparseAxis=stats.maximumXYZ(recover,3)-stats.minimumXYZ(recover,3)>=cfg.fine.poleSparseRecoveryMinimumHeight & ...
            vecnorm(slope(recover,:),2,2)<=tand(cfg.fine.poleSparseRecoveryMaximumTiltDegrees) & ...
            radial(recover)<=cfg.fine.poleSparseRecoveryMaximumRadialRms;
        seedIds=double(stats.pillarIndices(recover));
        seedIds=seedIds(relaxed.candidateMask(fineIds) | sparseAxis);
        completed=completeRecoveredPoleShafts(stats,maps.mapSize,seedIds,cfg.fine);
        % Complete an existing detailed seed only when it is a small edge
        % fragment beside a denser, upright shaft. Keep ordinary seeds intact.
        [sr,sc]=ind2sub(maps.mapSize,double(stats.pillarIndices));
        centers=maps.origin+([sc(:) sr(:)]-0.5).*[maps.dx maps.dy];
        sb=floor((centers-detailed.origin)./[detailed.dx detailed.dy])+1;
        sid=sub2ind(detailed.mapSize,sb(:,2),sb(:,1));
        strong=stats.count>=cfg.fine.poleRecoveryMinimumPoints & ...
            stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3)>=cfg.fine.poleRecoveryMinimumHeight & ...
            vecnorm(slope,2,2)<=tand(cfg.fine.poleRecoveryMaximumTiltDegrees) & ...
            radial<=cfg.fine.poleSparseRecoveryMaximumRadialRms;
        edgeSeeds=find(strong & offGround.poleCellMask(sid));
        splitCompleted=zeros(0,1);
        for seed=edgeSeeds(:).'
            adjacent=strong & abs(sr-sr(seed))<=1 & abs(sc-sc(seed))<=1;
            if ~any(stats.count(adjacent)>=cfg.fine.poleBoundaryMinimumCountRatio*stats.count(seed))
                continue;
            end
            initial=double(stats.pillarIndices(seed));
            joined=completeRecoveredPoleShafts(stats,maps.mapSize,initial,cfg.fine);
            joined=completeRecoveredPoleShafts(stats,maps.mapSize,joined,cfg.fine);
            % Never grow beyond the original seed's immediate XY neighbors.
            local=abs(sr-sr(seed))<=1 & abs(sc-sc(seed))<=1;
            joined=intersect(joined,double(stats.pillarIndices(local)));
            if numel(joined)>1,splitCompleted=union(splitCompleted,joined);end
        end
        completed=union(completed,splitCompleted);
        [rows,cols]=ind2sub(maps.mapSize,completed);
        xy=maps.origin+([cols(:) rows(:)]-0.5).*[maps.dx maps.dy];
        bins=floor((xy-geometry.origin)./geometry.cellSize)+1;
        fineBins=floor((xy-detailed.origin)./[detailed.dx detailed.dy])+1;
        fineIds=sub2ind(detailed.mapSize,fineBins(:,2),fineBins(:,1));
        recovered=int32(sub2ind(geometry.mapSize,bins(:,2),bins(:,1)));
        recovered=recovered(~offGround.facade.mask(fineIds));
        channel=find(candidates.semanticNames=="pole");
        fineCandidates.basePolePillarIndices=fineCandidates.pillarIndices{channel};
        if ~isempty(splitCompleted)
            % Move the seed's entire connected base component with its added
            % neighbors, so validation cannot mistake its own shaft for clutter.
            baseMask=false(geometry.mapSize);
            baseMask(fineCandidates.basePolePillarIndices)=true;
            components=bwlabel(baseMask,8);
            [rr,cc]=ind2sub(maps.mapSize,splitCompleted);
            splitXY=maps.origin+([cc(:) rr(:)]-0.5).*[maps.dx maps.dy];
            splitBins=floor((splitXY-geometry.origin)./geometry.cellSize)+1;
            splitIds=sub2ind(geometry.mapSize,splitBins(:,2),splitBins(:,1));
            touched=unique(components(splitIds));touched=touched(touched>0);
            fineCandidates.basePolePillarIndices=setdiff(fineCandidates.basePolePillarIndices,find(ismember(components,touched)));
        end
        fineCandidates.pillarIndices{channel}=union(fineCandidates.pillarIndices{channel},recovered);
    end
    candidates=fineCandidates;
    candidates.framePointCount=numel(frame.x);
    context.offGround=offGround;
    context.offGroundVoxelGrid=fineGrid;
end
