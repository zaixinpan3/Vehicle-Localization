function surface = detectFacadeSurfaces(pillars, zReference, cfg, candidatePillars, supportPillars)
% detectFacadeSurfaces: Sparse wall seeds and anchored point-plane support.
% Offline only: use the coarse candidate envelope, local 3D patches and connected
% MSAC plane inliers. Seeds are immutable: new wall points never establish
% new anchors. No fine XY grid, dense 3D tensor, external model or reference
% labels are used. The point decisions also supply offline mapping masks.
%
% Input: pillars: nonground pillar context; zReference: shared height phase;
%        cfg: facadeSurfaceConfig for the pillar spacing;
%        candidatePillars: immutable label eligibility;
%        supportPillars: optional bounded neighborhood context, containing
%            candidates (default: candidates). Context cannot receive labels.
% Output: surface: pillar mask/line ownership, point decisions, candidate
%         indices and supporting plane diagnostics on the original points.
    geometry = pillars.pillarGeometry;
    xyz = double(pillars.points);
    ids = double(pillars.pointPillarLinIdx);
    if nargin<4,candidatePillars=true(geometry.mapSize);end
    if nargin<5,supportPillars=candidatePillars;end
    assert(isequal(size(candidatePillars),geometry.mapSize));
    assert(isequal(size(supportPillars),geometry.mapSize));
    assert(~any(candidatePillars & ~supportPillars,'all'));
    eligible = true(size(ids));
    if isfield(pillars.pointAttributes,'intensity')
        intensity = double(pillars.pointAttributes.intensity(:));
        eligible = ~(isfinite(intensity) & intensity>cfg.trafficSignIntensityThreshold);
    end
    [maps, samples] = heightSupport(xyz, ids, eligible, geometry, zReference, cfg);
    [~, maps.lineScore, maps.normalOrientation] = buildPillarShapeScores( ...
        maps.supportEvidence,maps.occupiedMask,cfg.hough,maps.dx,maps.dy);
    proposals = extractFacadeFeatures(maps,cfg.hough);
    seedMap = validateSeedPatches(proposals.lineMap,proposals.detectedLines,samples,maps,cfg);
    % Neighboring walls may veto a pole without receiving facade labels.
    contextSeedMap = seedMap;
    seedMap(~supportPillars)=0;
    seedIds = seedMap(ids);
    seedIds(~eligible) = 0;
    eligible = eligible & supportPillars(ids);
    [pointLineIds, detail] = completeFacadeSurfaces(xyz,ids,seedIds,eligible, ...
        proposals.detectedLines,cfg.completion);
    % Neighboring plane inliers supply shape/overlap context; only immutable
    % candidate members receive the public facade decision.
    contextPlaneSupportIds=pointLineIds;
    pointLineIds(~candidatePillars(ids))=0;
    detail.candidateMask=detail.candidateMask & candidatePillars(ids);
    accepted = pointLineIds>0;
    lineMap = zeros(geometry.mapSize,'uint16');
    lineMap(ids(accepted)) = pointLineIds(accepted);
    count = accumarray(ids(eligible),1,[prod(geometry.mapSize),1]);
    support = accumarray(ids(accepted),1,[prod(geometry.mapSize),1]);
    surface = struct('mask',lineMap>0,'lineMap',lineMap, ...
        'detectedLines',proposals.detectedLines,'seedLineMap',seedMap, ...
        'fullContextSeedLineMap',contextSeedMap,'lineScore',maps.lineScore, ...
        'pointIndices',double(pillars.pointIndices),'pointLineIds',pointLineIds, ...
        'contextPlaneSupportIds',contextPlaneSupportIds, ...
        'candidateMask',detail.candidateMask,'planeValidation',detail, ...
        'supportFraction',reshape(support./max(count,1),geometry.mapSize));
end

function [maps,samples] = heightSupport(xyz,ids,eligible,geometry,zReference,cfg)
% Reduce occupied height samples without allocating a 3D volume.
    dims = double(geometry.mapSize); number = prod(dims);
    zBin = floor((xyz(eligible,3)-double(zReference))/cfg.heightResolution)+1;
    key = ids(eligible)+(zBin-1)*number;
    [uniqueKey,~,membership] = unique(key);
    counts = accumarray(membership,1,[numel(uniqueKey),1]);
    columns = mod(uniqueKey-1,number)+1;
    heights = floor((uniqueKey-1)/number)+1;
    run = zeros(number,1); layerCount = zeros(number,1); height = zeros(number,1);
    if ~isempty(columns)
        pairs = sortrows([columns,heights],[1 2]);
        starts = find([true;diff(pairs(:,1))~=0 | diff(pairs(:,2))~=1]);
        run = accumarray(pairs(starts,1),diff([starts;size(pairs,1)+1]),[number 1],@max,0);
        layerCount = accumarray(columns,1,[number 1]);
        low = accumarray(columns,heights,[number 1],@min,NaN);
        high = accumarray(columns,heights,[number 1],@max,NaN);
        height = (high-low)*cfg.heightResolution;
    end
    [rows,cols] = ind2sub(dims,columns);
    samples = struct('rows',rows,'cols',cols,'heights',heights,'weights',counts, ...
        'columns',columns,'zReference',double(zReference));
    [xMap,yMap] = meshgrid(single(geometry.origin(1)+((1:dims(2))-.5)*geometry.cellSize(1)), ...
        single(geometry.origin(2)+((1:dims(1))-.5)*geometry.cellSize(2)));
    maps = struct('mapSize',dims,'origin',geometry.origin,'dx',geometry.cellSize(1), ...
        'dy',geometry.cellSize(2),'xMap',xMap,'yMap',yMap, ...
        'occupiedMask',reshape(layerCount>0,dims),'supportEvidence',reshape(single(run),dims), ...
        'columnHeight',reshape(height,dims),'columnLayers',reshape(layerCount,dims));
end

function refined = validateSeedPatches(lineMap,lines,samples,maps,cfg)
% Preserve tuned patch origin, sample weighting and strict height gates.
    refined = zeros(size(lineMap),'uint16'); bestScore = -inf(size(lineMap));
    radius = round(cfg.seed.supportRadiusMeters/max(maps.dx,maps.dy));
    patchSize = max(1,round(cfg.seed.patchSizeMeters./[maps.dy,maps.dx,cfg.heightResolution]));
    for lineId = 1:size(lines,1)
        seed = lineMap==lineId;
        if ~any(seed,'all'), continue; end
        support = imdilate(seed,true(2*radius+1));
        selected = find(support(samples.columns));
        if isempty(selected), continue; end
        row = samples.rows(selected); col = samples.cols(selected); z = samples.heights(selected);
        bin = floor(([row,col,z]-min([row,col,z],[],1))./patchSize);
        sizes = max(bin,[],1)+1;
        patchIds = bin(:,1)+bin(:,2)*sizes(1)+bin(:,3)*prod(sizes(1:2))+1;
        [~,~,membership] = unique(patchIds,'stable');
        tangent = lines(lineId,3:4)-lines(lineId,1:2);
        normal = [-tangent(2),tangent(1)]/max(norm(tangent),eps);
        for patchId = 1:max(membership)
            members = selected(membership==patchId); weights = samples.weights(members);
            if numel(members)<cfg.seed.minimumVoxels || sum(weights)<cfg.seed.minimumPoints, continue; end
            centers = [maps.origin(1)+(samples.cols(members)-.5)*maps.dx, ...
                maps.origin(2)+(samples.rows(members)-.5)*maps.dy, ...
                samples.zReference+(samples.heights(members)-.5)*cfg.heightResolution];
            if max(centers(:,3))-min(centers(:,3))<cfg.seed.minimumPatchHeight, continue; end
            center = sum(centers.*weights,1)/sum(weights);
            residual = centers-center;
            covariance = (residual.*weights).'*residual/sum(weights);
            [vectors,values] = eig((covariance+covariance.')/2,'vector');
            [values,order] = sort(values,'descend');
            planarity = (max(values(2),0)-max(values(3),0))/max(values(1),eps);
            direction = vectors(:,order(3));
            tilt = asind(min(1,abs(direction(3))));
            alignment = abs(direction(1:2).'*normal.')/max(norm(direction(1:2)),eps);
            angle = acosd(min(1,alignment));
            if planarity<cfg.seed.minimumPlanarity || angle>cfg.seed.maximumNormalAngleDegrees || ...
                    tilt>cfg.seed.maximumNormalTiltDegrees, continue; end
            columns = unique(samples.columns(members));
            score = planarity*max(0,1-angle/cfg.seed.maximumNormalAngleDegrees);
            update = columns(score>bestScore(columns));
            refined(update) = uint16(lineId); bestScore(update) = score;
        end
    end
    columnSupport = maps.columnHeight>=cfg.seed.minimumColumnHeight & ...
        maps.columnLayers>=cfg.seed.minimumColumnLayers;
    refined(~columnSupport) = 0;
end
