function [pointLineIds, debug] = completeFacadeSurfaces(xyz, pillarIndices, seedLineIds, eligible, lines, cfg)
% Adapted from RobustVehicleLocalization tunedV12 (2026-10-03).
% ---------------------------------------------------------------------
% completeFacadeSurfaces: Recover raw wall points using connected plane
% components anchored to an immutable set of strictly validated facade
% points. For each seeded Hough line, fit a near-vertical plane in a bounded
% strip with MATLAB MSAC. Optional local fitting windows prevent dense,
% thin strips from dominating the plane estimate. By default each local
% model requires tall, distributed original anchors inside its fit window.
% Panel mode validates each connected component with original anchors over
% the whole component instead; a fitting window only proposes the plane.
% Split inliers into connected cells in the plane's
% horizontal-direction/height coordinates. Accept only large components
% with tall, spatially distributed original anchors. Added points cannot
% seed further completion. Ground and traffic-sign points are ineligible.
% Optional strict seed validation also removes original column labels that
% are not individual inliers of an accepted connected plane component.
% Only individual plane inliers receive new labels, never whole columns.
% An optional tall-wall pass uses the same immutable original anchors and
% stronger height support, with its own bounded plane tolerance. It only
% supplements points rejected by the primary pass. Height windows separate
% upper/lower wall surfaces during fitting without bootstrapping new anchors.
% Optional parallel-line anchor sharing removes artificial Hough ownership
% boundaries. Shared anchors must still be original seeds in the candidate
% strip and accepted connected plane. Optional projected convex-hull area
% gates require two-dimensional wall/anchor support instead of a tall stripe.
% Preserve the caller's random state and use a fixed seed for repeatability.
%
% Input:
%   xyz: [N x 3] finite off-ground coordinates in meters.
%   pillarIndices: [N x 1] original shared-lattice XY column indices.
%   seedLineIds: [N x 1] validated line IDs; zero denotes unassigned.
%   eligible: [N x 1] logical mask excluding traffic-sign points.
%   lines: [L x 4] Hough segments [x1 y1 x2 y2] in meters.
%   cfg: completion thresholds from facadeSurfaceConfig.
%
% Output:
%   pointLineIds: [N x 1] uint16 IDs. Original assignments are preserved
%       unless cfg.requireSeedPlaneValidation requests point-level checking.
%   debug: global models, accepted component counts, and additions per line.
%       allModels and pointModelIds identify the exact global/local plane
%       validating each accepted point. Unchecked original column seeds have
%       model ID 0 only when strict seed validation is disabled.
%       allModelPassIds and allModelMaxDistances record per-plane provenance
%       and distance limits; tallWallDebug retains supplementary fit details.
%       localModelAnchorCounts counts validation anchors in accepted
%       components; localModelFitAnchorCounts counts original fit-window
%       anchors that are actual plane inliers, even outside accepted parts.
% ---------------------------------------------------------------------
    if cfg.enabled && isfield(cfg,"tallWall") && cfg.tallWall.enabled
        assert(cfg.requireSeedPlaneValidation && cfg.tallWall.requireSeedPlaneValidation);
        assert(~isfield(cfg.tallWall,"tallWall"),"Supplementary passes cannot recurse.");
        primaryCfg = rmfield(cfg,"tallWall");
        [pointLineIds,debug] = completeFacadeSurfaces( ...
            xyz,pillarIndices,seedLineIds,eligible,lines,primaryCfg);
        [tallIds,tallDebug] = completeFacadeSurfaces( ...
            xyz,pillarIndices,seedLineIds,eligible,lines,cfg.tallWall);
        added = pointLineIds == 0 & tallIds > 0;
        modelOffset = uint32(size(debug.allModels,1));
        pointLineIds(added) = tallIds(added);
        debug.pointModelIds(added) = tallDebug.pointModelIds(added)+modelOffset;
        debug.allModels = [debug.allModels;tallDebug.allModels];
        debug.allModelPassIds = [debug.allModelPassIds;2*ones(size(tallDebug.allModels,1),1,"uint8")];
        debug.allModelMaxDistances = [debug.allModelMaxDistances;tallDebug.allModelMaxDistances];
        debug.acceptedComponents = debug.acceptedComponents+tallDebug.acceptedComponents;
        debug.candidateMask = debug.candidateMask | tallDebug.candidateMask;
        debug.tallWallAddedPoints = nnz(added);
        debug.tallWallDebug = tallDebug;
        for lineId = 1:size(lines,1)
            debug.addedPoints(lineId) = nnz(seedLineIds(:) == 0 & pointLineIds == lineId);
        end
        debug.rejectedSeedPoints = nnz(seedLineIds(:) > 0 & pointLineIds == 0);
        return;
    end
    xyz = double(xyz);
    pointLineIds = uint16(seedLineIds(:));
    eligible = logical(eligible(:));
    seedLineIds = pointLineIds;
    requireSeedPlaneValidation = isfield(cfg,"requireSeedPlaneValidation") && ...
        cfg.requireSeedPlaneValidation;
    if requireSeedPlaneValidation
        pointLineIds(:) = 0;
    end
    needsPlaneValidation = seedLineIds == 0 | requireSeedPlaneValidation;
    lineCount = size(lines,1);
    debug = struct("models",nan(lineCount,4), ...
        "acceptedComponents",zeros(lineCount,1),"addedPoints",zeros(lineCount,1), ...
        "pointModelIds",zeros(size(pointLineIds),"uint32"), ...
        "candidateMask",seedLineIds > 0 & eligible);
    if ~cfg.enabled
        return;
    end
    randomState = rng;
    cleanup = onCleanup(@() rng(randomState));
    rng(cfg.randomSeed,"twister");
    bestDistance = inf(size(pointLineIds));
    for lineId = 1:lineCount
        anchors = seedLineIds == lineId & eligible;
        if nnz(anchors) < cfg.minAnchorPoints
            continue;
        end
        segment = double(lines(lineId,:));
        tangent = segment(3:4)-segment(1:2);
        segmentLength = norm(tangent);
        if ~isfinite(segmentLength) || segmentLength <= eps
            continue;
        end
        tangent = tangent/segmentLength;
        referenceNormal = [-tangent(2),tangent(1),0];
        along = xyz(:,1:2)*tangent.';
        across = (xyz(:,1:2)-segment(1:2))*referenceNormal(1:2).';
        candidates = eligible & abs(across) <= cfg.candidateBandMeters & ...
            along >= min(along(anchors))-cfg.maxExtensionMeters & ...
            along <= max(along(anchors))+cfg.maxExtensionMeters;
        debug.candidateMask = debug.candidateMask | candidates;
        candidateRows = find(candidates);
        if numel(candidateRows) < cfg.minComponentPoints
            continue;
        end
        supportAnchors = planeAnchorsLocal(seedLineIds,eligible,candidates,lines,lineId,cfg);
        [model,inliers] = pcfitplane(pointCloud(xyz(candidateRows,:)), ...
            cfg.maxPointDistanceMeters,referenceNormal,cfg.maxNormalAngleDeg, ...
            MaxNumTrials=cfg.maxNumTrials);
        if isempty(inliers)
            continue;
        end
        plane = double(model.Parameters);
        plane = plane/norm(plane(1:3));
        debug.models(lineId,:) = plane;
        planeRows = candidateRows(inliers);
        [rows,componentCount] = connectedPlaneSupportLocal( ...
            xyz,along,pillarIndices,planeRows,supportAnchors,cfg);
        debug.acceptedComponents(lineId) = componentCount;
        distances = abs(xyz(rows,:)*plane(1:3).'+plane(4));
        update = needsPlaneValidation(rows) & distances <= cfg.maxPointDistanceMeters & ...
            distances < bestDistance(rows);
        pointLineIds(rows(update)) = uint16(lineId);
        bestDistance(rows(update)) = distances(update);
        debug.pointModelIds(rows(update)) = uint32(lineId);
    end
    debug.allModels = debug.models;
    debug.localModelLineIds = zeros(0,1);
    debug.localModelCenters = zeros(0,1);
    debug.localModelHeightCenters = zeros(0,1);
    debug.localModelAnchorCounts = zeros(0,1);
    debug.localModelFitAnchorCounts = zeros(0,1);
    if isfield(cfg,"localFitWindowMeters") && cfg.localFitWindowMeters > 0
        validateattributes(cfg.localFitWindowMeters,{'numeric'},{'scalar','finite','positive'});
        validateattributes(cfg.localFitStepMeters,{'numeric'},{'scalar','finite','positive'});
        modeCount = 1;
        if isfield(cfg,"localSectionHeightMeters") && cfg.localSectionHeightMeters > 0
            validateattributes(cfg.localSectionHeightMeters,{'numeric'},{'scalar','finite','positive'});
            validateattributes(cfg.localSectionStepMeters,{'numeric'},{'scalar','finite','positive'});
            modeCount = 2;
        end
        for fitMode = 1:modeCount
            for lineId = 1:lineCount
                anchors = seedLineIds == lineId & eligible;
                if nnz(anchors) < cfg.minAnchorPoints
                    continue;
                end
                segment = double(lines(lineId,:));
                tangent = segment(3:4)-segment(1:2);
                segmentLength = norm(tangent);
                if ~isfinite(segmentLength) || segmentLength <= eps
                    continue;
                end
                tangent = tangent/segmentLength;
                referenceNormal = [-tangent(2),tangent(1),0];
                along = xyz(:,1:2)*tangent.';
                across = (xyz(:,1:2)-segment(1:2))*referenceNormal(1:2).';
                candidates = eligible & abs(across) <= cfg.candidateBandMeters & ...
                    along >= min(along(anchors))-cfg.maxExtensionMeters & ...
                    along <= max(along(anchors))+cfg.maxExtensionMeters;
                debug.candidateMask = debug.candidateMask | candidates;
                candidateRows = find(candidates);
                supportAnchors = planeAnchorsLocal(seedLineIds,eligible,candidates,lines,lineId,cfg);
                centers = (floor(min(along(anchors))/cfg.localFitStepMeters): ...
                    ceil(max(along(anchors))/cfg.localFitStepMeters))*cfg.localFitStepMeters;
                for center = centers
                    horizontalWindow = abs(along-center) <= cfg.localFitWindowMeters/2;
                    heightCenters = NaN;
                    if fitMode == 2
                        windowAnchors = supportAnchors & candidates & horizontalWindow;
                        if ~any(windowAnchors)
                            continue;
                        end
                        anchorZ = xyz(windowAnchors,3);
                        heightCenters = (floor(min(anchorZ)/cfg.localSectionStepMeters): ...
                            ceil(max(anchorZ)/cfg.localSectionStepMeters))*cfg.localSectionStepMeters;
                    end
                    for heightCenter = heightCenters
                        inWindow = horizontalWindow;
                        if fitMode == 2
                            inWindow = inWindow & abs(xyz(:,3)-heightCenter) <= cfg.localSectionHeightMeters/2;
                        end
                        fittingAnchors = supportAnchors & candidates & inWindow;
                        sampleIndices = find(inWindow(candidateRows));
                        if nnz(fittingAnchors) < cfg.minAnchorPoints || numel(sampleIndices) < cfg.minComponentPoints
                            continue;
                        end
                        rng(cfg.randomSeed,"twister");
                        [model,inliers] = pcfitplane(pointCloud(xyz(candidateRows,:)), ...
                            cfg.maxPointDistanceMeters,referenceNormal,cfg.maxNormalAngleDeg, ...
                            MaxNumTrials=cfg.maxNumTrials,SampleIndices=sampleIndices);
                        if isempty(inliers)
                            continue;
                        end
                        plane = double(model.Parameters);
                        plane = plane/norm(plane(1:3));
                        validationAnchors = fittingAnchors;
                        if isfield(cfg,"requireAnchorsInFitWindow") && ~cfg.requireAnchorsInFitWindow
                            validationAnchors = supportAnchors;
                        end
                        [rows,componentCount] = connectedPlaneSupportLocal( ...
                            xyz,along,pillarIndices,candidateRows(inliers),validationAnchors,cfg);
                        if isempty(rows)
                            continue;
                        end
                        debug.allModels(end+1,:) = plane;
                        debug.localModelLineIds(end+1,1) = lineId;
                        debug.localModelCenters(end+1,1) = center;
                        debug.localModelHeightCenters(end+1,1) = heightCenter;
                        debug.localModelAnchorCounts(end+1,1) = nnz(validationAnchors(rows));
                        debug.localModelFitAnchorCounts(end+1,1) = nnz(fittingAnchors(candidateRows(inliers)));
                        debug.acceptedComponents(lineId) = debug.acceptedComponents(lineId)+componentCount;
                        distances = abs(xyz(rows,:)*plane(1:3).'+plane(4));
                        update = needsPlaneValidation(rows) & distances <= cfg.maxPointDistanceMeters & ...
                            distances < bestDistance(rows);
                        pointLineIds(rows(update)) = uint16(lineId);
                        bestDistance(rows(update)) = distances(update);
                        debug.pointModelIds(rows(update)) = uint32(size(debug.allModels,1));
                    end
                end
            end
        end
    end
    debug.allModelPassIds = ones(size(debug.allModels,1),1,"uint8");
    debug.allModelMaxDistances = repmat(cfg.maxPointDistanceMeters,size(debug.allModels,1),1);
    for lineId = 1:lineCount
        debug.addedPoints(lineId) = nnz(seedLineIds == 0 & pointLineIds == lineId);
    end
    debug.rejectedSeedPoints = nnz(seedLineIds > 0 & pointLineIds == 0);
end

function anchors = planeAnchorsLocal(seedLineIds,eligible,candidates,lines,lineId,cfg)
% ---------------------------------------------------------------------
% planeAnchorsLocal: Select immutable original seeds from the current line
% or nearby parallel Hough hypotheses. Sharing is restricted to the same
% bounded candidate strip; downstream component checks additionally require
% every counted anchor to be a connected inlier of the fitted 3D plane.
%
% Input:
%   seedLineIds: original per-point Hough line assignments, with zero unset.
%   eligible, candidates: point eligibility and bounded candidate masks.
%   lines: [L x 4] Hough segments; lineId selects the current hypothesis.
%   cfg: optional parallelAnchorAngleDeg limit; absent preserves ownership.
%
% Output:
%   anchors: logical per-point mask; newly classified points are never used.
% ---------------------------------------------------------------------
    anchors = seedLineIds == lineId & eligible;
    if ~isfield(cfg,"parallelAnchorAngleDeg")
        return;
    end
    validateattributes(cfg.parallelAnchorAngleDeg,{'numeric'},{'scalar','finite','>=',0,'<',90});
    directions = double(lines(:,3:4)-lines(:,1:2));
    lengths = vecnorm(directions,2,2);
    directions = directions./max(lengths,eps);
    aligned = lengths > eps & abs(directions*directions(lineId,:).') >= cosd(cfg.parallelAnchorAngleDeg);
    rows = find(seedLineIds > 0 & eligible & candidates);
    anchors(rows) = aligned(seedLineIds(rows));
end

function [acceptedRows,componentCount] = connectedPlaneSupportLocal(xyz,along,pillarIndices,planeRows,anchors,cfg)
% ---------------------------------------------------------------------
% connectedPlaneSupportLocal: Accept connected plane components only when
% their height/width and their original anchor points meet every geometric
% threshold. The caller selects original anchors either inside a local fit
% window or over the full component, always in the original bounded strip.
% Optional convex-hull areas in along-wall/height coordinates reject thin
% diagonal structures even when their bounding width and height are large.
%
% Input:
%   xyz, along, pillarIndices: full coordinates, line projection, columns.
%   planeRows: raw-point row indices already within the plane tolerance.
%   anchors: logical eligible original-anchor mask, optionally windowed.
%   cfg: connectivity, component, anchor, and optional projected-area limits.
%
% Output:
%   acceptedRows: indices of all points in accepted components.
%   componentCount: number of accepted components.
% ---------------------------------------------------------------------
    acceptedRows = zeros(0,1);
    componentCount = 0;
    if isempty(planeRows)
        return;
    end
    coordinates = [along(planeRows),xyz(planeRows,3)];
    cells = floor((coordinates-min(coordinates,[],1))/cfg.connectivityCellMeters)+1;
    cellSize = max(cells,[],1);
    cellIds = sub2ind(cellSize,cells(:,1),cells(:,2));
    occupancy = false(cellSize);
    occupancy(cellIds) = true;
    components = bwconncomp(occupancy,8);
    componentLabels = labelmatrix(components);
    pointComponents = componentLabels(cellIds);
    accepted = false(size(planeRows));
    for componentId = 1:components.NumObjects
        membership = pointComponents == componentId;
        rows = planeRows(membership);
        localAnchors = rows(anchors(rows));
        if numel(rows) < cfg.minComponentPoints || numel(localAnchors) < cfg.minAnchorPoints
            continue;
        end
        if max(xyz(rows,3))-min(xyz(rows,3)) < cfg.minComponentHeightMeters || ...
                max(along(rows))-min(along(rows)) < cfg.minComponentWidthMeters || ...
                max(xyz(localAnchors,3))-min(xyz(localAnchors,3)) < cfg.minAnchorHeightMeters || ...
                max(along(localAnchors))-min(along(localAnchors)) < cfg.minAnchorWidthMeters || ...
                numel(unique(pillarIndices(localAnchors))) < cfg.minAnchorColumns
            continue;
        end
        if isfield(cfg,"minComponentAreaMeters2") && ...
                planeSupportAreaLocal([along(rows),xyz(rows,3)]) < cfg.minComponentAreaMeters2
            continue;
        end
        if isfield(cfg,"minAnchorAreaMeters2") && ...
                planeSupportAreaLocal([along(localAnchors),xyz(localAnchors,3)]) < cfg.minAnchorAreaMeters2
            continue;
        end
        accepted(membership) = true;
        componentCount = componentCount+1;
    end
    acceptedRows = planeRows(accepted);
end

function area = planeSupportAreaLocal(coordinates)
% ---------------------------------------------------------------------
% planeSupportAreaLocal: Measure the convex-hull area of projected wall or
% anchor points. Collinear or insufficient points have zero area and cannot
% satisfy a positive surface-support threshold.
%
% Input:
%   coordinates: [N x 2] finite along-wall/height coordinates in meters.
%
% Output:
%   area: nonnegative projected support area in square meters.
% ---------------------------------------------------------------------
    area = 0;
    if size(coordinates,1) > 2 && rank(coordinates-mean(coordinates,1)) == 2
        [~,area] = convhull(coordinates);
    end
end
