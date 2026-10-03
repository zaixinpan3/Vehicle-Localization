function recovery = recoverDowntownDensePoles(points,pillarIdx,eligible,facadeIds,nativeMask,debug,cfg)
% ---------------------------------------------------------------------
% recoverDowntownDensePoles: Reassemble native pole candidates split
% across a small XY footprint and validate dense short-pole support using
% original points. Grow only one configured neighborhood around existing
% candidate cells, intersect with native occupied-layer support, and test
% connected footprints. Reject footprints touching accepted native poles,
% facade or sign points, broad objects, sparse or disconnected samples,
% tilted axes, underpopulated height slices, or substantial nearby clutter.
% Accepted footprints contribute all their original off-ground points.
% No frame number, annotation, point index, or fixed location is consulted.
%
% Input:
%   points: N-by-3 finite original off-ground coordinates in meters.
%   pillarIdx: N-by-1 linear XY indices into nativeMask.
%   eligible: N-by-1 logical points not assigned to traffic signs.
%   facadeIds: N-by-1 current facade labels, zero for non-facade points.
%   nativeMask: Ny-by-Nx logical accepted native pole columns.
%   debug: native detector diagnostics with poleMaskRaw and poleSupportMask.
%   cfg: downtownStructuralConfig().denseRecovery structure.
%
% Output:
%   recovery: pointMask, pointComponentIds, mask, representative pillarLinIdx,
%       and per-candidate metrics including explicit acceptance/rejection.
%       Component IDs are local to this recovery branch.
% ---------------------------------------------------------------------
    candidateMask = debug.poleSupportMask & imdilate(debug.poleMaskRaw, ...
        ones(2*cfg.supportRadiusCells+1));
    components = bwconncomp(candidateMask,8);
    count = components.NumObjects;
    recovery = struct("pointMask",false(size(pillarIdx)), ...
        "pointComponentIds",zeros(size(pillarIdx)),"mask",false(size(nativeMask)), ...
        "pillarLinIdx",zeros(0,1));
    metrics = table((1:count).',zeros(count,1),zeros(count,1),zeros(count,1), ...
        zeros(count,1),nan(count,1),nan(count,1),nan(count,1),nan(count,1),nan(count,1), ...
        false(count,1),strings(count,1),VariableNames=["componentId","pointCount", ...
        "heightMeters","footprintDiagonalMeters","maxVerticalGapMeters", ...
        "principalVarianceFraction","axisTiltDegrees","populatedSliceFraction", ...
        "objectToContextPointRatio","footprintSpanCells","accepted","rejectReason"]);
    representatives = zeros(count,1);
    for k = 1:count
        cells = components.PixelIdxList{k};
        selected = ismember(pillarIdx,cells);
        xyz = double(points(selected,:));
        metrics.pointCount(k) = size(xyz,1);
        if any(nativeMask(cells))
            metrics.rejectReason(k) = "nativePoleOverlap";
            continue;
        end
        if any(~eligible(selected) | facadeIds(selected)>0)
            metrics.rejectReason(k) = "otherFeatureOverlap";
            continue;
        end
        if size(xyz,1)<cfg.minPoints
            metrics.rejectReason(k) = "insufficientPoints";
            continue;
        end
        [rows,cols] = ind2sub(size(nativeMask),cells);
        metrics.footprintSpanCells(k) = max([max(rows)-min(rows)+1,max(cols)-min(cols)+1]);
        heights = sort(xyz(:,3));
        metrics.heightMeters(k) = heights(end)-heights(1);
        metrics.maxVerticalGapMeters(k) = max(diff(heights));
        metrics.footprintDiagonalMeters(k) = norm(max(xyz(:,1:2))-min(xyz(:,1:2)));
        [~,shape] = validateDowntownPoleShape(xyz,ones(size(xyz,1),1),cfg);
        metrics.principalVarianceFraction(k) = shape.principalVarianceFraction;
        metrics.axisTiltDegrees(k) = shape.axisTiltDegrees;
        sliceIdx = floor((xyz(:,3)-heights(1))/cfg.sliceHeightMeters)+1;
        sliceCounts = accumarray(sliceIdx,1);
        metrics.populatedSliceFraction(k) = mean(sliceCounts>=cfg.minSlicePoints);
        contextMask = false(size(nativeMask));
        contextMask(cells) = true;
        contextMask = imdilate(contextMask,ones(2*cfg.contextRadiusCells+1));
        context = contextMask(pillarIdx) & points(:,3)>=heights(1) & points(:,3)<=heights(end);
        metrics.objectToContextPointRatio(k) = nnz(selected)/nnz(context);
        rejected = [metrics.footprintSpanCells(k)>cfg.maxFootprintSpanCells, ...
            metrics.heightMeters(k)<cfg.minHeightMeters || metrics.heightMeters(k)>cfg.maxHeightMeters, ...
            metrics.footprintDiagonalMeters(k)>cfg.maxFootprintDiagonalMeters, ...
            metrics.maxVerticalGapMeters(k)>cfg.maxVerticalGapMeters, ...
            metrics.principalVarianceFraction(k)<cfg.minPrincipalVarianceFraction, ...
            metrics.axisTiltDegrees(k)>cfg.maxAxisTiltDegrees, ...
            metrics.populatedSliceFraction(k)<cfg.minPopulatedSliceFraction, ...
            metrics.objectToContextPointRatio(k)<cfg.minObjectToContextPointRatio];
        reasons = ["largeFootprint","heightOutsideShortPoleRange","wideFootprint", ...
            "verticalGap","notSlender","notVertical","sparseHeightSlices","nearbyClutter"];
        metrics.rejectReason(k) = strjoin(reasons(rejected),"+");
        metrics.accepted(k) = ~any(rejected);
        if metrics.accepted(k)
            recovery.pointMask(selected) = true;
            recovery.pointComponentIds(selected) = k;
            recovery.mask(cells) = true;
            counts = accumarray(pillarIdx(selected),1,[numel(nativeMask),1]);
            [~,best] = max(counts(cells));
            representatives(k) = cells(best);
        end
    end
    recovery.pillarLinIdx = representatives(representatives>0);
    recovery.metrics = metrics;
end
