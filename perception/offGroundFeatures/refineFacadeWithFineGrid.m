function [facadeLineMapRefined, facadeMaskRefined, refineDebug] = refineFacadeWithFineGrid(facadePillarLineMap, facadeMask, fineVoxelGrid, detectedLines, cfg)
% refineFacadeWithFineGrid: Refine facade pillar assignments per
% detected facade line by checking local 3D fine-grid patches for
% planarity and normal alignment to each line normal.
%
% Input:
%   facadePillarLineMap: [Ny x Nx] uint16 line-id map from detector
%   facadeMask: [Ny x Nx] logical facade pillar mask
%   fineVoxelGrid: struct with fields valid, count3D, zCenters, voxelSize
%   detectedLines: [L x 4] double facade line segments in meters
%   cfg: off-ground processing configuration struct
%
% Output:
%   facadeLineMapRefined: [Ny x Nx] uint16 refined line-id map
%   facadeMaskRefined: [Ny x Nx] logical refined facade mask
%   refineDebug: struct with per-line patch refinement metrics
    facadeLineMapRefined = uint16(facadePillarLineMap);
    facadeMaskRefined = logical(facadeMask) & (facadeLineMapRefined > 0);
    params = resolveFacadeRefineParams(cfg);

    lineCount = round(double(max(facadeLineMapRefined(:))));
    lineCount = max(0, lineCount);
    refineDebug = struct("enabled", params.enabled, "applied", false, "reason", "", ...
        "lineCount", lineCount, "lineSeedPillarCount", zeros(lineCount, 1), ...
        "linePatchCount", zeros(lineCount, 1), "lineAcceptedPatchCount", zeros(lineCount, 1), ...
        "lineAcceptedPillarCount", zeros(lineCount, 1), ...
        "lineMeanPlanarity", zeros(lineCount, 1), "lineMeanNormalAngleDeg", zeros(lineCount, 1));

    if ~params.enabled
        refineDebug.reason = "disabled";
        return;
    end
    if ~any(facadeMaskRefined(:))
        refineDebug.reason = "emptyFacadeMask";
        return;
    end
    if ~isstruct(fineVoxelGrid) || ~isfield(fineVoxelGrid, "valid") || ~logical(fineVoxelGrid.valid)
        refineDebug.reason = "missingFineGrid";
        return;
    end

    supportMask = imdilate(facadeMaskRefined, params.supportKernel2D);
    [count3D, zCenters, fineReason, fineGridMeta] = resolveFineGridForSupportMask(fineVoxelGrid, supportMask);
    if isempty(count3D) || size(count3D, 3) < 1
        refineDebug.reason = fineReason;
        return;
    end

    voxelSizeXY = fineGridMeta.voxelSizeXY;
    originXY = fineGridMeta.originXY;

    [~, lineTheta, lineValid] = segmentsToHoughParams(double(detectedLines));
    lineNormals = zeros(lineCount, 2);
    lineValidLocal = false(lineCount, 1);
    for lineId = 1:lineCount
        refineDebug.lineSeedPillarCount(lineId) = nnz(facadeLineMapRefined == lineId);
        if lineId <= numel(lineValid) && lineValid(lineId)
            nxy = [cosd(lineTheta(lineId)), sind(lineTheta(lineId))];
            nNorm = hypot(nxy(1), nxy(2));
            if isfinite(nNorm) && nNorm > eps
                lineNormals(lineId, :) = nxy ./ nNorm;
                lineValidLocal(lineId) = true;
            end
        end
    end
    if ~any(lineValidLocal(:))
        refineDebug.reason = "noValidLineNormals";
        return;
    end

    [ny, nx, ~] = size(count3D);
    refinedMap = zeros(size(facadeLineMapRefined), "uint16");
    bestScore = -inf(size(facadeLineMapRefined));

    for lineId = 1:lineCount
        if ~lineValidLocal(lineId)
            continue;
        end
        lineSeedMask = facadeLineMapRefined == lineId;
        if ~any(lineSeedMask(:))
            continue;
        end

        lineSupport = imdilate(lineSeedMask, params.supportKernel2D) & supportMask;
        lineSupportFine = lineSupport;
        if (size(count3D, 1) ~= size(lineSupport, 1)) || (size(count3D, 2) ~= size(lineSupport, 2))
            lineSupportFine = expandCoarseMaskToFine(lineSupport, [size(count3D, 1), size(count3D, 2)], fineGridMeta);
        end
        lineCount3D = count3D .* cast(lineSupportFine, "like", count3D);
        occLin = find(lineCount3D > 0);
        if isempty(occLin)
            continue;
        end

        [rowIdx, colIdx, zIdx] = ind2sub(size(lineCount3D), occLin);
        patchId = assignFacadePatchIds(rowIdx, colIdx, zIdx, params.patchSizeVoxels);
        [~, ~, patchMembership] = unique(patchId, "stable");
        numPatches = max(patchMembership);
        refineDebug.linePatchCount(lineId) = numPatches;

        acceptedPatchCount = 0;
        acceptedPlanarity = zeros(numPatches, 1);
        acceptedAngleDeg = zeros(numPatches, 1);
        for iPatch = 1:numPatches
            patchMask = patchMembership == iPatch;
            patchVoxelCount = nnz(patchMask);
            if patchVoxelCount < params.minPatchVoxels
                continue;
            end

            patchLin = occLin(patchMask);
            patchWeights = double(lineCount3D(patchLin));
            patchPointCount = sum(patchWeights);
            if ~isfinite(patchPointCount) || patchPointCount < params.minPatchPoints
                continue;
            end

            [keepPatch, patchMetrics] = evaluateFacadePatch( ...
                rowIdx(patchMask), colIdx(patchMask), zIdx(patchMask), patchWeights, ...
                originXY, voxelSizeXY, zCenters, lineNormals(lineId, :), params);
            if ~keepPatch
                continue;
            end

            acceptedPatchCount = acceptedPatchCount + 1;
            acceptedPlanarity(acceptedPatchCount) = patchMetrics.planarity;
            acceptedAngleDeg(acceptedPatchCount) = patchMetrics.normalAngleDeg;

            patchRows = rowIdx(patchMask);
            patchCols = colIdx(patchMask);
            patchMask2D = false(ny, nx);
            patchMask2D(sub2ind([ny, nx], patchRows, patchCols)) = true;
            patchScore = patchMetrics.planarity * max(0, 1 - (patchMetrics.normalAngleDeg ./ max(params.maxNormalAngleDeg, eps)));
            if ~isfinite(patchScore)
                patchScore = 0;
            end

            patchMaskCoarse = patchMask2D;
            if (size(patchMask2D, 1) ~= size(bestScore, 1)) || (size(patchMask2D, 2) ~= size(bestScore, 2))
                patchMaskCoarse = projectFineMaskToCoarse(patchMask2D, size(bestScore), fineGridMeta);
            end

            updateMask = patchMaskCoarse & (patchScore > bestScore);
            if any(updateMask(:))
                refinedMap(updateMask) = uint16(lineId);
                bestScore(updateMask) = patchScore;
            end
        end

        refineDebug.lineAcceptedPatchCount(lineId) = acceptedPatchCount;
        if acceptedPatchCount > 0
            refineDebug.lineMeanPlanarity(lineId) = mean(acceptedPlanarity(1:acceptedPatchCount));
            refineDebug.lineMeanNormalAngleDeg(lineId) = mean(acceptedAngleDeg(1:acceptedPatchCount));
        end
    end

    facadeLineMapRefined = refinedMap;
    facadeMaskRefined = facadeLineMapRefined > 0;
    for lineId = 1:lineCount
        refineDebug.lineAcceptedPillarCount(lineId) = nnz(facadeLineMapRefined == lineId);
    end

    if ~any(facadeMaskRefined(:)) && params.keepSeedWhenEmpty
        facadeLineMapRefined = uint16(facadePillarLineMap);
        facadeMaskRefined = logical(facadeMask) & (facadeLineMapRefined > 0);
        refineDebug.applied = true;
        refineDebug.reason = "fallbackToSeed";
        return;
    end

    refineDebug.applied = true;
    refineDebug.reason = "ok";
end

function patchId = assignFacadePatchIds(rowIdx, colIdx, zIdx, patchSizeVoxels)
% assignFacadePatchIds: Assign occupied fine voxels to regular 3D
% patch ids by quantizing row, column, and z indices with a patch size.
%
% Input:
%   rowIdx: [K x 1] row indices in [1, Ny]
%   colIdx: [K x 1] column indices in [1, Nx]
%   zIdx: [K x 1] z indices in [1, Nz]
%   patchSizeVoxels: [1 x 3] positive integer patch sizes [row col z]
%
% Output:
%   patchId: [K x 1] double patch id per occupied voxel
    rowIdx = double(rowIdx(:));
    colIdx = double(colIdx(:));
    zIdx = double(zIdx(:));
    if isempty(rowIdx)
        patchId = zeros(0, 1);
        return;
    end

    patchSize = double(patchSizeVoxels(:).');
    if numel(patchSize) < 3
        patchSize = [patchSize, ones(1, 3 - numel(patchSize))];
    end
    patchSize = max(1, round(patchSize(1:3)));

    rowBin = floor((rowIdx - min(rowIdx)) ./ patchSize(1));
    colBin = floor((colIdx - min(colIdx)) ./ patchSize(2));
    zBin = floor((zIdx - min(zIdx)) ./ patchSize(3));

    numRowBins = max(rowBin) + 1;
    numColBins = max(colBin) + 1;
    patchId = rowBin + (colBin .* numRowBins) + (zBin .* numRowBins .* numColBins) + 1;
end

function [keepPatch, metrics] = evaluateFacadePatch(rowIdx, colIdx, zIdx, weights, originXY, voxelSizeXY, zCenters, lineNormalXY, params)
% evaluateFacadePatch: Evaluate one fine-grid patch by weighted 3D
% PCA planarity and angle between patch normal and facade line normal.
%
% Input:
%   rowIdx: [K x 1] row indices of occupied voxels
%   colIdx: [K x 1] column indices of occupied voxels
%   zIdx: [K x 1] z indices of occupied voxels
%   weights: [K x 1] positive voxel weights
%   originXY: [1 x 2] xy origin in meters
%   voxelSizeXY: [1 x 2] xy voxel size in meters
%   zCenters: [Nz x 1] z-center vector in meters
%   lineNormalXY: [1 x 2] normalized facade-line normal in xy plane
%   params: refinement parameter struct
%
% Output:
%   keepPatch: logical scalar keep/remove decision
%   metrics: struct with planarity and normal-angle values
    keepPatch = false;
    metrics = struct("planarity", 0, "normalAngleDeg", 90);

    rowIdx = double(rowIdx(:));
    colIdx = double(colIdx(:));
    zIdx = double(zIdx(:));
    weights = double(weights(:));
    if isempty(rowIdx) || numel(rowIdx) ~= numel(colIdx) || numel(colIdx) ~= numel(zIdx) || numel(zIdx) ~= numel(weights)
        return;
    end

    valid = isfinite(rowIdx) & isfinite(colIdx) & isfinite(zIdx) & isfinite(weights) & (weights > 0);
    valid = valid & (zIdx >= 1) & (zIdx <= numel(zCenters));
    if ~any(valid)
        return;
    end

    rowIdx = rowIdx(valid);
    colIdx = colIdx(valid);
    zIdx = zIdx(valid);
    weights = weights(valid);

    xVals = originXY(1) + ((colIdx - 0.5) .* voxelSizeXY(1));
    yVals = originXY(2) + ((rowIdx - 0.5) .* voxelSizeXY(2));
    zVals = double(zCenters(round(zIdx)));

    wSum = sum(weights);
    if ~isfinite(wSum) || wSum <= eps
        return;
    end

    mx = sum(weights .* xVals) ./ wSum;
    my = sum(weights .* yVals) ./ wSum;
    mz = sum(weights .* zVals) ./ wSum;
    dxVals = xVals - mx;
    dyVals = yVals - my;
    dzVals = zVals - mz;

    cxx = sum(weights .* dxVals .* dxVals) ./ wSum;
    cyy = sum(weights .* dyVals .* dyVals) ./ wSum;
    czz = sum(weights .* dzVals .* dzVals) ./ wSum;
    cxy = sum(weights .* dxVals .* dyVals) ./ wSum;
    cxz = sum(weights .* dxVals .* dzVals) ./ wSum;
    cyz = sum(weights .* dyVals .* dzVals) ./ wSum;

    covMat = [cxx, cxy, cxz; cxy, cyy, cyz; cxz, cyz, czz];
    covMat = 0.5 * (covMat + covMat.');
    covMat(~isfinite(covMat)) = 0;

    [eigMaxRaw, eigMidRaw, eigMinRaw] = computeEigenvalues3x3Analytic(covMat);
    eigMax = max(eigMaxRaw, 0);
    eigMid = max(eigMidRaw, 0);
    eigMin = max(eigMinRaw, 0);
    planarity = (eigMid - eigMin) ./ max(eigMax, eps);
    if ~isfinite(planarity)
        planarity = 0;
    end
    planarity = min(max(planarity, 0), 1);

    nVec = computeSmallestEigenvector3x3(covMat, eigMinRaw);
    nNorm = norm(nVec);
    if ~isfinite(nNorm) || nNorm <= eps
        return;
    end
    nVec = nVec ./ nNorm;
    nxy = nVec(1:2);
    nxyNorm = hypot(nxy(1), nxy(2));
    if ~isfinite(nxyNorm) || nxyNorm <= eps
        return;
    end
    nxy = nxy ./ nxyNorm;

    ln = double(lineNormalXY(:));
    lnNorm = hypot(ln(1), ln(2));
    if ~isfinite(lnNorm) || lnNorm <= eps
        return;
    end
    ln = ln ./ lnNorm;

    alignment = abs(nxy(:).' * ln(:));
    alignment = min(max(alignment, 0), 1);
    normalAngleDeg = acosd(alignment);

    metrics.planarity = planarity;
    metrics.normalAngleDeg = normalAngleDeg;

    keepPatch = (planarity >= params.planarityThreshold) && (normalAngleDeg <= params.maxNormalAngleDeg);
end

function params = resolveFacadeRefineParams(cfg)
% resolveFacadeRefineParams: Read and sanitize facade 3D refinement
% parameters from configuration with robust defaults.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   params: struct with validated scalar/vector refinement parameters
    params = struct();
    params.enabled = true;
    if isfield(cfg, "facadeRefineEnabled") && isscalar(cfg.facadeRefineEnabled)
        params.enabled = logical(cfg.facadeRefineEnabled);
    end

    params.supportRadiusVoxels = 2;
    if isfield(cfg, "facadeRefineHorizontalSupportRadiusVoxels") && isscalar(cfg.facadeRefineHorizontalSupportRadiusVoxels) && ...
            isfinite(cfg.facadeRefineHorizontalSupportRadiusVoxels)
        params.supportRadiusVoxels = max(0, round(double(cfg.facadeRefineHorizontalSupportRadiusVoxels)));
    end

    params.patchSizeVoxels = [4, 4, 6];
    if isfield(cfg, "facadeRefinePatchSizeVoxels") && ~isempty(cfg.facadeRefinePatchSizeVoxels) && all(isfinite(cfg.facadeRefinePatchSizeVoxels(:)))
        patch = double(cfg.facadeRefinePatchSizeVoxels(:).');
        if isscalar(patch)
            patch = [patch, patch, patch];
        elseif numel(patch) == 2
            patch = [patch, patch(2)];
        else
            patch = patch(1:3);
        end
        params.patchSizeVoxels = max(1, round(patch));
    end

    params.minPatchVoxels = 8;
    if isfield(cfg, "facadeRefineMinPatchVoxels") && isscalar(cfg.facadeRefineMinPatchVoxels) && isfinite(cfg.facadeRefineMinPatchVoxels)
        params.minPatchVoxels = max(1, round(double(cfg.facadeRefineMinPatchVoxels)));
    end

    params.minPatchPoints = 30;
    if isfield(cfg, "facadeRefineMinPatchPoints") && isscalar(cfg.facadeRefineMinPatchPoints) && isfinite(cfg.facadeRefineMinPatchPoints)
        params.minPatchPoints = max(1, double(cfg.facadeRefineMinPatchPoints));
    end

    params.planarityThreshold = 0.35;
    if isfield(cfg, "facadeRefinePlanarityThreshold") && isscalar(cfg.facadeRefinePlanarityThreshold) && isfinite(cfg.facadeRefinePlanarityThreshold)
        params.planarityThreshold = min(max(double(cfg.facadeRefinePlanarityThreshold), 0), 1);
    end

    params.maxNormalAngleDeg = 20;
    if isfield(cfg, "facadeRefineMaxNormalAngleDeg") && isscalar(cfg.facadeRefineMaxNormalAngleDeg) && isfinite(cfg.facadeRefineMaxNormalAngleDeg)
        params.maxNormalAngleDeg = min(max(double(cfg.facadeRefineMaxNormalAngleDeg), 0), 90);
    end

    params.keepSeedWhenEmpty = true;
    if isfield(cfg, "facadeRefineKeepSeedWhenEmpty") && isscalar(cfg.facadeRefineKeepSeedWhenEmpty)
        params.keepSeedWhenEmpty = logical(cfg.facadeRefineKeepSeedWhenEmpty);
    end

    params.supportKernel2D = true((2 * params.supportRadiusVoxels) + 1, (2 * params.supportRadiusVoxels) + 1);
end

function [eigMax, eigMid, eigMin] = computeEigenvalues3x3Analytic(covMat)
% computeEigenvalues3x3Analytic: Compute the descending eigenvalues of a
% real symmetric 3x3 matrix with the closed-form cubic solution so PCA
% feature extraction avoids iterative eigendecomposition.
%
% Input:
%   covMat: [3 x 3] real covariance or symmetric matrix
%
% Output:
%   eigMax: scalar largest eigenvalue
%   eigMid: scalar middle eigenvalue
%   eigMin: scalar smallest eigenvalue
    assert(isequal(size(covMat), [3, 3]), ...
        "covMat must be a 3x3 matrix.");

    covMat = double(covMat);
    covMat = 0.5 * (covMat + covMat.');
    covMat(~isfinite(covMat)) = 0;

    a11 = covMat(1, 1);
    a22 = covMat(2, 2);
    a33 = covMat(3, 3);
    a12 = covMat(1, 2);
    a13 = covMat(1, 3);
    a23 = covMat(2, 3);

    p1 = (a12 * a12) + (a13 * a13) + (a23 * a23);
    if p1 <= eps
        diagonalVals = sort([a11, a22, a33], "descend");
        eigMax = diagonalVals(1);
        eigMid = diagonalVals(2);
        eigMin = diagonalVals(3);
        return;
    end

    q = (a11 + a22 + a33) / 3.0;
    b11 = a11 - q;
    b22 = a22 - q;
    b33 = a33 - q;
    p2 = (b11 * b11) + (b22 * b22) + (b33 * b33) + (2.0 * p1);
    p = sqrt(max(p2, 0) / 6.0);
    if ~isfinite(p) || p <= eps
        diagonalVals = sort([a11, a22, a33], "descend");
        eigMax = diagonalVals(1);
        eigMid = diagonalVals(2);
        eigMin = diagonalVals(3);
        return;
    end

    detCentered = ...
        (b11 * ((b22 * b33) - (a23 * a23))) - ...
        (a12 * ((a12 * b33) - (a13 * a23))) + ...
        (a13 * ((a12 * a23) - (b22 * a13)));
    r = 0.5 * detCentered / (p * p * p);
    r = min(max(r, -1), 1);
    phi = acos(r) / 3.0;

    eigMax = q + (2.0 * p * cos(phi));
    eigMin = q + (2.0 * p * cos(phi + (2.0 * pi / 3.0)));
    eigMid = (3.0 * q) - eigMax - eigMin;
end

function eigenVec = computeSmallestEigenvector3x3(covMat, eigMin)
% computeSmallestEigenvector3x3: Construct one eigenvector associated
% with the smallest eigenvalue of a real symmetric 3x3 matrix by taking
% cross-products of shifted-matrix row pairs and selecting the most
% numerically stable candidate.
%
% Input:
%   covMat: [3 x 3] real covariance or symmetric matrix
%   eigMin: scalar smallest eigenvalue associated with covMat
%
% Output:
%   eigenVec: [3 x 1] double eigenvector candidate for eigMin
    assert(isequal(size(covMat), [3, 3]), ...
        "covMat must be a 3x3 matrix.");

    covMat = double(covMat);
    covMat = 0.5 * (covMat + covMat.');
    covMat(~isfinite(covMat)) = 0;
    eigMin = double(eigMin);

    shiftedMat = covMat - (eigMin * eye(3));
    row1 = shiftedMat(1, :);
    row2 = shiftedMat(2, :);
    row3 = shiftedMat(3, :);

    vec12 = cross(row1, row2);
    vec13 = cross(row1, row3);
    vec23 = cross(row2, row3);

    norm12 = sum(vec12 .* vec12);
    norm13 = sum(vec13 .* vec13);
    norm23 = sum(vec23 .* vec23);

    [bestNorm, bestIdx] = max([norm12, norm13, norm23]);
    if bestNorm <= eps(max(1, norm(covMat, "fro"))) ^ 2
        diagonalVals = diag(covMat);
        [~, minIdx] = min(diagonalVals);
        eigenVec = zeros(3, 1);
        eigenVec(minIdx) = 1;
        return;
    end

    if bestIdx == 1
        eigenVec = vec12(:);
        return;
    end
    if bestIdx == 2
        eigenVec = vec13(:);
        return;
    end

    eigenVec = vec23(:);
end
