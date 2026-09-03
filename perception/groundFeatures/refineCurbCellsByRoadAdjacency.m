function energyMaps = refineCurbCellsByRoadAdjacency(energyMaps, roadCellMask, cfg, xyView, roadSeedMask)
% refineCurbCellsByRoadAdjacency: Keep curb-cell candidates only
% when they are adjacent to the road surface grown from ego-near road
% seeds. This removes isolated high-energy structures that are not part of
% the current road boundary without encoding fixed curb rectangles.
%
% Input:
%   energyMaps: struct with extractedMask curb-cell raster
%   roadCellMask: [Ny x Nx] logical road-surface raster
%   cfg: struct from groundFeatureConfig().curb with road-adjacency parameters
%   xyView: optional XY view struct with y centers and y map
%   roadSeedMask: optional [Ny x Nx] logical seed raster for the road
%       centerline reference
%
% Output:
%   energyMaps: struct with extractedMask filtered by road adjacency
    if ~isfield(cfg, "roadAdjacencyFilterEnabled") || ~logical(cfg.roadAdjacencyFilterEnabled)
        return;
    end

    radiusCells = max(0, round(double(cfg.roadAdjacencyRadiusCells)));
    adjacencyKernel = ones((2 * radiusCells) + 1, (2 * radiusCells) + 1);
    adjacencyMask = conv2(double(logical(roadCellMask)), adjacencyKernel, "same") > 0;
    rawExtractedMask = logical(energyMaps.extractedMask);
    bridgeMask = false(size(rawExtractedMask));
    energyMaps.rawExtractedMask = rawExtractedMask;
    if isfield(energyMaps, "extractionStandaloneBridgeMask")
        bridgeMask = logical(energyMaps.extractionStandaloneBridgeMask);
    end
    seedMask = rawExtractedMask & ~bridgeMask;
    if isfield(energyMaps, "extractionStandaloneSeedMask")
        seedMask = logical(energyMaps.extractionStandaloneSeedMask);
    end
    fillMask = false(size(rawExtractedMask));
    if isfield(energyMaps, "extractionFillMask")
        fillMask = logical(energyMaps.extractionFillMask);
    end
    endpointMask = false(size(rawExtractedMask));
    if isfield(energyMaps, "extractionEndpointMask")
        endpointMask = logical(energyMaps.extractionEndpointMask);
    end
    centerEvidenceMask = false(size(rawExtractedMask));
    if isfield(energyMaps, "linearityComponentCenterEvidence")
        centerEvidenceMask = single(energyMaps.linearityComponentCenterEvidence) >= single(cfg.extractionCenterEvidenceMin);
    end
    bridgeOnlyMask = bridgeMask & ~seedMask & ~fillMask & ~endpointMask;
    highConfidenceAnchorMask = seedMask | endpointMask | centerEvidenceMask;
    filteredExtractedMask = false(size(rawExtractedMask));
    components = connectedComponents8(rawExtractedMask);
    minAdjacentCells = 1;
    if isfield(cfg, "roadAdjacencyComponentMinAdjacentCells")
        minAdjacentCells = max(1, round(double(cfg.roadAdjacencyComponentMinAdjacentCells)));
    end
    minAdjacentFraction = 0;
    if isfield(cfg, "roadAdjacencyComponentMinAdjacentFraction")
        minAdjacentFraction = min(max(double(cfg.roadAdjacencyComponentMinAdjacentFraction), 0), 1);
    end
    maxFullKeepCells = Inf;
    if isfield(cfg, "roadAdjacencyComponentMaxFullKeepCells")
        maxFullKeepCells = max(1, round(double(cfg.roadAdjacencyComponentMaxFullKeepCells)));
    end
    largeComponentMinCells = Inf;
    if isfield(cfg, "roadAdjacencyLargeComponentMinCells")
        largeComponentMinCells = max(1, round(double(cfg.roadAdjacencyLargeComponentMinCells)));
    end
    intrinsicMinCells = Inf;
    intrinsicMaxCells = -Inf;
    intrinsicMinSeedCells = Inf;
    if isfield(cfg, "intrinsicComponentMinCells")
        intrinsicMinCells = max(1, round(double(cfg.intrinsicComponentMinCells)));
    end
    if isfield(cfg, "intrinsicComponentMaxCells")
        intrinsicMaxCells = max(1, round(double(cfg.intrinsicComponentMaxCells)));
    end
    if isfield(cfg, "intrinsicComponentMinSeedCells")
        intrinsicMinSeedCells = max(1, round(double(cfg.intrinsicComponentMinSeedCells)));
    end
    for componentIdx = 1:numel(components)
        cellIdx = components{componentIdx};
        adjacentCellCount = nnz(adjacencyMask(cellIdx));
        minFractionAdjacentCells = ceil(double(numel(cellIdx)) .* minAdjacentFraction);
        if adjacentCellCount >= minAdjacentCells && adjacentCellCount >= minFractionAdjacentCells
            if numel(cellIdx) <= maxFullKeepCells
                if isSmallRoadAdjacentCurbComponentSupported(cellIdx, energyMaps, cfg)
                    filteredExtractedMask(cellIdx) = true;
                end
            else
                componentKeepMask = adjacencyMask(cellIdx) & ~bridgeOnlyMask(cellIdx) & highConfidenceAnchorMask(cellIdx);
                if numel(cellIdx) >= largeComponentMinCells
                    componentKeepMask = keepRoadFacingBoundaryAnchors( ...
                        cellIdx, componentKeepMask, roadCellMask, size(rawExtractedMask), cfg, energyMaps);
                    componentKeepMask = componentKeepMask | buildRoadAdjacencyBoundaryContinuation( ...
                        cellIdx, componentKeepMask, size(rawExtractedMask), cfg);
                end
                filteredExtractedMask(cellIdx) = componentKeepMask;
            end
        elseif numel(cellIdx) >= intrinsicMinCells && numel(cellIdx) <= intrinsicMaxCells ...
                && nnz(seedMask(cellIdx)) >= intrinsicMinSeedCells
            filteredExtractedMask(cellIdx) = true;
        end
    end
    if isfield(cfg, "continuationExpansionEnabled") && logical(cfg.continuationExpansionEnabled)
        expansionRadius = max(0, round(double(cfg.continuationExpansionRadiusCells)));
        expansionSupport = conv2(double(filteredExtractedMask), ones((2 * expansionRadius) + 1), "same") > 0;
        continuationMask = expansionSupport & ~filteredExtractedMask ...
            & single(energyMaps.totalBase) >= single(cfg.continuationBaseEnergyThreshold) ...
            & single(energyMaps.linearityComponentCenterEvidence) >= single(cfg.continuationCenterEvidenceMin) ...
            & single(energyMaps.heightStepMeters) >= single(cfg.extractionHeightStepMinMeters);
        filteredExtractedMask = filteredExtractedMask | continuationMask;
    end
    if isfield(cfg, "roadAdjacencyExcludeRoadCells") && logical(cfg.roadAdjacencyExcludeRoadCells)
        filteredExtractedMask = filteredExtractedMask & ~logical(roadCellMask);
    end
    if nargin >= 5 && isstruct(xyView) && ~isempty(roadSeedMask)
        [filteredExtractedMask, weakGapCompletionMask] = completeWeakCurbBoundaryGaps( ...
            filteredExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
        [filteredExtractedMask, pathCompletionMask] = completeProjectedCurbBoundaryPaths( ...
            filteredExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
        [filteredExtractedMask, peakPromotionMask] = keepSameColumnFeaturePeakCurbCells( ...
            filteredExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
        peakPromotionProtectionMask = keepRoadConsistentPeakPromotionProtection( ...
            peakPromotionMask, filteredExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
        shadowProtectedMask = weakGapCompletionMask | pathCompletionMask | peakPromotionMask;
        protectedLinearityMin = 0.45;
        if isfield(cfg, "sameSideCellShadowProtectedLinearityMin")
            protectedLinearityMin = double(cfg.sameSideCellShadowProtectedLinearityMin);
        end
        protectedRoughnessMin = 0;
        if isfield(cfg, "sameSideCellShadowProtectedRoughnessMinMeters")
            protectedRoughnessMin = double(cfg.sameSideCellShadowProtectedRoughnessMinMeters);
        end
        featureProtectedMask = double(energyMaps.linearity) >= protectedLinearityMin ...
            & double(energyMaps.roughnessMeters) >= protectedRoughnessMin;
        negativeFeatureLinearityMin = 0.30;
        if isfield(cfg, "sameSideCellShadowNegativeFeatureLinearityMin")
            negativeFeatureLinearityMin = double(cfg.sameSideCellShadowNegativeFeatureLinearityMin);
        end
        yMap = double(xyView.yMap);
        seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
        referenceY = 0;
        if ~isempty(seedYValues)
            referenceY = median(seedYValues, "omitnan");
        end
        negativeSideRows = double(xyView.yCenters(:)) < referenceY;
        negativeFeatureProtectedMask = false(size(filteredExtractedMask));
        negativeFeatureProtectedMask(negativeSideRows, :) = filteredExtractedMask(negativeSideRows, :) ...
            & double(energyMaps.linearity(negativeSideRows, :)) >= negativeFeatureLinearityMin ...
            & double(energyMaps.roughnessMeters(negativeSideRows, :)) >= protectedRoughnessMin;
        shadowProtectedMask = peakPromotionProtectionMask | negativeFeatureProtectedMask ...
            | (shadowProtectedMask & featureProtectedMask);
        filteredExtractedMask = removeShadowedSameSideCurbComponents(filteredExtractedMask, xyView, roadSeedMask, cfg);
        filteredExtractedMask = removeShadowedSameSideCurbCells( ...
            filteredExtractedMask, xyView, roadSeedMask, cfg, shadowProtectedMask, energyMaps);
        recoveryInputMask = filteredExtractedMask;
        filteredExtractedMask = recoverRoadFacingShoulderCurbCells( ...
            filteredExtractedMask, rawExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
        filteredExtractedMask = completeBoundaryEvidenceRuns( ...
            filteredExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
        if ~isequal(filteredExtractedMask, recoveryInputMask)
            filteredExtractedMask = recoverRoadFacingShoulderCurbCells( ...
                filteredExtractedMask, rawExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
            filteredExtractedMask = completeBoundaryEvidenceRuns( ...
                filteredExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
        end
        filteredExtractedMask = removeRoadFacingDuplicateCurbCells( ...
            filteredExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
        filteredExtractedMask = removeFartherSameSideOutlierComponents( ...
            filteredExtractedMask, xyView, roadSeedMask, cfg);
        filteredExtractedMask = completeDominantBoundaryContinuation( ...
            filteredExtractedMask, rawExtractedMask, energyMaps, xyView, roadSeedMask, cfg);
    end
    energyMaps.extractedMask = filteredExtractedMask;
end

function completedMask = completeDominantBoundaryContinuation(curbMask, rawExtractedMask, energyMaps, xyView, roadSeedMask, cfg)
% completeDominantBoundaryContinuation: Extend accepted curb-cell
% runs along the dominant road-boundary direction when adjacent columns
% contain weak but coherent curb evidence. The completion follows each
% accepted row run with a slope-limited row search, preferring the
% road-facing evidence cell so long left/right curb edges are not cut off
% when the road-adjacency raster ends early.
%
% Input:
%   curbMask: [Ny x Nx] logical accepted curb-cell raster
%   rawExtractedMask: [Ny x Nx] logical raw curb evidence before
%       road-adjacency filtering
%   energyMaps: struct with base, height-step, linearity, and extraction
%       evidence maps
%   xyView: XY view struct with x/y centers and road-seed map coordinates
%   roadSeedMask: [Ny x Nx] logical road seed raster for side reference
%   cfg: struct from groundFeatureConfig().curb with continuation parameters
%
% Output:
%   completedMask: [Ny x Nx] logical curb-cell raster after continuation
    completedMask = logical(curbMask);
    if ~isfield(cfg, "dominantBoundaryContinuationEnabled") ...
            || ~logical(cfg.dominantBoundaryContinuationEnabled) ...
            || ~any(completedMask(:)) || ~isstruct(xyView) || ~isfield(xyView, "yCenters")
        return;
    end

    maxGapCells = 16;
    if isfield(cfg, "dominantBoundaryContinuationMaxGapCells")
        maxGapCells = max(0, round(double(cfg.dominantBoundaryContinuationMaxGapCells)));
    end
    searchRows = 1;
    if isfield(cfg, "dominantBoundaryContinuationSearchRows")
        searchRows = max(0, round(double(cfg.dominantBoundaryContinuationSearchRows)));
    end
    minTotalEnergy = 0;
    if isfield(cfg, "dominantBoundaryContinuationMinTotalEnergy")
        minTotalEnergy = max(0, double(cfg.dominantBoundaryContinuationMinTotalEnergy));
    end
    maxRelativeHeightMeters = Inf;
    if isfield(cfg, "dominantBoundaryContinuationMaxRelativeHeightMeters")
        maxRelativeHeightMeters = double(cfg.dominantBoundaryContinuationMaxRelativeHeightMeters);
    end
    minBaseEnergy = 0.25;
    if isfield(cfg, "dominantBoundaryContinuationMinBaseEnergy")
        minBaseEnergy = max(0, double(cfg.dominantBoundaryContinuationMinBaseEnergy));
    end
    minHeightStepMeters = 0.04;
    if isfield(cfg, "dominantBoundaryContinuationMinHeightStepMeters")
        minHeightStepMeters = max(0, double(cfg.dominantBoundaryContinuationMinHeightStepMeters));
    end
    minLinearity = 0.10;
    if isfield(cfg, "dominantBoundaryContinuationMinLinearity")
        minLinearity = max(0, double(cfg.dominantBoundaryContinuationMinLinearity));
    end
    minXCenterMeters = -Inf;
    if isfield(cfg, "dominantBoundaryContinuationMinXCenterMeters")
        minXCenterMeters = double(cfg.dominantBoundaryContinuationMinXCenterMeters);
        if isempty(minXCenterMeters) || ~isfinite(minXCenterMeters(1))
            minXCenterMeters = -Inf;
        else
            minXCenterMeters = minXCenterMeters(1);
        end
    end

    rawEvidenceMask = logical(rawExtractedMask);
    if isfield(energyMaps, "extractionStandaloneSeedMask")
        rawEvidenceMask = rawEvidenceMask | logical(energyMaps.extractionStandaloneSeedMask);
    end
    if isfield(energyMaps, "extractionStandaloneBridgeMask")
        rawEvidenceMask = rawEvidenceMask | logical(energyMaps.extractionStandaloneBridgeMask);
    end
    if isfield(energyMaps, "extractionFillMask")
        rawEvidenceMask = rawEvidenceMask | logical(energyMaps.extractionFillMask);
    end
    if isfield(energyMaps, "extractionEndpointMask")
        rawEvidenceMask = rawEvidenceMask | logical(energyMaps.extractionEndpointMask);
    end
    weakEvidenceMask = double(energyMaps.totalBase) >= minBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= minHeightStepMeters ...
        & double(energyMaps.linearity) >= minLinearity;
    evidenceMask = (rawEvidenceMask | weakEvidenceMask) & isfinite(double(energyMaps.totalBase)) ...
        & double(energyMaps.total) >= minTotalEnergy;
    if isfinite(maxRelativeHeightMeters) && isfield(energyMaps, "relativeHeightMeters")
        evidenceMask = evidenceMask & double(energyMaps.relativeHeightMeters) <= maxRelativeHeightMeters;
    end
    if isfinite(minXCenterMeters) && isfield(xyView, "xCenters")
        xCenters = double(xyView.xCenters(:)).';
        if numel(xCenters) == size(evidenceMask, 2)
            evidenceMask(:, xCenters < minXCenterMeters) = false;
        end
    end
    evidenceMask = evidenceMask & ~completedMask;
    if ~any(evidenceMask(:))
        return;
    end

    yCenters = double(xyView.yCenters(:));
    referenceY = resolveRoadSeedReferenceY(xyView, roadSeedMask);
    sideByRow = sign(yCenters - referenceY);
    anchorMask = completedMask;
    selectedRows = find(any(anchorMask, 2));
    directionMode = "both";
    if isfield(cfg, "dominantBoundaryContinuationDirection") ...
            && ~isempty(cfg.dominantBoundaryContinuationDirection)
        directionMode = lower(string(cfg.dominantBoundaryContinuationDirection));
    end
    extendBackward = directionMode == "both" || directionMode == "backward" || directionMode == "negativex";
    extendForward = directionMode == "both" || directionMode == "forward" || directionMode == "positivex";
    for rowListIdx = 1:numel(selectedRows)
        rowIdx = selectedRows(rowListIdx);
        sideValue = sideByRow(rowIdx);
        if sideValue == 0
            continue;
        end

        anchorCols = find(anchorMask(rowIdx, :));
        if isempty(anchorCols)
            continue;
        end
        runStarts = [1, find(diff(anchorCols) > 1) + 1];
        runEnds = [runStarts(2:end) - 1, numel(anchorCols)];
        for runIdx = 1:numel(runStarts)
            leftCol = anchorCols(runStarts(runIdx));
            rightCol = anchorCols(runEnds(runIdx));
            if extendBackward
                completedMask = extendDominantBoundaryRunOneDirection( ...
                    completedMask, evidenceMask, yCenters, sideByRow, sideValue, ...
                    rowIdx, leftCol, -1, maxGapCells, searchRows);
            end
            if extendForward
                completedMask = extendDominantBoundaryRunOneDirection( ...
                    completedMask, evidenceMask, yCenters, sideByRow, sideValue, ...
                    rowIdx, rightCol, 1, maxGapCells, searchRows);
            end
            evidenceMask = evidenceMask & ~completedMask;
        end
    end
end

function completedMask = extendDominantBoundaryRunOneDirection(completedMask, evidenceMask, yCenters, sideByRow, sideValue, startRow, startCol, direction, maxGapCells, searchRows)
% extendDominantBoundaryRunOneDirection: Follow one accepted curb run
% endpoint through nearby evidence cells in a single x direction, allowing
% short gaps while keeping the row transition slope bounded by the search
% radius.
%
% Input:
%   completedMask: [Ny x Nx] logical accepted curb-cell raster
%   evidenceMask: [Ny x Nx] logical weak curb evidence raster
%   yCenters: [Ny x 1] double row center coordinates
%   sideByRow: [Ny x 1] double side sign for every row
%   sideValue: scalar side sign of the run being extended
%   startRow: scalar starting row index
%   startCol: scalar starting column index
%   direction: scalar x direction, either -1 or 1
%   maxGapCells: scalar maximum evidence-free columns to bridge
%   searchRows: scalar row search radius around the previous accepted row
%
% Output:
%   completedMask: [Ny x Nx] logical curb-cell raster after this extension
    numRows = size(completedMask, 1);
    numCols = size(completedMask, 2);
    previousRow = startRow;
    gapCount = 0;
    colIdx = startCol + direction;
    while colIdx >= 1 && colIdx <= numCols && gapCount <= maxGapCells
        rowWindow = (max(1, previousRow - searchRows):min(numRows, previousRow + searchRows)).';
        candidateRows = rowWindow(evidenceMask(rowWindow, colIdx) & sideByRow(rowWindow) == sideValue);
        if isempty(candidateRows)
            gapCount = gapCount + 1;
            colIdx = colIdx + direction;
            continue;
        end

        rowDistance = abs(candidateRows - previousRow);
        nearestRows = candidateRows(rowDistance == min(rowDistance));
        if sideValue > 0
            [~, roadFacingIdx] = min(yCenters(nearestRows));
        else
            [~, roadFacingIdx] = max(yCenters(nearestRows));
        end
        previousRow = nearestRows(roadFacingIdx);
        completedMask(previousRow, colIdx) = true;
        gapCount = 0;
        colIdx = colIdx + direction;
    end
end

function [refinedMask, promotedMask] = keepSameColumnFeaturePeakCurbCells(curbMask, energyMaps, xyView, roadSeedMask, cfg)
% keepSameColumnFeaturePeakCurbCells: Refine same-column curb cells
% using local feature peaks. Strong adjacent candidate peaks are promoted
% near already selected cells, while already selected weak shoulders are
% removed only when a neighboring same-side selected cell has clearly
% stronger height-step evidence. The refinement uses only local geometric
% evidence and road-side grouping rather than fixed coordinate bands.
%
% Input:
%   curbMask: [Ny x Nx] logical curb-cell raster
%   energyMaps: struct with totalBase, heightStepMeters, and linearity maps
%   xyView: XY view struct with y centers and y map
%   roadSeedMask: [Ny x Nx] logical road seed raster for side reference
%   cfg: struct from groundFeatureConfig().curb with same-column peak params
%
% Output:
%   refinedMask: [Ny x Nx] logical curb-cell raster after peak selection
%   promotedMask: [Ny x Nx] logical cells added by this refinement
    refinedMask = logical(curbMask);
    promotedMask = false(size(refinedMask));
    if ~isfield(cfg, "sameColumnFeaturePeakSelectionEnabled") ...
            || ~logical(cfg.sameColumnFeaturePeakSelectionEnabled) || ~any(refinedMask(:))
        return;
    end

    searchRadiusCells = 1;
    if isfield(cfg, "sameColumnFeaturePeakSearchRadiusCells")
        searchRadiusCells = max(0, round(double(cfg.sameColumnFeaturePeakSearchRadiusCells)));
    end
    minBaseEnergy = 0.25;
    if isfield(cfg, "sameColumnFeaturePeakMinBaseEnergy")
        minBaseEnergy = double(cfg.sameColumnFeaturePeakMinBaseEnergy);
    end
    minHeightStepMeters = 0.025;
    if isfield(cfg, "sameColumnFeaturePeakMinHeightStepMeters")
        minHeightStepMeters = double(cfg.sameColumnFeaturePeakMinHeightStepMeters);
    end
    minLinearity = 0.25;
    if isfield(cfg, "sameColumnFeaturePeakMinLinearity")
        minLinearity = double(cfg.sameColumnFeaturePeakMinLinearity);
    end
    promotionMinHeightGainMeters = 0.004;
    if isfield(cfg, "sameColumnFeaturePeakPromotionMinHeightGainMeters")
        promotionMinHeightGainMeters = double(cfg.sameColumnFeaturePeakPromotionMinHeightGainMeters);
    end
    promotionScoreRatio = 0.95;
    if isfield(cfg, "sameColumnFeaturePeakPromotionScoreRatio")
        promotionScoreRatio = min(1, max(0, double(cfg.sameColumnFeaturePeakPromotionScoreRatio)));
    end
    promotionBaseRatio = 0.75;
    if isfield(cfg, "sameColumnFeaturePeakPromotionBaseRatio")
        promotionBaseRatio = min(1, max(0, double(cfg.sameColumnFeaturePeakPromotionBaseRatio)));
    end
    promotionMinRoughnessMeters = 0.010;
    if isfield(cfg, "sameColumnFeaturePeakPromotionMinRoughnessMeters")
        promotionMinRoughnessMeters = double(cfg.sameColumnFeaturePeakPromotionMinRoughnessMeters);
    end
    promotionRoughnessRatio = 1.50;
    if isfield(cfg, "sameColumnFeaturePeakPromotionRoughnessRatio")
        promotionRoughnessRatio = max(1, double(cfg.sameColumnFeaturePeakPromotionRoughnessRatio));
    end
    roadFacingMinBaseEnergy = 0.32;
    if isfield(cfg, "sameColumnFeaturePeakRoadFacingMinBaseEnergy")
        roadFacingMinBaseEnergy = double(cfg.sameColumnFeaturePeakRoadFacingMinBaseEnergy);
    end
    roadFacingMinRoughnessMeters = 0.020;
    if isfield(cfg, "sameColumnFeaturePeakRoadFacingMinRoughnessMeters")
        roadFacingMinRoughnessMeters = double(cfg.sameColumnFeaturePeakRoadFacingMinRoughnessMeters);
    end
    roadFacingMinLinearity = 0.32;
    if isfield(cfg, "sameColumnFeaturePeakRoadFacingMinLinearity")
        roadFacingMinLinearity = double(cfg.sameColumnFeaturePeakRoadFacingMinLinearity);
    end
    roadFacingMinCenterEvidence = 0.90;
    if isfield(cfg, "sameColumnFeaturePeakRoadFacingMinCenterEvidence")
        roadFacingMinCenterEvidence = double(cfg.sameColumnFeaturePeakRoadFacingMinCenterEvidence);
    end
    suppressionMinHeightGainMeters = 0.012;
    if isfield(cfg, "sameColumnFeaturePeakSuppressionMinHeightGainMeters")
        suppressionMinHeightGainMeters = double(cfg.sameColumnFeaturePeakSuppressionMinHeightGainMeters);
    end
    suppressionScoreRatio = 1.25;
    if isfield(cfg, "sameColumnFeaturePeakSuppressionScoreRatio")
        suppressionScoreRatio = max(1, double(cfg.sameColumnFeaturePeakSuppressionScoreRatio));
    end
    strongCenterRoughnessMinMeters = roadFacingMinRoughnessMeters;

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    yCenters = double(xyView.yCenters(:));
    sideByRow = sign(yCenters - referenceY);
    roadFacingCandidateMask = double(energyMaps.totalBase) >= roadFacingMinBaseEnergy ...
        & double(energyMaps.roughnessMeters) >= roadFacingMinRoughnessMeters ...
        & double(energyMaps.linearity) >= roadFacingMinLinearity ...
        & double(energyMaps.linearityComponentCenterEvidence) >= roadFacingMinCenterEvidence;
    candidateMask = (double(energyMaps.totalBase) >= minBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= minHeightStepMeters ...
        & double(energyMaps.linearity) >= minLinearity) ...
        | roadFacingCandidateMask;
    peakScore = double(energyMaps.heightStepMeters) ...
        .* max(double(energyMaps.totalBase), 0) ...
        .* (1 + max(double(energyMaps.linearity), 0));

    numRows = size(refinedMask, 1);
    numCols = size(refinedMask, 2);
    originalMask = refinedMask;
    for colIdx = 1:numCols
        selectedRows = find(originalMask(:, colIdx));
        if isempty(selectedRows)
            continue;
        end

        for sideValue = [-1, 1]
            sideRows = selectedRows(sideByRow(selectedRows) == sideValue);
            if isempty(sideRows)
                continue;
            end

            rowSupport = false(numRows, 1);
            for rowIdx = reshape(sideRows, 1, [])
                rowWindow = max(1, rowIdx - searchRadiusCells):min(numRows, rowIdx + searchRadiusCells);
                rowSupport(rowWindow) = true;
            end
            candidateRows = find(rowSupport ...
                & candidateMask(:, colIdx) ...
                & (sideByRow == sideValue));
            if isempty(candidateRows)
                continue;
            end

            selectedHeight = double(energyMaps.heightStepMeters(sideRows, colIdx));
            selectedBase = double(energyMaps.totalBase(sideRows, colIdx));
            selectedRoughness = double(energyMaps.roughnessMeters(sideRows, colIdx));
            selectedPeakScore = peakScore(sideRows, colIdx);
            candidateHeight = double(energyMaps.heightStepMeters(candidateRows, colIdx));
            candidateBase = double(energyMaps.totalBase(candidateRows, colIdx));
            candidateRoughness = double(energyMaps.roughnessMeters(candidateRows, colIdx));
            candidateLinearity = double(energyMaps.linearity(candidateRows, colIdx));
            candidateCenterEvidence = double(energyMaps.linearityComponentCenterEvidence(candidateRows, colIdx));
            candidateDistance = abs(yCenters(candidateRows) - referenceY);
            selectedDistance = abs(yCenters(sideRows) - referenceY);
            candidatePeakScore = peakScore(candidateRows, colIdx);
            heightPromotionMask = candidateHeight >= max(selectedHeight) + promotionMinHeightGainMeters ...
                & candidatePeakScore >= max(selectedPeakScore) .* promotionScoreRatio ...
                & candidateBase >= max(selectedBase) .* promotionBaseRatio ...
                & candidateRoughness >= promotionMinRoughnessMeters;
            roughnessPromotionMask = candidateRoughness >= promotionMinRoughnessMeters ...
                & candidateRoughness >= max(selectedRoughness) .* promotionRoughnessRatio ...
                & candidateBase >= max(selectedBase) .* promotionBaseRatio;
            roadFacingPromotionMask = candidateDistance < max(selectedDistance) ...
                & candidateBase >= roadFacingMinBaseEnergy ...
                & candidateRoughness >= roadFacingMinRoughnessMeters ...
                & candidateLinearity >= roadFacingMinLinearity ...
                & candidateCenterEvidence >= roadFacingMinCenterEvidence;
            promoteRows = candidateRows(~originalMask(candidateRows, colIdx) ...
                & (heightPromotionMask | roughnessPromotionMask | roadFacingPromotionMask));
            refinedMask(promoteRows, colIdx) = true;
            promotedMask(promoteRows, colIdx) = true;
        end
    end

    selectionBeforeSuppression = refinedMask;
    for colIdx = 1:numCols
        selectedRows = find(selectionBeforeSuppression(:, colIdx));
        if isempty(selectedRows)
            continue;
        end

        for sideValue = [-1, 1]
            sideRows = selectedRows(sideByRow(selectedRows) == sideValue);
            if numel(sideRows) < 2
                continue;
            end

            sideHeights = double(energyMaps.heightStepMeters(sideRows, colIdx));
            sideScores = peakScore(sideRows, colIdx);
            sideDistances = abs(yCenters(sideRows) - referenceY);
            sideStrongCenterRoughness = double(energyMaps.totalBase(sideRows, colIdx)) >= roadFacingMinBaseEnergy ...
                & double(energyMaps.roughnessMeters(sideRows, colIdx)) >= strongCenterRoughnessMinMeters ...
                & double(energyMaps.linearityComponentCenterEvidence(sideRows, colIdx)) >= roadFacingMinCenterEvidence;
            for rowListIdx = 1:numel(sideRows)
                if promotedMask(sideRows(rowListIdx), colIdx)
                    continue;
                end

                neighborMask = abs(sideRows - sideRows(rowListIdx)) <= searchRadiusCells ...
                    & sideRows ~= sideRows(rowListIdx);
                closerNeighborMask = sideDistances < sideDistances(rowListIdx);
                neighborMask = neighborMask & closerNeighborMask;
                if ~any(neighborMask)
                    continue;
                end

                strongerNeighborMask = sideHeights(neighborMask) ...
                    >= sideHeights(rowListIdx) + suppressionMinHeightGainMeters;
                strongerNeighborMask = strongerNeighborMask ...
                    & sideScores(neighborMask) >= sideScores(rowListIdx) .* suppressionScoreRatio;
                if sideStrongCenterRoughness(rowListIdx)
                    strongerNeighborMask = strongerNeighborMask & sideStrongCenterRoughness(neighborMask);
                end
                if any(strongerNeighborMask)
                    refinedMask(sideRows(rowListIdx), colIdx) = false;
                end
            end
        end
    end
end

function protectionMask = keepRoadConsistentPeakPromotionProtection(peakPromotionMask, curbMask, energyMaps, xyView, roadSeedMask, cfg)
% keepRoadConsistentPeakPromotionProtection: Protect same-column
% promoted curb cells from local shadow thinning only when they are not a
% stronger outward height-step shoulder beyond an already selected
% road-facing cell. This keeps legitimate promoted curb grids whose
% height-step is consistent with the road-facing boundary, while allowing
% outer platform/sidewalk shoulders to be removed by the one-curb-per-side
% thinning rule.
%
% Input:
%   peakPromotionMask: [Ny x Nx] logical cells added by same-column
%       feature-peak promotion
%   curbMask: [Ny x Nx] logical current selected curb-cell raster
%   energyMaps: struct with heightStepMeters map
%   xyView: XY view struct with y centers and cell size
%   roadSeedMask: [Ny x Nx] logical road seed raster for side reference
%   cfg: struct from groundFeatureConfig().curb with protection parameters
%
% Output:
%   protectionMask: [Ny x Nx] logical promoted cells protected from local
%       shadow thinning
    protectionMask = logical(peakPromotionMask);
    if ~any(protectionMask(:)) || ~isfield(energyMaps, "heightStepMeters")
        return;
    end

    minSeparationCells = 0.75;
    if isfield(cfg, "sameSideCellShadowMinSeparationCells")
        minSeparationCells = max(0, double(cfg.sameSideCellShadowMinSeparationCells));
    end
    maxOuterHeightGainMeters = 0.012;
    if isfield(cfg, "sameColumnFeaturePeakOuterProtectionMaxHeightGainMeters")
        maxOuterHeightGainMeters = max(0, double(cfg.sameColumnFeaturePeakOuterProtectionMaxHeightGainMeters));
    end

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    yCenters = double(xyView.yCenters(:));
    cellHeight = max(double(xyView.cellSize(2)), eps);
    [promotedRows, promotedCols] = find(logical(peakPromotionMask));
    [selectedRows, selectedCols] = find(logical(curbMask));
    selectedSide = sign(yCenters(selectedRows) - referenceY);
    selectedDistance = abs(yCenters(selectedRows) - referenceY);
    promotedSide = sign(yCenters(promotedRows) - referenceY);
    promotedDistance = abs(yCenters(promotedRows) - referenceY);
    for promotedIdx = 1:numel(promotedRows)
        if promotedSide(promotedIdx) == 0
            continue;
        end

        roadFacingMask = selectedCols == promotedCols(promotedIdx) ...
            & selectedSide == promotedSide(promotedIdx) ...
            & selectedDistance < promotedDistance(promotedIdx) ...
            & ((promotedDistance(promotedIdx) - selectedDistance) ./ cellHeight) >= minSeparationCells;
        if ~any(roadFacingMask)
            continue;
        end

        closestDistance = min(selectedDistance(roadFacingMask));
        closestMask = roadFacingMask & (selectedDistance == closestDistance);
        closestRows = selectedRows(closestMask);
        closestCols = selectedCols(closestMask);
        currentHeight = double(energyMaps.heightStepMeters(promotedRows(promotedIdx), promotedCols(promotedIdx)));
        closestHeight = max(double(energyMaps.heightStepMeters(sub2ind(size(curbMask), closestRows, closestCols))));
        if currentHeight > closestHeight + maxOuterHeightGainMeters
            protectionMask(promotedRows(promotedIdx), promotedCols(promotedIdx)) = false;
        end
    end
end

function [completedMask, completionMask] = completeWeakCurbBoundaryGaps(curbMask, energyMaps, xyView, roadSeedMask, cfg)
% completeWeakCurbBoundaryGaps: Fill short gaps between already
% accepted curb anchors on the same road side when the intervening cells
% have weak but coherent curb evidence. This recovers sparse curb boundary
% cells whose normalized total energy is suppressed by local component
% support, while avoiding fixed coordinate priors by using only existing
% anchors, road-seed side, and per-cell feature evidence.
%
% Input:
%   curbMask: [Ny x Nx] logical filtered curb-cell raster
%   energyMaps: struct with totalBase, linearity, and heightStepMeters
%       evidence maps
%   xyView: XY view struct with y centers, y map, and cell size
%   roadSeedMask: [Ny x Nx] logical road seed raster for the side reference
%   cfg: struct from groundFeatureConfig().curb with gap-completion parameters
%
% Output:
%   completedMask: [Ny x Nx] logical curb-cell raster after short weak-gap
%       completion
%   completionMask: [Ny x Nx] logical cells added by weak-gap completion
    completedMask = logical(curbMask);
    completionMask = false(size(completedMask));
    if ~isfield(cfg, "boundaryGapCompletionEnabled") ...
            || ~logical(cfg.boundaryGapCompletionEnabled) || ~any(completedMask(:))
        return;
    end

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    maxGapCells = 6;
    if isfield(cfg, "boundaryGapCompletionMaxGapCells")
        maxGapCells = max(0, round(double(cfg.boundaryGapCompletionMaxGapCells)));
    end
    minBaseEnergy = 0.28;
    if isfield(cfg, "boundaryGapCompletionMinBaseEnergy")
        minBaseEnergy = double(cfg.boundaryGapCompletionMinBaseEnergy);
    end
    minLinearity = 0.30;
    if isfield(cfg, "boundaryGapCompletionMinLinearity")
        minLinearity = double(cfg.boundaryGapCompletionMinLinearity);
    end
    minHeightStepMeters = 0.025;
    if isfield(cfg, "boundaryGapCompletionMinHeightStepMeters")
        minHeightStepMeters = double(cfg.boundaryGapCompletionMinHeightStepMeters);
    end
    minRoughnessMeters = 0.003;
    if isfield(cfg, "boundaryGapCompletionMinRoughnessMeters")
        minRoughnessMeters = double(cfg.boundaryGapCompletionMinRoughnessMeters);
    end

    weakEvidenceMask = double(energyMaps.totalBase) >= minBaseEnergy ...
        & double(energyMaps.linearity) >= minLinearity ...
        & double(energyMaps.heightStepMeters) >= minHeightStepMeters;
    if isfield(energyMaps, "roughnessMeters")
        weakEvidenceMask = weakEvidenceMask & double(energyMaps.roughnessMeters) >= minRoughnessMeters;
    end

    yCenters = double(xyView.yCenters(:));
    numRows = size(completedMask, 1);
    for rowIdx = 1:numRows
        if sign(yCenters(rowIdx) - referenceY) == 0
            continue;
        end

        anchorCols = find(completedMask(rowIdx, :));
        if numel(anchorCols) < 2
            continue;
        end

        for anchorIdx = 1:(numel(anchorCols) - 1)
            leftCol = anchorCols(anchorIdx);
            rightCol = anchorCols(anchorIdx + 1);
            gapCells = rightCol - leftCol - 1;
            if gapCells < 1 || gapCells > maxGapCells
                continue;
            end

            fillCols = (leftCol + 1):(rightCol - 1);
            addedCols = fillCols(weakEvidenceMask(rowIdx, fillCols));
            completionMask(rowIdx, addedCols) = completionMask(rowIdx, addedCols) | ~completedMask(rowIdx, addedCols);
            completedMask(rowIdx, addedCols) = true;
        end
    end
end

function [completedMask, completionMask] = completeProjectedCurbBoundaryPaths(curbMask, energyMaps, xyView, roadSeedMask, cfg)
% completeProjectedCurbBoundaryPaths: Move selected curb anchors to
% the nearest weak curb evidence on the road-facing side of the same
% column, then bridge neighboring curb components with a slope-limited
% path through weak geometric support. This handles diagonal curb
% boundaries whose per-cell energy is sparse, while keeping the completion
% constrained by existing anchors, road-seed side, and local feature maps
% instead of fixed coordinate bands.
%
% Input:
%   curbMask: [Ny x Nx] logical filtered curb-cell raster
%   energyMaps: struct with totalBase, heightStepMeters, and roughnessMeters
%       evidence maps
%   xyView: XY view struct with y centers, y map, and cell size
%   roadSeedMask: [Ny x Nx] logical road seed raster for the side reference
%   cfg: struct from groundFeatureConfig().curb with boundary-path parameters
%
% Output:
%   completedMask: [Ny x Nx] logical curb-cell raster after road-facing
%       projection and diagonal path completion
%   completionMask: [Ny x Nx] logical cells added or moved by projected
%       path completion
    completedMask = logical(curbMask);
    completionMask = false(size(completedMask));
    if ~isfield(cfg, "boundaryPathCompletionEnabled") ...
            || ~logical(cfg.boundaryPathCompletionEnabled) || ~any(completedMask(:))
        return;
    end

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    projectionRadiusCells = 4;
    if isfield(cfg, "boundaryPathCompletionProjectionRadiusCells")
        projectionRadiusCells = max(0, round(double(cfg.boundaryPathCompletionProjectionRadiusCells)));
    end
    maxGapCells = 24;
    if isfield(cfg, "boundaryPathCompletionMaxGapCells")
        maxGapCells = max(0, round(double(cfg.boundaryPathCompletionMaxGapCells)));
    end
    maxSlopeRowsPerCol = 0.40;
    if isfield(cfg, "boundaryPathCompletionMaxSlopeRowsPerCol")
        maxSlopeRowsPerCol = max(0, double(cfg.boundaryPathCompletionMaxSlopeRowsPerCol));
    end
    radiusCells = 2;
    if isfield(cfg, "boundaryPathCompletionRadiusCells")
        radiusCells = max(0, round(double(cfg.boundaryPathCompletionRadiusCells)));
    end
    minBaseEnergy = 0.04;
    if isfield(cfg, "boundaryPathCompletionMinBaseEnergy")
        minBaseEnergy = double(cfg.boundaryPathCompletionMinBaseEnergy);
    end
    minHeightStepMeters = 0.014;
    if isfield(cfg, "boundaryPathCompletionMinHeightStepMeters")
        minHeightStepMeters = double(cfg.boundaryPathCompletionMinHeightStepMeters);
    end
    minLinearity = 0;
    if isfield(cfg, "boundaryPathCompletionMinLinearity")
        minLinearity = double(cfg.boundaryPathCompletionMinLinearity);
    end

    candidateMask = isfinite(double(energyMaps.roughnessMeters)) ...
        & double(energyMaps.totalBase) >= minBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= minHeightStepMeters ...
        & double(energyMaps.linearity) >= minLinearity;
    [completedMask, projectedCompletionMask] = projectCurbAnchorsToRoadFacingEvidence( ...
        completedMask, candidateMask, xyView, referenceY, projectionRadiusCells);
    completionMask = completionMask | projectedCompletionMask;

    components = connectedComponents8(completedMask);
    numComponents = numel(components);
    if numComponents < 2
        return;
    end

    yCenters = double(xyView.yCenters(:));
    componentSide = zeros(numComponents, 1);
    componentMinCol = zeros(numComponents, 1);
    componentMaxCol = zeros(numComponents, 1);
    componentLeftRow = zeros(numComponents, 1);
    componentRightRow = zeros(numComponents, 1);
    mapSize = size(completedMask);
    for componentIdx = 1:numComponents
        [rowIdx, colIdx] = ind2sub(mapSize, components{componentIdx});
        componentMedianY = median(yCenters(rowIdx), "omitnan");
        componentSide(componentIdx) = sign(componentMedianY - referenceY);
        componentMinCol(componentIdx) = min(colIdx);
        componentMaxCol(componentIdx) = max(colIdx);
        componentLeftRow(componentIdx) = round(median(rowIdx(colIdx == componentMinCol(componentIdx)), "omitnan"));
        componentRightRow(componentIdx) = round(median(rowIdx(colIdx == componentMaxCol(componentIdx)), "omitnan"));
    end

    for componentIdx = 1:numComponents
        bestNextComponentIdx = 0;
        bestGapCells = Inf;
        for nextComponentIdx = 1:numComponents
            if componentIdx == nextComponentIdx ...
                    || componentSide(componentIdx) == 0 ...
                    || componentSide(componentIdx) ~= componentSide(nextComponentIdx)
                continue;
            end

            gapCells = componentMinCol(nextComponentIdx) - componentMaxCol(componentIdx) - 1;
            if gapCells < 1 || gapCells > maxGapCells
                continue;
            end

            rowDiff = abs(componentLeftRow(nextComponentIdx) - componentRightRow(componentIdx));
            if rowDiff > ceil(maxSlopeRowsPerCol .* double(gapCells + 1)) + 1
                continue;
            end
            if gapCells < bestGapCells
                bestNextComponentIdx = nextComponentIdx;
                bestGapCells = gapCells;
            end
        end

        if bestNextComponentIdx == 0
            continue;
        end

        startCol = componentMaxCol(componentIdx);
        endCol = componentMinCol(bestNextComponentIdx);
        startRow = componentRightRow(componentIdx);
        endRow = componentLeftRow(bestNextComponentIdx);
        colSeq = startCol:endCol;
        rowSeq = startRow + ((double(colSeq) - double(startCol)) .* double(endRow - startRow) ./ double(endCol - startCol));
        for pathIdx = 1:numel(colSeq)
            targetRow = round(rowSeq(pathIdx));
            rowSeqLocal = max(1, targetRow - radiusCells):min(mapSize(1), targetRow + radiusCells);
            col = colSeq(pathIdx);
            fillRows = rowSeqLocal(candidateMask(rowSeqLocal, col));
            completionMask(fillRows, col) = completionMask(fillRows, col) | ~completedMask(fillRows, col);
            completedMask(fillRows, col) = true;
        end
    end
end

function [projectedMask, completionMask] = projectCurbAnchorsToRoadFacingEvidence(curbMask, candidateMask, xyView, referenceY, radiusCells)
% projectCurbAnchorsToRoadFacingEvidence: For each selected curb
% anchor, search a short same-column interval toward the road seed
% centerline and keep the first weak-evidence cell encountered. This
% changes outer duplicate anchors into their road-facing boundary cells
% before longer diagonal path completion is attempted.
%
% Input:
%   curbMask: [Ny x Nx] logical selected curb-cell raster
%   candidateMask: [Ny x Nx] logical weak curb evidence raster
%   xyView: XY view struct with y centers
%   referenceY: scalar road seed centerline y coordinate
%   radiusCells: scalar nonnegative same-column search radius in cells
%
% Output:
%   projectedMask: [Ny x Nx] logical projected curb-cell raster
%   completionMask: [Ny x Nx] logical cells moved to road-facing evidence
    projectedMask = false(size(curbMask));
    completionMask = false(size(curbMask));
    yCenters = double(xyView.yCenters(:));
    [selectedRows, selectedCols] = find(logical(curbMask));
    for cellIdx = 1:numel(selectedRows)
        rowIdx = selectedRows(cellIdx);
        colIdx = selectedCols(cellIdx);
        side = sign(yCenters(rowIdx) - referenceY);
        if side > 0
            searchRows = max(1, rowIdx - radiusCells):rowIdx;
            [~, orderIdx] = sort(yCenters(searchRows), "ascend");
            searchRows = searchRows(orderIdx);
        elseif side < 0
            searchRows = rowIdx:min(numel(yCenters), rowIdx + radiusCells);
            [~, orderIdx] = sort(yCenters(searchRows), "descend");
            searchRows = searchRows(orderIdx);
        else
            searchRows = rowIdx;
        end

        projectedRow = rowIdx;
        for searchIdx = 1:numel(searchRows)
            candidateRow = searchRows(searchIdx);
            if candidateMask(candidateRow, colIdx) || curbMask(candidateRow, colIdx)
                projectedRow = candidateRow;
                break;
            end
        end
        projectedMask(projectedRow, colIdx) = true;
        if projectedRow ~= rowIdx && ~curbMask(projectedRow, colIdx)
            completionMask(projectedRow, colIdx) = true;
        end
    end
end

function filteredMask = removeShadowedSameSideCurbComponents(curbMask, xyView, roadSeedMask, cfg)
% removeShadowedSameSideCurbComponents: Enforce a single curb
% component per road side over overlapping x columns. When two components
% lie on the same side of the ego-near road seed centerline and overlap
% substantially in x, the component farther from that centerline is
% removed as an outer duplicate. This keeps the left/right curb topology
% to one boundary per side without using fixed coordinate bands.
%
% Input:
%   curbMask: [Ny x Nx] logical filtered curb-cell raster
%   xyView: XY view struct with y centers, y map, and cell size
%   roadSeedMask: [Ny x Nx] logical road seed raster
%   cfg: struct from groundFeatureConfig().curb with duplicate parameters
%
% Output:
%   filteredMask: [Ny x Nx] logical curb-cell raster after same-side
%       duplicate suppression
    filteredMask = logical(curbMask);
    if ~isfield(cfg, "sameSideDuplicateSuppressionEnabled") ...
            || ~logical(cfg.sameSideDuplicateSuppressionEnabled) || ~any(filteredMask(:))
        return;
    end

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    minOverlapFraction = 0.25;
    if isfield(cfg, "sameSideDuplicateMinOverlapFraction")
        minOverlapFraction = min(max(double(cfg.sameSideDuplicateMinOverlapFraction), 0), 1);
    end
    minSeparationCells = 2;
    if isfield(cfg, "sameSideDuplicateMinSeparationCells")
        minSeparationCells = max(0, round(double(cfg.sameSideDuplicateMinSeparationCells)));
    end

    components = connectedComponents8(filteredMask);
    numComponents = numel(components);
    if numComponents < 2
        return;
    end

    yCenters = double(xyView.yCenters(:));
    cellHeight = max(double(xyView.cellSize(2)), eps);
    componentCols = cell(numComponents, 1);
    componentSide = zeros(numComponents, 1);
    componentDistance = zeros(numComponents, 1);
    componentMedianY = zeros(numComponents, 1);
    keepComponent = true(numComponents, 1);
    mapSize = size(filteredMask);
    for componentIdx = 1:numComponents
        [rowIdx, colIdx] = ind2sub(mapSize, components{componentIdx});
        componentCols{componentIdx} = unique(colIdx(:));
        componentY = yCenters(rowIdx);
        componentMedianY(componentIdx) = median(componentY, "omitnan");
        componentSide(componentIdx) = sign(componentMedianY(componentIdx) - referenceY);
        componentDistance(componentIdx) = abs(componentMedianY(componentIdx) - referenceY);
    end

    for outerIdx = 1:numComponents
        if componentSide(outerIdx) == 0
            continue;
        end
        for innerIdx = 1:numComponents
            if outerIdx == innerIdx || componentSide(outerIdx) ~= componentSide(innerIdx)
                continue;
            end
            if componentDistance(outerIdx) <= componentDistance(innerIdx)
                continue;
            end

            overlapCount = numel(intersect(componentCols{outerIdx}, componentCols{innerIdx}));
            overlapFraction = double(overlapCount) / double(max(1, numel(componentCols{outerIdx})));
            separationCells = abs(componentMedianY(outerIdx) - componentMedianY(innerIdx)) / cellHeight;
            if overlapFraction >= minOverlapFraction && separationCells >= minSeparationCells
                keepComponent(outerIdx) = false;
            end
        end
    end

    filteredMask(:) = false;
    for componentIdx = 1:numComponents
        if keepComponent(componentIdx)
            filteredMask(components{componentIdx}) = true;
        end
    end
end

function filteredMask = removeShadowedSameSideCurbCells(curbMask, xyView, roadSeedMask, cfg, protectedMask, energyMaps)
% removeShadowedSameSideCurbCells: Thin locally adjacent curb cells
% on each road side to the road-facing boundary cell. If a selected cell
% has another selected cell on the same side within a short x-column
% radius and the other cell is closer to the road-seed centerline, the
% farther cell is removed. This enforces one curb grid per side locally
% without relying on fixed coordinate rectangles.
%
% Input:
%   curbMask: [Ny x Nx] logical curb-cell raster
%   xyView: XY view struct with y centers, y map, and cell size
%   roadSeedMask: [Ny x Nx] logical road seed raster for the side reference
%   cfg: struct from groundFeatureConfig().curb with cell-shadow parameters
%   protectedMask: optional [Ny x Nx] logical raster of boundary-completion
%       cells that should not be removed by local shadow thinning
%   energyMaps: optional struct with total, roughnessMeters, linearity, and
%       linearityComponentCenterEvidence maps for feature competition
%
% Output:
%   filteredMask: [Ny x Nx] logical curb-cell raster after road-facing
%       local thinning
    filteredMask = logical(curbMask);
    if nargin < 5 || isempty(protectedMask)
        protectedMask = false(size(filteredMask));
    else
        protectedMask = logical(protectedMask);
    end
    if ~isfield(cfg, "sameSideCellShadowSuppressionEnabled") ...
            || ~logical(cfg.sameSideCellShadowSuppressionEnabled) || ~any(filteredMask(:))
        return;
    end

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    radiusCells = 1;
    if isfield(cfg, "sameSideCellShadowRadiusCells")
        radiusCells = max(0, round(double(cfg.sameSideCellShadowRadiusCells)));
    end
    minSeparationCells = 0.75;
    if isfield(cfg, "sameSideCellShadowMinSeparationCells")
        minSeparationCells = max(0, double(cfg.sameSideCellShadowMinSeparationCells));
    end
    featureRoughnessScaleMeters = 0.020;
    if isfield(cfg, "sameSideCellShadowFeatureRoughnessScaleMeters")
        featureRoughnessScaleMeters = max(eps, double(cfg.sameSideCellShadowFeatureRoughnessScaleMeters));
    end
    dominantScoreRatio = 1.25;
    if isfield(cfg, "sameSideCellShadowDominantScoreRatio")
        dominantScoreRatio = max(1, double(cfg.sameSideCellShadowDominantScoreRatio));
    end
    dominantRoughnessGainMeters = 0.004;
    if isfield(cfg, "sameSideCellShadowDominantRoughnessGainMeters")
        dominantRoughnessGainMeters = max(0, double(cfg.sameSideCellShadowDominantRoughnessGainMeters));
    end
    smoothRoughnessMaxMeters = 0.010;
    if isfield(cfg, "sameSideCellShadowSmoothRoughnessMaxMeters")
        smoothRoughnessMaxMeters = max(0, double(cfg.sameSideCellShadowSmoothRoughnessMaxMeters));
    end
    reliableCloserCenterEvidenceMin = 0.90;
    if isfield(cfg, "sameSideCellShadowReliableCloserCenterEvidenceMin")
        reliableCloserCenterEvidenceMin = double(cfg.sameSideCellShadowReliableCloserCenterEvidenceMin);
    end
    reliableCloserDominantTotalMin = 0.40;
    if isfield(cfg, "sameSideCellShadowReliableCloserDominantTotalMin")
        reliableCloserDominantTotalMin = max(0, double(cfg.sameSideCellShadowReliableCloserDominantTotalMin));
    end
    useDominantProtection = nargin >= 6 ...
        && isstruct(energyMaps) ...
        && isfield(energyMaps, "total") ...
        && isfield(energyMaps, "roughnessMeters") ...
        && isfield(energyMaps, "linearity") ...
        && isfield(energyMaps, "linearityComponentCenterEvidence");

    yCenters = double(xyView.yCenters(:));
    cellHeight = max(double(xyView.cellSize(2)), eps);
    [selectedRows, selectedCols] = find(filteredMask);
    selectedSide = sign(yCenters(selectedRows) - referenceY);
    selectedDistance = abs(yCenters(selectedRows) - referenceY);
    selectedScore = ones(numel(selectedRows), 1);
    selectedRoughness = zeros(numel(selectedRows), 1);
    selectedTotal = ones(numel(selectedRows), 1);
    selectedCenterEvidence = zeros(numel(selectedRows), 1);
    if useDominantProtection
        selectedLinearIdx = sub2ind(size(filteredMask), selectedRows, selectedCols);
        selectedTotal = max(double(energyMaps.total(selectedLinearIdx)), 0);
        selectedRoughness = max(double(energyMaps.roughnessMeters(selectedLinearIdx)), 0);
        selectedLinearity = max(double(energyMaps.linearity(selectedLinearIdx)), 0);
        selectedCenterEvidence = max(double(energyMaps.linearityComponentCenterEvidence(selectedLinearIdx)), 0);
        selectedScore = selectedTotal ...
            .* (1 + (selectedRoughness ./ featureRoughnessScaleMeters)) ...
            .* (1 + selectedLinearity);
    end
    keepCell = true(numel(selectedRows), 1);
    for cellIdx = 1:numel(selectedRows)
        if selectedSide(cellIdx) == 0
            continue;
        end

        neighborMask = abs(selectedCols - selectedCols(cellIdx)) <= radiusCells ...
            & selectedSide == selectedSide(cellIdx) ...
            & ((abs(selectedDistance - selectedDistance(cellIdx))) ./ cellHeight) >= minSeparationCells;
        if useDominantProtection
            fartherCellMask = neighborMask & selectedDistance > selectedDistance(cellIdx);
            strongerFartherMask = fartherCellMask ...
                & selectedScore >= selectedScore(cellIdx) .* dominantScoreRatio ...
                & selectedRoughness >= selectedRoughness(cellIdx) + dominantRoughnessGainMeters;
            if selectedRoughness(cellIdx) <= smoothRoughnessMaxMeters && any(strongerFartherMask)
                keepCell(cellIdx) = false;
                continue;
            end
        elseif protectedMask(selectedRows(cellIdx), selectedCols(cellIdx))
            continue;
        end

        closerCellMask = neighborMask & selectedDistance < selectedDistance(cellIdx);
        if any(closerCellMask)
            suppressCell = true;
            if useDominantProtection
                closerScore = max(selectedScore(closerCellMask));
                closerRoughness = max(selectedRoughness(closerCellMask));
                reliableCloserMask = closerCellMask ...
                    & selectedCenterEvidence >= reliableCloserCenterEvidenceMin;
                dominantFeature = selectedScore(cellIdx) >= closerScore .* dominantScoreRatio ...
                    && selectedRoughness(cellIdx) >= closerRoughness + dominantRoughnessGainMeters;
                if any(reliableCloserMask)
                    dominantFeature = dominantFeature ...
                        && selectedTotal(cellIdx) >= reliableCloserDominantTotalMin;
                end
                suppressCell = ~dominantFeature;
            end
            keepCell(cellIdx) = ~suppressCell;
        elseif protectedMask(selectedRows(cellIdx), selectedCols(cellIdx))
            continue;
        end
    end

    filteredMask(:) = false;
    filteredMask(sub2ind(size(filteredMask), selectedRows(keepCell), selectedCols(keepCell))) = true;
end

function recoveredMask = recoverRoadFacingShoulderCurbCells(curbMask, rawExtractedMask, energyMaps, xyView, roadSeedMask, cfg)
% recoverRoadFacingShoulderCurbCells: Restore one-cell shoulder
% evidence immediately adjacent to an already accepted curb cell when the
% shoulder cell has strong raw base, height-step, and line evidence. The
% road-facing side uses stricter line evidence, while the road-away side
% also requires a rough accepted anchor and enough total energy. The
% recovery handles curb samples that straddle an XY grid boundary without
% allowing distant parallel road markings or sidewalk shoulders to become
% curb cells.
%
% Input:
%   curbMask: [Ny x Nx] logical curb-cell raster after duplicate thinning
%   rawExtractedMask: [Ny x Nx] logical raw curb evidence raster before
%       road-adjacency filtering
%   energyMaps: struct with totalBase, heightStepMeters, linearity, and
%       linearityComponentCenterEvidence maps
%   xyView: XY view struct with y centers and y map
%   roadSeedMask: [Ny x Nx] logical road seed raster for side reference
%   cfg: struct from groundFeatureConfig().curb with shoulder recovery params
%
% Output:
%   recoveredMask: [Ny x Nx] logical curb-cell raster after recovery
    recoveredMask = logical(curbMask);
    if ~isfield(cfg, "roadFacingShoulderRecoveryEnabled") ...
            || ~logical(cfg.roadFacingShoulderRecoveryEnabled) || ~any(recoveredMask(:))
        return;
    end

    minBaseEnergy = 0.55;
    if isfield(cfg, "roadFacingShoulderRecoveryMinBaseEnergy")
        minBaseEnergy = double(cfg.roadFacingShoulderRecoveryMinBaseEnergy);
    end
    minHeightStepMeters = 0.055;
    if isfield(cfg, "roadFacingShoulderRecoveryMinHeightStepMeters")
        minHeightStepMeters = double(cfg.roadFacingShoulderRecoveryMinHeightStepMeters);
    end
    minLinearity = 0.45;
    if isfield(cfg, "roadFacingShoulderRecoveryMinLinearity")
        minLinearity = double(cfg.roadFacingShoulderRecoveryMinLinearity);
    end
    lineShoulderMinBaseEnergy = 0.30;
    if isfield(cfg, "roadFacingLineShoulderRecoveryMinBaseEnergy")
        lineShoulderMinBaseEnergy = double(cfg.roadFacingLineShoulderRecoveryMinBaseEnergy);
    end
    lineShoulderMinHeightStepMeters = 0.055;
    if isfield(cfg, "roadFacingLineShoulderRecoveryMinHeightStepMeters")
        lineShoulderMinHeightStepMeters = double(cfg.roadFacingLineShoulderRecoveryMinHeightStepMeters);
    end
    lineShoulderMinLinearity = 0.45;
    if isfield(cfg, "roadFacingLineShoulderRecoveryMinLinearity")
        lineShoulderMinLinearity = double(cfg.roadFacingLineShoulderRecoveryMinLinearity);
    end
    anchorCenterEvidenceMin = 0.90;
    if isfield(cfg, "roadFacingShoulderRecoveryAnchorCenterEvidenceMin")
        anchorCenterEvidenceMin = double(cfg.roadFacingShoulderRecoveryAnchorCenterEvidenceMin);
    end
    roadAwayMinBaseEnergy = 0.75;
    if isfield(cfg, "roadAwayShoulderRecoveryMinBaseEnergy")
        roadAwayMinBaseEnergy = double(cfg.roadAwayShoulderRecoveryMinBaseEnergy);
    end
    roadAwayMinTotalEnergy = 0.35;
    if isfield(cfg, "roadAwayShoulderRecoveryMinTotalEnergy")
        roadAwayMinTotalEnergy = double(cfg.roadAwayShoulderRecoveryMinTotalEnergy);
    end
    roadAwayMinHeightStepMeters = 0.050;
    if isfield(cfg, "roadAwayShoulderRecoveryMinHeightStepMeters")
        roadAwayMinHeightStepMeters = double(cfg.roadAwayShoulderRecoveryMinHeightStepMeters);
    end
    roadAwayMinLinearity = 0.35;
    if isfield(cfg, "roadAwayShoulderRecoveryMinLinearity")
        roadAwayMinLinearity = double(cfg.roadAwayShoulderRecoveryMinLinearity);
    end
    roadAwayAnchorRoughnessMinMeters = 0.035;
    if isfield(cfg, "roadAwayShoulderRecoveryAnchorRoughnessMinMeters")
        roadAwayAnchorRoughnessMinMeters = double(cfg.roadAwayShoulderRecoveryAnchorRoughnessMinMeters);
    end
    roadAwayLineMinBaseEnergy = 0.45;
    if isfield(cfg, "roadAwayLineShoulderRecoveryMinBaseEnergy")
        roadAwayLineMinBaseEnergy = double(cfg.roadAwayLineShoulderRecoveryMinBaseEnergy);
    end
    roadAwayLineMinHeightStepMeters = 0.055;
    if isfield(cfg, "roadAwayLineShoulderRecoveryMinHeightStepMeters")
        roadAwayLineMinHeightStepMeters = double(cfg.roadAwayLineShoulderRecoveryMinHeightStepMeters);
    end
    roadAwayLineMinLinearity = 0.45;
    if isfield(cfg, "roadAwayLineShoulderRecoveryMinLinearity")
        roadAwayLineMinLinearity = double(cfg.roadAwayLineShoulderRecoveryMinLinearity);
    end
    roadAwayLineMinCenterEvidence = 0.90;
    if isfield(cfg, "roadAwayLineShoulderRecoveryMinCenterEvidence")
        roadAwayLineMinCenterEvidence = double(cfg.roadAwayLineShoulderRecoveryMinCenterEvidence);
    end

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    yCenters = double(xyView.yCenters(:));
    sideByRow = sign(yCenters - referenceY);
    strongAnchorMask = recoveredMask ...
        & double(energyMaps.linearityComponentCenterEvidence) >= anchorCenterEvidenceMin;
    rawShoulderEvidenceMask = logical(rawExtractedMask) ...
        & ~recoveredMask ...
        & double(energyMaps.totalBase) >= minBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= minHeightStepMeters ...
        & double(energyMaps.linearity) >= minLinearity;
    lineShoulderEvidenceMask = logical(rawExtractedMask) ...
        & ~recoveredMask ...
        & double(energyMaps.totalBase) >= lineShoulderMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= lineShoulderMinHeightStepMeters ...
        & double(energyMaps.linearity) >= lineShoulderMinLinearity;
    roadAwayAnchorMask = strongAnchorMask ...
        & double(energyMaps.roughnessMeters) >= roadAwayAnchorRoughnessMinMeters;
    roadAwayShoulderEvidenceMask = logical(rawExtractedMask) ...
        & ~recoveredMask ...
        & double(energyMaps.total) >= roadAwayMinTotalEnergy ...
        & double(energyMaps.totalBase) >= roadAwayMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= roadAwayMinHeightStepMeters ...
        & double(energyMaps.linearity) >= roadAwayMinLinearity;
    roadAwayLineShoulderEvidenceMask = logical(rawExtractedMask) ...
        & ~recoveredMask ...
        & double(energyMaps.totalBase) >= roadAwayLineMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= roadAwayLineMinHeightStepMeters ...
        & double(energyMaps.linearity) >= roadAwayLineMinLinearity ...
        & double(energyMaps.linearityComponentCenterEvidence) >= roadAwayLineMinCenterEvidence;
    shoulderMask = false(size(recoveredMask));
    for rowIdx = 1:size(recoveredMask, 1)
        if sideByRow(rowIdx) > 0
            anchorRow = rowIdx + 1;
            roadAwayAnchorRow = rowIdx - 1;
        elseif sideByRow(rowIdx) < 0
            anchorRow = rowIdx - 1;
            roadAwayAnchorRow = rowIdx + 1;
        else
            continue;
        end
        if anchorRow < 1 || anchorRow > size(recoveredMask, 1)
            anchorRow = NaN;
        end
        if roadAwayAnchorRow < 1 || roadAwayAnchorRow > size(recoveredMask, 1)
            roadAwayAnchorRow = NaN;
        end
        if isfinite(anchorRow)
            shoulderMask(rowIdx, :) = shoulderMask(rowIdx, :) ...
                | ((rawShoulderEvidenceMask(rowIdx, :) | lineShoulderEvidenceMask(rowIdx, :)) ...
                & strongAnchorMask(anchorRow, :));
        end
        if isfinite(roadAwayAnchorRow)
            shoulderMask(rowIdx, :) = shoulderMask(rowIdx, :) ...
                | (roadAwayShoulderEvidenceMask(rowIdx, :) & roadAwayAnchorMask(roadAwayAnchorRow, :)) ...
                | (roadAwayLineShoulderEvidenceMask(rowIdx, :) & recoveredMask(roadAwayAnchorRow, :));
        end
    end
    recoveredMask = recoveredMask | shoulderMask;
end

function completedMask = completeBoundaryEvidenceRuns(curbMask, energyMaps, ~, ~, cfg)
% completeBoundaryEvidenceRuns: Complete short same-row curb runs
% after duplicate thinning by filling small holes and extending short
% endpoints through center-supported curb evidence. The completion uses
% local feature masks and existing accepted anchors rather than fixed
% coordinate bands.
%
% Input:
%   curbMask: [Ny x Nx] logical accepted curb-cell raster
%   energyMaps: struct with totalBase, heightStepMeters, roughnessMeters,
%       linearity, linearityComponentCenterEvidence, and extraction support
%       masks
%   ~: unused XY view argument kept for call-site consistency
%   ~: unused road seed mask argument kept for call-site consistency
%   cfg: struct from groundFeatureConfig().curb with run-completion params
%
% Output:
%   completedMask: [Ny x Nx] logical curb-cell raster after run completion
    completedMask = logical(curbMask);
    if ~isfield(cfg, "boundaryRunCompletionEnabled") ...
            || ~logical(cfg.boundaryRunCompletionEnabled) || ~any(completedMask(:))
        return;
    end

    maxGapCells = 8;
    if isfield(cfg, "boundaryRunCompletionMaxGapCells")
        maxGapCells = max(0, round(double(cfg.boundaryRunCompletionMaxGapCells)));
    end
    maxEndpointCells = 4;
    if isfield(cfg, "boundaryRunCompletionMaxEndpointCells")
        maxEndpointCells = max(0, round(double(cfg.boundaryRunCompletionMaxEndpointCells)));
    end
    noEvidenceMaxGapCells = 1;
    if isfield(cfg, "boundaryRunCompletionNoEvidenceMaxGapCells")
        noEvidenceMaxGapCells = max(0, round(double(cfg.boundaryRunCompletionNoEvidenceMaxGapCells)));
    end
    minCenterEvidence = 0.90;
    if isfield(cfg, "boundaryRunCompletionMinCenterEvidence")
        minCenterEvidence = double(cfg.boundaryRunCompletionMinCenterEvidence);
    end
    minBaseEnergy = 0.33;
    if isfield(cfg, "boundaryRunCompletionMinBaseEnergy")
        minBaseEnergy = double(cfg.boundaryRunCompletionMinBaseEnergy);
    end
    minHeightStepMeters = 0.016;
    if isfield(cfg, "boundaryRunCompletionMinHeightStepMeters")
        minHeightStepMeters = double(cfg.boundaryRunCompletionMinHeightStepMeters);
    end
    minRoughnessMeters = 0.035;
    if isfield(cfg, "boundaryRunCompletionMinRoughnessMeters")
        minRoughnessMeters = double(cfg.boundaryRunCompletionMinRoughnessMeters);
    end
    minLinearity = 0.25;
    if isfield(cfg, "boundaryRunCompletionMinLinearity")
        minLinearity = double(cfg.boundaryRunCompletionMinLinearity);
    end
    stepRoughMinBaseEnergy = 0.20;
    if isfield(cfg, "boundaryRunCompletionStepRoughMinBaseEnergy")
        stepRoughMinBaseEnergy = double(cfg.boundaryRunCompletionStepRoughMinBaseEnergy);
    end
    stepRoughMinHeightStepMeters = 0.080;
    if isfield(cfg, "boundaryRunCompletionStepRoughMinHeightStepMeters")
        stepRoughMinHeightStepMeters = double(cfg.boundaryRunCompletionStepRoughMinHeightStepMeters);
    end
    stepRoughMinRoughnessMeters = 0.045;
    if isfield(cfg, "boundaryRunCompletionStepRoughMinRoughnessMeters")
        stepRoughMinRoughnessMeters = double(cfg.boundaryRunCompletionStepRoughMinRoughnessMeters);
    end
    lineStepMinBaseEnergy = 0.30;
    if isfield(cfg, "boundaryRunCompletionLineStepMinBaseEnergy")
        lineStepMinBaseEnergy = double(cfg.boundaryRunCompletionLineStepMinBaseEnergy);
    end
    lineStepMinHeightStepMeters = 0.055;
    if isfield(cfg, "boundaryRunCompletionLineStepMinHeightStepMeters")
        lineStepMinHeightStepMeters = double(cfg.boundaryRunCompletionLineStepMinHeightStepMeters);
    end
    lineStepMinLinearity = 0.45;
    if isfield(cfg, "boundaryRunCompletionLineStepMinLinearity")
        lineStepMinLinearity = double(cfg.boundaryRunCompletionLineStepMinLinearity);
    end
    lineStepMinCenterEvidence = 0.00;
    if isfield(cfg, "boundaryRunCompletionLineStepMinCenterEvidence")
        lineStepMinCenterEvidence = double(cfg.boundaryRunCompletionLineStepMinCenterEvidence);
    end
    strongCenterLineStepMinBaseEnergy = 0.75;
    if isfield(cfg, "boundaryRunCompletionStrongCenterLineStepMinBaseEnergy")
        strongCenterLineStepMinBaseEnergy = double(cfg.boundaryRunCompletionStrongCenterLineStepMinBaseEnergy);
    end
    strongCenterLineStepMinHeightStepMeters = 0.055;
    if isfield(cfg, "boundaryRunCompletionStrongCenterLineStepMinHeightStepMeters")
        strongCenterLineStepMinHeightStepMeters = double(cfg.boundaryRunCompletionStrongCenterLineStepMinHeightStepMeters);
    end
    strongCenterLineStepMinLinearity = 0.35;
    if isfield(cfg, "boundaryRunCompletionStrongCenterLineStepMinLinearity")
        strongCenterLineStepMinLinearity = double(cfg.boundaryRunCompletionStrongCenterLineStepMinLinearity);
    end
    rawLineStepMinCenterEvidence = 0.50;
    if isfield(cfg, "boundaryRunCompletionRawLineStepMinCenterEvidence")
        rawLineStepMinCenterEvidence = double(cfg.boundaryRunCompletionRawLineStepMinCenterEvidence);
    end
    rawLineStepMinBaseEnergy = 0.35;
    if isfield(cfg, "boundaryRunCompletionRawLineStepMinBaseEnergy")
        rawLineStepMinBaseEnergy = double(cfg.boundaryRunCompletionRawLineStepMinBaseEnergy);
    end
    rawLineStepMinHeightStepMeters = 0.055;
    if isfield(cfg, "boundaryRunCompletionRawLineStepMinHeightStepMeters")
        rawLineStepMinHeightStepMeters = double(cfg.boundaryRunCompletionRawLineStepMinHeightStepMeters);
    end
    rawLineStepMinLinearity = 0.40;
    if isfield(cfg, "boundaryRunCompletionRawLineStepMinLinearity")
        rawLineStepMinLinearity = double(cfg.boundaryRunCompletionRawLineStepMinLinearity);
    end
    endpointLineStepMinCenterEvidence = 0.10;
    if isfield(cfg, "boundaryRunCompletionEndpointLineStepMinCenterEvidence")
        endpointLineStepMinCenterEvidence = double(cfg.boundaryRunCompletionEndpointLineStepMinCenterEvidence);
    end
    endpointLineStepMinBaseEnergy = 0.35;
    if isfield(cfg, "boundaryRunCompletionEndpointLineStepMinBaseEnergy")
        endpointLineStepMinBaseEnergy = double(cfg.boundaryRunCompletionEndpointLineStepMinBaseEnergy);
    end
    endpointLineStepMinHeightStepMeters = 0.055;
    if isfield(cfg, "boundaryRunCompletionEndpointLineStepMinHeightStepMeters")
        endpointLineStepMinHeightStepMeters = double(cfg.boundaryRunCompletionEndpointLineStepMinHeightStepMeters);
    end
    endpointLineStepMinLinearity = 0.35;
    if isfield(cfg, "boundaryRunCompletionEndpointLineStepMinLinearity")
        endpointLineStepMinLinearity = double(cfg.boundaryRunCompletionEndpointLineStepMinLinearity);
    end

    extractionEvidenceMask = false(size(completedMask));
    if isfield(energyMaps, "extractionStandaloneSeedMask")
        extractionEvidenceMask = extractionEvidenceMask | logical(energyMaps.extractionStandaloneSeedMask);
    end
    if isfield(energyMaps, "extractionStandaloneBridgeMask")
        extractionEvidenceMask = extractionEvidenceMask | logical(energyMaps.extractionStandaloneBridgeMask);
    end
    if isfield(energyMaps, "extractionFillMask")
        extractionEvidenceMask = extractionEvidenceMask | logical(energyMaps.extractionFillMask);
    end
    if isfield(energyMaps, "extractionEndpointMask")
        extractionEvidenceMask = extractionEvidenceMask | logical(energyMaps.extractionEndpointMask);
    end
    centerMask = double(energyMaps.linearityComponentCenterEvidence) >= minCenterEvidence;
    centerRoughMask = centerMask ...
        & double(energyMaps.totalBase) >= minBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= minHeightStepMeters ...
        & double(energyMaps.roughnessMeters) >= minRoughnessMeters ...
        & double(energyMaps.linearity) >= minLinearity;
    centerStepRoughMask = centerMask ...
        & double(energyMaps.totalBase) >= stepRoughMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= stepRoughMinHeightStepMeters ...
        & double(energyMaps.roughnessMeters) >= stepRoughMinRoughnessMeters;
    centerLineStepMask = centerMask ...
        & double(energyMaps.totalBase) >= lineStepMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= lineStepMinHeightStepMeters ...
        & double(energyMaps.linearity) >= lineStepMinLinearity;
    evidenceMask = centerRoughMask | centerStepRoughMask | centerLineStepMask;
    gapLineStepMask = extractionEvidenceMask ...
        & double(energyMaps.linearityComponentCenterEvidence) >= lineStepMinCenterEvidence ...
        & double(energyMaps.totalBase) >= lineStepMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= lineStepMinHeightStepMeters ...
        & double(energyMaps.linearity) >= lineStepMinLinearity;
    strongCenterLineStepMask = centerMask ...
        & double(energyMaps.totalBase) >= strongCenterLineStepMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= strongCenterLineStepMinHeightStepMeters ...
        & double(energyMaps.linearity) >= strongCenterLineStepMinLinearity;
    rawLineStepMask = extractionEvidenceMask ...
        & double(energyMaps.linearityComponentCenterEvidence) >= rawLineStepMinCenterEvidence ...
        & double(energyMaps.totalBase) >= rawLineStepMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= rawLineStepMinHeightStepMeters ...
        & double(energyMaps.linearity) >= rawLineStepMinLinearity;
    endpointLineStepMask = double(energyMaps.linearityComponentCenterEvidence) >= endpointLineStepMinCenterEvidence ...
        & double(energyMaps.totalBase) >= endpointLineStepMinBaseEnergy ...
        & double(energyMaps.heightStepMeters) >= endpointLineStepMinHeightStepMeters ...
        & double(energyMaps.linearity) >= endpointLineStepMinLinearity;
    gapEvidenceMask = evidenceMask | gapLineStepMask | strongCenterLineStepMask | rawLineStepMask;
    endpointEvidenceMask = evidenceMask | endpointLineStepMask;

    for rowIdx = 1:size(completedMask, 1)
        anchorCols = find(completedMask(rowIdx, :));
        if isempty(anchorCols)
            continue;
        end

        firstCol = anchorCols(1);
        leftCols = max(1, firstCol - maxEndpointCells):(firstCol - 1);
        for colIdx = fliplr(leftCols)
            if ~endpointEvidenceMask(rowIdx, colIdx)
                break;
            end
            completedMask(rowIdx, colIdx) = true;
        end

        lastCol = anchorCols(end);
        rightCols = (lastCol + 1):min(size(completedMask, 2), lastCol + maxEndpointCells);
        for colIdx = rightCols
            if ~endpointEvidenceMask(rowIdx, colIdx)
                break;
            end
            completedMask(rowIdx, colIdx) = true;
        end

        anchorCols = find(completedMask(rowIdx, :));
        if numel(anchorCols) < 2
            continue;
        end
        for anchorIdx = 1:(numel(anchorCols) - 1)
            leftCol = anchorCols(anchorIdx);
            rightCol = anchorCols(anchorIdx + 1);
            gapCells = rightCol - leftCol - 1;
            if gapCells < 1 || gapCells > maxGapCells
                continue;
            end
            fillCols = (leftCol + 1):(rightCol - 1);
            if gapCells <= noEvidenceMaxGapCells
                completedMask(rowIdx, fillCols) = true;
            else
                completedMask(rowIdx, fillCols(gapEvidenceMask(rowIdx, fillCols))) = true;
            end
        end
    end
end

function filteredMask = removeFartherSameSideOutlierComponents(curbMask, xyView, roadSeedMask, cfg)
% removeFartherSameSideOutlierComponents: Remove small same-side
% curb components that sit much farther from the road seed centerline than
% a larger reference component on the same side. This suppresses isolated
% parallel off-road ridges after the main curb boundary has been recovered.
%
% Input:
%   curbMask: [Ny x Nx] logical curb-cell raster
%   xyView: XY view struct with y centers, y map, and cell size
%   roadSeedMask: [Ny x Nx] logical road seed raster for side reference
%   cfg: struct from groundFeatureConfig().curb with outlier params
%
% Output:
%   filteredMask: [Ny x Nx] logical curb-cell raster after outlier removal
    filteredMask = logical(curbMask);
    if ~isfield(cfg, "sameSideOutlierSuppressionEnabled") ...
            || ~logical(cfg.sameSideOutlierSuppressionEnabled) || ~any(filteredMask(:))
        return;
    end

    referenceMinCells = 6;
    if isfield(cfg, "sameSideOutlierReferenceMinCells")
        referenceMinCells = max(1, round(double(cfg.sameSideOutlierReferenceMinCells)));
    end
    maxComponentCells = 8;
    if isfield(cfg, "sameSideOutlierMaxComponentCells")
        maxComponentCells = max(1, round(double(cfg.sameSideOutlierMaxComponentCells)));
    end
    maxColumnSpanCells = Inf;
    if isfield(cfg, "sameSideOutlierMaxColumnSpanCells")
        maxColumnSpanCells = max(1, round(double(cfg.sameSideOutlierMaxColumnSpanCells)));
    end
    maxExtraDistanceCells = 6;
    if isfield(cfg, "sameSideOutlierMaxExtraDistanceCells")
        maxExtraDistanceCells = max(0, double(cfg.sameSideOutlierMaxExtraDistanceCells));
    end

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    components = connectedComponents8(filteredMask);
    if numel(components) < 2
        return;
    end

    yCenters = double(xyView.yCenters(:));
    cellHeight = max(double(xyView.cellSize(2)), eps);
    mapSize = size(filteredMask);
    componentSide = zeros(numel(components), 1);
    componentDistance = zeros(numel(components), 1);
    componentCells = zeros(numel(components), 1);
    componentColumnSpan = zeros(numel(components), 1);
    for componentIdx = 1:numel(components)
        [rowIdx, colIdx] = ind2sub(mapSize, components{componentIdx});
        componentSide(componentIdx) = sign(median(yCenters(rowIdx), "omitnan") - referenceY);
        componentDistance(componentIdx) = abs(median(yCenters(rowIdx), "omitnan") - referenceY);
        componentCells(componentIdx) = numel(rowIdx);
        componentColumnSpan(componentIdx) = max(colIdx) - min(colIdx) + 1;
    end

    keepComponent = true(numel(components), 1);
    for sideValue = [-1, 1]
        sideComponentMask = componentSide == sideValue;
        referenceMask = sideComponentMask & componentCells >= referenceMinCells;
        if ~any(referenceMask)
            continue;
        end
        referenceDistance = min(componentDistance(referenceMask));
        outlierMask = sideComponentMask ...
            & (componentCells <= maxComponentCells | componentColumnSpan <= maxColumnSpanCells) ...
            & ((componentDistance - referenceDistance) ./ cellHeight) > maxExtraDistanceCells;
        keepComponent(outlierMask) = false;
    end

    filteredMask(:) = false;
    for componentIdx = 1:numel(components)
        if keepComponent(componentIdx)
            filteredMask(components{componentIdx}) = true;
        end
    end
end

function filteredMask = removeRoadFacingDuplicateCurbCells(curbMask, energyMaps, xyView, roadSeedMask, cfg)
% removeRoadFacingDuplicateCurbCells: Remove road-facing curb cells
% when a same-column road-away neighbor has stronger curb evidence. This
% final thinning pass handles cells reintroduced by shoulder recovery or
% run completion and keeps one curb grid per road side without fixed
% coordinate bands.
%
% Input:
%   curbMask: [Ny x Nx] logical curb-cell raster
%   energyMaps: struct with totalBase, heightStepMeters, linearity,
%       linearityComponentCenterEvidence, and extraction support masks
%   xyView: XY view struct with y centers and y map
%   roadSeedMask: [Ny x Nx] logical road seed raster for side reference
%   cfg: struct from groundFeatureConfig().curb with duplicate-thinning params
%
% Output:
%   filteredMask: [Ny x Nx] logical curb-cell raster after road-facing
%       duplicate suppression
    filteredMask = logical(curbMask);
    if ~isfield(cfg, "sameSideRoadFacingDuplicateSuppressionEnabled") ...
            || ~logical(cfg.sameSideRoadFacingDuplicateSuppressionEnabled) || ~any(filteredMask(:))
        return;
    end

    searchRows = 2;
    if isfield(cfg, "sameSideRoadFacingDuplicateSearchRowsCells")
        searchRows = max(1, round(double(cfg.sameSideRoadFacingDuplicateSearchRowsCells)));
    end
    outerCenterEvidenceMin = 0.90;
    if isfield(cfg, "sameSideRoadFacingDuplicateOuterCenterEvidenceMin")
        outerCenterEvidenceMin = double(cfg.sameSideRoadFacingDuplicateOuterCenterEvidenceMin);
    end
    outerLinearityMin = 0.85;
    if isfield(cfg, "sameSideRoadFacingDuplicateOuterLinearityMin")
        outerLinearityMin = double(cfg.sameSideRoadFacingDuplicateOuterLinearityMin);
    end
    outerStepLinearityMin = 0.35;
    if isfield(cfg, "sameSideRoadFacingDuplicateOuterStepLinearityMin")
        outerStepLinearityMin = double(cfg.sameSideRoadFacingDuplicateOuterStepLinearityMin);
    end
    innerLinearityMax = 0.65;
    if isfield(cfg, "sameSideRoadFacingDuplicateInnerLinearityMax")
        innerLinearityMax = double(cfg.sameSideRoadFacingDuplicateInnerLinearityMax);
    end
    innerStepLinearityMax = 0.48;
    if isfield(cfg, "sameSideRoadFacingDuplicateInnerStepLinearityMax")
        innerStepLinearityMax = double(cfg.sameSideRoadFacingDuplicateInnerStepLinearityMax);
    end
    innerRoughnessMaxMeters = 0.040;
    if isfield(cfg, "sameSideRoadFacingDuplicateInnerRoughnessMaxMeters")
        innerRoughnessMaxMeters = double(cfg.sameSideRoadFacingDuplicateInnerRoughnessMaxMeters);
    end
    innerCenterEvidenceMax = 0.05;
    if isfield(cfg, "sameSideRoadFacingDuplicateInnerCenterEvidenceMax")
        innerCenterEvidenceMax = double(cfg.sameSideRoadFacingDuplicateInnerCenterEvidenceMax);
    end
    outerBaseRatio = 0.80;
    if isfield(cfg, "sameSideRoadFacingDuplicateOuterBaseRatio")
        outerBaseRatio = double(cfg.sameSideRoadFacingDuplicateOuterBaseRatio);
    end
    outerBaseGainRatio = 1.20;
    if isfield(cfg, "sameSideRoadFacingDuplicateOuterBaseGainRatio")
        outerBaseGainRatio = double(cfg.sameSideRoadFacingDuplicateOuterBaseGainRatio);
    end
    outerHeightGainMeters = 0.015;
    if isfield(cfg, "sameSideRoadFacingDuplicateOuterHeightGainMeters")
        outerHeightGainMeters = double(cfg.sameSideRoadFacingDuplicateOuterHeightGainMeters);
    end

    extractionEvidenceMask = false(size(filteredMask));
    if isfield(energyMaps, "extractionStandaloneSeedMask")
        extractionEvidenceMask = extractionEvidenceMask | logical(energyMaps.extractionStandaloneSeedMask);
    end
    if isfield(energyMaps, "extractionStandaloneBridgeMask")
        extractionEvidenceMask = extractionEvidenceMask | logical(energyMaps.extractionStandaloneBridgeMask);
    end
    if isfield(energyMaps, "extractionFillMask")
        extractionEvidenceMask = extractionEvidenceMask | logical(energyMaps.extractionFillMask);
    end
    if isfield(energyMaps, "extractionEndpointMask")
        extractionEvidenceMask = extractionEvidenceMask | logical(energyMaps.extractionEndpointMask);
    end

    yMap = double(xyView.yMap);
    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    referenceY = 0;
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    yCenters = double(xyView.yCenters(:));
    sideByRow = sign(yCenters - referenceY);
    keepMask = true(size(filteredMask));
    [selectedRows, selectedCols] = find(filteredMask);
    for selectedIdx = 1:numel(selectedRows)
        rowIdx = selectedRows(selectedIdx);
        colIdx = selectedCols(selectedIdx);
        sideValue = sideByRow(rowIdx);
        if sideValue == 0
            continue;
        end

        if sideValue > 0
            outerRows = (rowIdx + 1):min(size(filteredMask, 1), rowIdx + searchRows);
        else
            outerRows = max(1, rowIdx - searchRows):(rowIdx - 1);
        end
        if isempty(outerRows)
            continue;
        end

        outerEvidenceMask = (filteredMask(outerRows, colIdx) | extractionEvidenceMask(outerRows, colIdx)) ...
            & double(energyMaps.linearityComponentCenterEvidence(outerRows, colIdx)) >= outerCenterEvidenceMin;
        if ~any(outerEvidenceMask)
            continue;
        end

        outerBase = double(energyMaps.totalBase(outerRows, colIdx));
        outerHeight = double(energyMaps.heightStepMeters(outerRows, colIdx));
        outerRoughness = double(energyMaps.roughnessMeters(outerRows, colIdx));
        outerLinearity = double(energyMaps.linearity(outerRows, colIdx));
        selectedBase = double(energyMaps.totalBase(rowIdx, colIdx));
        selectedHeight = double(energyMaps.heightStepMeters(rowIdx, colIdx));
        selectedRoughness = double(energyMaps.roughnessMeters(rowIdx, colIdx));
        selectedLinearity = double(energyMaps.linearity(rowIdx, colIdx));
        selectedCenterEvidence = double(energyMaps.linearityComponentCenterEvidence(rowIdx, colIdx));
        centerSuppressionAllowed = selectedCenterEvidence <= innerCenterEvidenceMax || sideValue < 0;
        outerLineDominantMask = outerEvidenceMask ...
            & outerLinearity >= outerLinearityMin ...
            & centerSuppressionAllowed ...
            & selectedLinearity <= innerLinearityMax ...
            & outerBase >= selectedBase .* outerBaseRatio ...
            & (selectedRoughness <= innerRoughnessMaxMeters ...
            | outerRoughness >= selectedRoughness .* 0.80 ...
            | outerHeight >= selectedHeight + outerHeightGainMeters);
        outerStepDominantMask = outerEvidenceMask ...
            & outerLinearity >= outerStepLinearityMin ...
            & centerSuppressionAllowed ...
            & selectedLinearity <= innerStepLinearityMax ...
            & selectedRoughness <= innerRoughnessMaxMeters ...
            & outerBase >= selectedBase .* outerBaseGainRatio ...
            & outerHeight >= selectedHeight + outerHeightGainMeters;
        if any(outerLineDominantMask | outerStepDominantMask)
            keepMask(rowIdx, colIdx) = false;
        end
    end

    filteredMask = filteredMask & keepMask;
end

function supported = isSmallRoadAdjacentCurbComponentSupported(cellIdx, energyMaps, cfg)
% isSmallRoadAdjacentCurbComponentSupported: Decide whether a small
% road-adjacent curb component has enough intrinsic high-energy evidence
% to be retained as a whole. This prevents short endpoint-only or
% bridge-only fragments from being promoted solely because they fall near
% the grown road surface, while still allowing compact components with at
% least one strong energy anchor.
%
% Input:
%   cellIdx: [N x 1] linear indices of one connected curb component
%   energyMaps: struct with total energy raster
%   cfg: struct from groundFeatureConfig().curb with small-component
%       confirmation parameters
%
% Output:
%   supported: logical true when the component has enough strong cells
    strongEnergyThreshold = double(cfg.extractionEnergyThreshold);
    if isfield(cfg, "roadAdjacencySmallComponentStrongEnergyThreshold")
        strongEnergyThreshold = double(cfg.roadAdjacencySmallComponentStrongEnergyThreshold);
    end
    minStrongCells = 1;
    if isfield(cfg, "roadAdjacencySmallComponentMinStrongCells")
        minStrongCells = max(1, round(double(cfg.roadAdjacencySmallComponentMinStrongCells)));
    end

    componentEnergy = double(energyMaps.total(cellIdx));
    supported = nnz(componentEnergy >= strongEnergyThreshold) >= minStrongCells;
end

function anchorMask = keepRoadFacingBoundaryAnchors(cellIdx, anchorMask, roadCellMask, mapSize, cfg, energyMaps)
% keepRoadFacingBoundaryAnchors: Thin large-component road-adjacent
% anchors to one boundary per component column. For each component column,
% the retained anchor row is selected by the local curb-feature peak when
% available, with road-facing proximity as a fallback. This suppresses
% wider sidewalk/platform rows that happen to fall inside the road-
% adjacency dilation while preserving the strongest curb grid.
%
% Input:
%   cellIdx: [N x 1] linear indices of one connected curb component
%   anchorMask: [N x 1] logical road-adjacent component anchors
%   roadCellMask: [Ny x Nx] logical grown road-surface raster
%   mapSize: [1 x 2] size of the [Ny x Nx] curb-cell raster
%   cfg: struct from groundFeatureConfig().curb with anchor parameters
%   energyMaps: struct with heightStepMeters, totalBase, and linearity maps
%
% Output:
%   anchorMask: [N x 1] logical road-facing component anchors
    anchorMask = logical(anchorMask(:));
    if ~isfield(cfg, "roadAdjacencyRoadFacingAnchorEnabled") ...
            || ~logical(cfg.roadAdjacencyRoadFacingAnchorEnabled) || ~any(anchorMask)
        return;
    end

    anchorRadiusCells = 1;
    if isfield(cfg, "roadAdjacencyRoadFacingAnchorRadiusCells")
        anchorRadiusCells = max(0, round(double(cfg.roadAdjacencyRoadFacingAnchorRadiusCells)));
    end
    roadReferenceRadiusCells = 4;
    if isfield(cfg, "roadAdjacencyRoadReferenceRadiusCells")
        roadReferenceRadiusCells = max(0, round(double(cfg.roadAdjacencyRoadReferenceRadiusCells)));
    elseif isfield(cfg, "roadAdjacencyRadiusCells")
        roadReferenceRadiusCells = max(0, round(double(cfg.roadAdjacencyRadiusCells)));
    end

    [componentRows, componentCols] = ind2sub(mapSize, cellIdx(:));
    [roadRows, roadCols] = find(logical(roadCellMask));
    if isempty(roadRows)
        return;
    end

    thinnedAnchorMask = false(size(anchorMask));
    anchorCols = unique(componentCols(anchorMask));
    for k = 1:numel(anchorCols)
        colIdx = anchorCols(k);
        componentInCol = componentCols == colIdx;
        anchorInCol = componentInCol & anchorMask;
        roadInWindow = abs(roadCols - colIdx) <= roadReferenceRadiusCells;
        if ~any(roadInWindow)
            thinnedAnchorMask(anchorInCol) = true;
            continue;
        end

        roadReferenceRow = median(double(roadRows(roadInWindow)), "omitnan");
        anchorRows = double(componentRows(anchorInCol));
        if nargin >= 6 && isfield(cfg, "roadAdjacencyFeaturePeakAnchorEnabled") ...
                && logical(cfg.roadAdjacencyFeaturePeakAnchorEnabled) ...
                && isfield(energyMaps, "heightStepMeters") ...
                && isfield(energyMaps, "totalBase") ...
                && isfield(energyMaps, "linearity")
            anchorLinearIdx = sub2ind(mapSize, componentRows(anchorInCol), componentCols(anchorInCol));
            anchorScore = double(energyMaps.heightStepMeters(anchorLinearIdx)) ...
                .* max(double(energyMaps.totalBase(anchorLinearIdx)), 0) ...
                .* (1 + max(double(energyMaps.linearity(anchorLinearIdx)), 0));
            maxScore = max(anchorScore);
            peakRows = anchorRows(anchorScore == maxScore);
            if numel(peakRows) > 1
                nearestDistance = min(abs(peakRows - roadReferenceRow));
                peakRows = peakRows(abs(peakRows - roadReferenceRow) == nearestDistance);
            end
            boundaryRow = median(peakRows, "omitnan");
        else
            nearestDistance = min(abs(anchorRows - roadReferenceRow));
            nearestRows = anchorRows(abs(anchorRows - roadReferenceRow) == nearestDistance);
            boundaryRow = median(nearestRows, "omitnan");
        end
        thinnedAnchorMask(anchorInCol) = abs(double(componentRows(anchorInCol)) - boundaryRow) <= anchorRadiusCells;
    end
    anchorMask = thinnedAnchorMask;
end

function continuationMask = buildRoadAdjacencyBoundaryContinuation(cellIdx, anchorMask, mapSize, cfg)
% buildRoadAdjacencyBoundaryContinuation: Extend a large curb
% component along its road-facing boundary by interpolating the row
% sequence of road-adjacent anchor cells across component columns. The
% continuation remains inside the same component and only fills cells near
% that dynamic boundary, so elongated curbs can pass through short gaps in
% seeded road growth without keeping the whole component.
%
% Input:
%   cellIdx: [N x 1] linear indices of one connected curb component
%   anchorMask: [N x 1] logical mask of road-adjacent cells to use as
%       boundary anchors
%   mapSize: [1 x 2] size of the [Ny x Nx] curb-cell raster
%   cfg: struct from groundFeatureConfig().curb with continuation parameters
%
% Output:
%   continuationMask: [N x 1] logical mask of component cells retained by
%       boundary continuation
    continuationMask = false(size(cellIdx));
    if ~isfield(cfg, "roadAdjacencyBoundaryContinuationEnabled") ...
            || ~logical(cfg.roadAdjacencyBoundaryContinuationEnabled)
        return;
    end

    anchorMask = logical(anchorMask(:));
    if ~any(anchorMask)
        return;
    end

    radiusCells = 1;
    if isfield(cfg, "roadAdjacencyBoundaryContinuationRadiusCells")
        radiusCells = max(0, round(double(cfg.roadAdjacencyBoundaryContinuationRadiusCells)));
    end
    maxGapCells = Inf;
    if isfield(cfg, "roadAdjacencyBoundaryContinuationMaxGapCells")
        maxGapCells = max(0, round(double(cfg.roadAdjacencyBoundaryContinuationMaxGapCells)));
    end
    minAnchorCols = 2;
    if isfield(cfg, "roadAdjacencyBoundaryContinuationMinAnchorCols")
        minAnchorCols = max(2, round(double(cfg.roadAdjacencyBoundaryContinuationMinAnchorCols)));
    end

    [componentRows, componentCols] = ind2sub(mapSize, cellIdx(:));
    [anchorRows, anchorCols] = ind2sub(mapSize, cellIdx(anchorMask));
    uniqueAnchorCols = unique(anchorCols(:));
    if numel(uniqueAnchorCols) < minAnchorCols
        return;
    end

    anchorRowByCol = NaN(numel(uniqueAnchorCols), 1);
    for k = 1:numel(uniqueAnchorCols)
        rowsInCol = anchorRows(anchorCols == uniqueAnchorCols(k));
        anchorRowByCol(k) = median(double(rowsInCol), "omitnan");
    end

    uniqueComponentCols = unique(componentCols(:));
    minAnchorCol = min(uniqueAnchorCols);
    maxAnchorCol = max(uniqueAnchorCols);
    for k = 1:numel(uniqueComponentCols)
        colIdx = uniqueComponentCols(k);
        if colIdx < minAnchorCol - maxGapCells || colIdx > maxAnchorCol + maxGapCells
            continue;
        end

        if any(uniqueAnchorCols == colIdx)
            targetRow = anchorRowByCol(uniqueAnchorCols == colIdx);
        else
            targetRow = interp1(double(uniqueAnchorCols), anchorRowByCol, double(colIdx), "linear", "extrap");
        end
        if ~isfinite(targetRow)
            continue;
        end

        componentInCol = componentCols == colIdx;
        continuationMask(componentInCol) = abs(double(componentRows(componentInCol)) - targetRow) <= radiusCells;
    end
end
