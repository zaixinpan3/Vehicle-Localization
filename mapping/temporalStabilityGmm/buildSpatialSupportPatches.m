function [patchLocalIndices, patchComponents, removedPatchCount] = buildSpatialSupportPatches(classPoints, sourceIndices, timestampBins, params, pcaEpsilon)
% buildSpatialSupportPatches: Build class-local spatial support candidates
% outside EM. Patches are connected components of a radius-pruned mutual
% kNN graph, recursively split along their dominant PCA axis until each
% patch is compact, filtered to a buildable point count, and converted into
% PCA-oriented initialization components with patch-level temporal
% diagnostics. The patches influence reliability scoring, sampling,
% model order, and EM initialization, never the EM updates themselves.
%
% Input:
%   classPoints: [N x 2] BEV points of one semantic class
%   sourceIndices: [N x 1] indices of classPoints in the full input
%   timestampBins: [N x 1] observation-bin ids per point
%   params: resolved class parameter struct
%   pcaEpsilon: scalar covariance regularization
%
% Output:
%   patchLocalIndices: {P x 1} cell of local point indices per patch
%   patchComponents: [P x 1] initialization component struct array
%   removedPatchCount: scalar number of unbuildable patches removed
    patchLocalIndices = buildMutualKnnPatches(classPoints, params, pcaEpsilon);
    [patchLocalIndices, removedPatchCount] = filterBuildablePatches(patchLocalIndices, max(2, round(double(params.minComponentPoints))));
    patchComponents = repmat(emptyComponent(), numel(patchLocalIndices), 1);
    for patchIdx = 1:numel(patchLocalIndices)
        localIndices = patchLocalIndices{patchIdx};
        patchComponents(patchIdx) = buildComponent(classPoints(localIndices, :), sourceIndices(localIndices), timestampBins(localIndices), params, pcaEpsilon);
    end
end

function patchLocalIndices = buildMutualKnnPatches(points, params, pcaEpsilon)
% buildMutualKnnPatches: Build initial class-local patches as connected
% components of the mutual-kNN graph with radius pruning, then recursively
% split each component along its dominant PCA axis until every final patch
% satisfies the configured Euclidean diameter bound.
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   params: class parameter struct with k, radius, and maxDiameter fields
%   pcaEpsilon: scalar covariance diagonal regularization
%
% Output:
%   patchLocalIndices: cell array of class-local point index vectors
    pointCount = size(points, 1);

    if pointCount == 0
        patchLocalIndices = cell(0, 1);
        return;
    end

    if pointCount == 1
        patchLocalIndices = {1};
        return;
    end

    kEff = min(params.k, pointCount - 1);

    if kEff == 0
        componentIds = (1:pointCount).';
        componentCount = pointCount;
    else
        [neighborIdx, neighborDistance] = knnSearchExcludingSelf(points, kEff);
        sourceIdx = repmat((1:pointCount).', 1, kEff);
        validNeighborMask = neighborIdx > 0 & neighborDistance <= params.radius;
        directedAdjacency = sparse(sourceIdx(validNeighborMask), neighborIdx(validNeighborMask), true, pointCount, pointCount);
        adjacency = directedAdjacency & directedAdjacency.';
        graphObj = graph(adjacency, "upper");
        componentIds = conncomp(graphObj).';
        componentCount = max(componentIds);
    end

    patchLocalIndices = cell(pointCount, 1);
    patchCount = 0;
    for componentIdx = 1:componentCount
        componentLocalIndices = find(componentIds == componentIdx);
        splitPatches = splitPatchByDiameter(points, componentLocalIndices, params.maxDiameter, pcaEpsilon);
        nextPatchIdx = patchCount + (1:numel(splitPatches));
        patchLocalIndices(nextPatchIdx, 1) = splitPatches(:);
        patchCount = patchCount + numel(splitPatches);
    end
    patchLocalIndices = patchLocalIndices(1:patchCount);
end

function [neighborIdx, neighborDistance] = knnSearchExcludingSelf(points, kEff)
% knnSearchExcludingSelf: Query a KD-tree for each point's nearest
% spatial neighbors while removing the point itself from its neighbor list.
%
% Input:
%   points: [N x 2] point coordinates
%   kEff: scalar number of non-self neighbors to return
%
% Output:
%   neighborIdx: [N x kEff] neighbor point indices, zero where unavailable
%   neighborDistance: [N x kEff] Euclidean neighbor distances
    pointCount = size(points, 1);
    assert(exist("KDTreeSearcher", "class") == 8, ...
        "KDTreeSearcher is required for optimized mutual-kNN patch construction.");
    searcher = KDTreeSearcher(points);
    [rawIdx, rawDistance] = knnsearch(searcher, points, "K", min(pointCount, kEff + 1));
    neighborIdx = zeros(pointCount, kEff);
    neighborDistance = inf(pointCount, kEff);

    for pointIdx = 1:pointCount
        rowIdx = rawIdx(pointIdx, :);
        rowDistance = rawDistance(pointIdx, :);
        nonSelfMask = rowIdx ~= pointIdx;
        rowIdx = rowIdx(nonSelfMask);
        rowDistance = rowDistance(nonSelfMask);
        keepCount = min(kEff, numel(rowIdx));
        if keepCount > 0
            neighborIdx(pointIdx, 1:keepCount) = rowIdx(1:keepCount);
            neighborDistance(pointIdx, 1:keepCount) = rowDistance(1:keepCount);
        end
    end
end

function splitPatches = splitPatchByDiameter(points, localIndices, maxDiameter, pcaEpsilon)
% splitPatchByDiameter: Recursively split a candidate patch along the
% patch dominant PCA axis at the median projection until each output patch
% has diameter less than or equal to the configured maximum.
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   localIndices: vector of indices for one candidate patch
%   maxDiameter: scalar Euclidean patch diameter limit
%   pcaEpsilon: scalar covariance diagonal regularization
%
% Output:
%   splitPatches: cell array of class-local index vectors after splitting
    maxQueueLength = max(1, 2 .* numel(localIndices) - 1);
    queue = cell(maxQueueLength, 1);
    queue{1} = localIndices(:);
    queueCount = 1;
    splitPatches = cell(numel(localIndices), 1);
    patchCount = 0;
    headIdx = 1;

    while headIdx <= queueCount
        currentIndices = queue{headIdx};
        headIdx = headIdx + 1;

        if numel(currentIndices) <= 1 || patchDiameter(points(currentIndices, :)) <= maxDiameter
            patchCount = patchCount + 1;
            splitPatches{patchCount, 1} = currentIndices;
        else
            [t, ~] = pcaDirections(points(currentIndices, :), pcaEpsilon);
            alpha = points(currentIndices, :) * t;
            alphaMedian = median(alpha);
            minusIndices = currentIndices(alpha <= alphaMedian);
            plusIndices = currentIndices(alpha > alphaMedian);

            if isempty(minusIndices) || isempty(plusIndices)
                [~, orderIdx] = sort(alpha, "ascend");
                splitAt = floor(numel(currentIndices) / 2);
                minusIndices = currentIndices(orderIdx(1:splitAt));
                plusIndices = currentIndices(orderIdx(splitAt + 1:end));
            end

            queueCount = queueCount + 1;
            queue{queueCount, 1} = minusIndices(:);
            queueCount = queueCount + 1;
            queue{queueCount, 1} = plusIndices(:);
        end
    end

    splitPatches = splitPatches(1:patchCount);
end

function diameter = patchDiameter(points)
% patchDiameter: Compute the bounding-box diagonal upper bound for a
% candidate patch diameter, returning zero for singleton or empty patches.
%
% Input:
%   points: [N x 2] patch point coordinates
%
% Output:
%   diameter: scalar bounding-box diagonal upper bound
    if size(points, 1) <= 1
        diameter = 0;
    else
        span = max(points, [], 1) - min(points, [], 1);
        diameter = hypot(span(1), span(2));
    end
end

function [patchLocalIndices, removedPatchCount] = filterBuildablePatches(patchLocalIndices, minPatchPointCount)
% filterBuildablePatches: Remove initialization patches whose point
% count is too small to construct the PCA-oriented covariance used by
% EM GMM component initialization.
%
% Input:
%   patchLocalIndices: cell array of class-local patch index vectors
%   minPatchPointCount: scalar minimum retained patch point count
%
% Output:
%   patchLocalIndices: retained cell array of buildable patch index vectors
%   removedPatchCount: scalar number of removed patches
    if isempty(patchLocalIndices)
        removedPatchCount = 0;
        return;
    end
    patchSizes = cellfun(@numel, patchLocalIndices(:));
    keepMask = patchSizes >= minPatchPointCount;
    removedPatchCount = nnz(~keepMask);
    patchLocalIndices = patchLocalIndices(keepMask);
end

function component = buildComponent(points, sourceIndices, timestampBins, params, pcaEpsilon)
% buildComponent: Convert one final semantic-geometric patch into an EM
% mixture initialization component with PCA-oriented covariance geometry, patch
% provenance, and patch-level temporal diagnostics. The initial temporal
% diagnostics are retained only for debugging and do not affect the integrated
% likelihood, responsibilities, M-step, covariance update, mixture-weight
% update, or convergence logic.
%
% Input:
%   points: [Q x 2] patch point coordinates
%   sourceIndices: [Q x 1] original input indices for patch points
%   timestampBins: [Q x 1] discrete observation bin labels for the patch
%   params: validated class parameter struct
%   pcaEpsilon: scalar covariance diagonal regularization
%
% Output:
%   component: scalar structured Gaussian support component
    [t, n] = pcaDirections(points, pcaEpsilon);
    centroid = mean(points, 1);
    minPoint = min(points, [], 1);
    maxPoint = max(points, [], 1);
    boundingBox = [minPoint, maxPoint];
    covariance = params.lengthParallel.^2 .* (t * t.') + params.lengthPerp.^2 .* (n * n.') + pcaEpsilon .* eye(2);
    [covariance, covarianceDiagnostics] = applyCovarianceEigenvalueSafeguards(covariance, t, n, params);
    invCovariance = invertCovariance(covariance, pcaEpsilon);
    [temporalDiversity, timestampBinCount, effectiveFrameCount, frameCounts] = patchTemporalDiversity(timestampBins, params);
    [normalStability, normalDispersion] = patchNormalStability(points, timestampBins, centroid, n, params);
    reliability = temporalDiversity .* normalStability;
    sampleSufficiency = computeSampleSufficiency(sum(saturateFrameSupport(frameCounts(:), params)), params);
    supportAmplitude = clipUnit(reliability .* sampleSufficiency);

    component = emptyComponent();
    component.points = points;
    component.sourceIndices = sourceIndices(:);
    component.timestampBins = timestampBins(:);
    component.patchPoints = points;
    component.patchSourceIndices = sourceIndices(:);
    component.patchTimestampBins = timestampBins(:);
    component.patchBoundingBox = boundingBox;
    component.patchPointCount = size(points, 1);
    component.timestampBinCount = 0;
    component.temporalDiversity = 0;
    component.normalStability = 1;
    component.normalDispersion = 0;
    component.supportAmplitude = 0;
    component.reliability = 0;
    component.sampleSufficiency = 0;
    component.initialTimestampBinCount = timestampBinCount;
    component.initialTemporalDiversity = temporalDiversity;
    component.initialNormalStability = normalStability;
    component.initialNormalDispersion = normalDispersion;
    component.initialSupportAmplitude = supportAmplitude;
    component.initialReliability = reliability;
    component.initialSampleSufficiency = sampleSufficiency;
    component.initialCentroid = centroid;
    component.centroid = centroid;
    component.mean = centroid;
    component.mixtureWeight = 0;
    component.initialMixtureWeight = 0;
    component.emMixtureWeightBeforePruning = 0;
    component.emResponsibilityPointCount = 0;
    component.emRobustWeightedPointCount = 0;
    component.emRobustScaleWeightMin = nan;
    component.emRobustScaleWeightMean = nan;
    component.emRobustScaleWeightMax = nan;
    component.emForegroundResponsibilityPointCount = 0;
    component.emNumericalFreezeApplied = false;
    component.emNumericalEffectiveSupportThreshold = params.emMinEffectiveSupport;
    component.covariance = covariance;
    component.invCovariance = invCovariance;
    component = storeCovarianceDiagnostics(component, covarianceDiagnostics, params);
    component.t = t;
    component.n = n;
    component.boundingBox = zeros(1, 4);
    component.pointCount = 0;
    component.supportBoundingBox = zeros(1, 4);
    component.effectiveSupportPointCount = 0;
    component.assignedSupportPointCount = 0;
    component.effectiveFrameCount = effectiveFrameCount;
    component.frameCounts = frameCounts(:);
    component.patchDiagnosticTimestampBinCount = timestampBinCount;
    component.patchDiagnosticTemporalDiversity = temporalDiversity;
    component.patchDiagnosticNormalStability = normalStability;
    component.patchDiagnosticNormalDispersion = normalDispersion;
    component.patchDiagnosticReliability = reliability;
    component.patchDiagnosticSampleSufficiency = sampleSufficiency;
    component.patchDiagnosticSupportAmplitude = supportAmplitude;
end

function [temporalDiversity, timestampBinCount, effectiveFrameCount, frameCounts] = patchTemporalDiversity(timestampBins, params)
% patchTemporalDiversity: Compute patch-level cross-frame temporal
% diversity from distinct frame-bin coverage. When frameCountSaturation is
% configured, the score uses a saturated effective-frame count; otherwise it
% uses the distinct frame-bin count.
%
% Input:
%   timestampBins: [Q x 1] string frame-bin labels for one patch
%   params: class parameter struct with timestampMaxBins and
%       frameCountSaturation
%
% Output:
%   temporalDiversity: scalar score in [0, 1]
%   timestampBinCount: scalar number of distinct frame bins in patch
%   effectiveFrameCount: scalar effective count used for diversity scaling
%   frameCounts: [B x 1] point counts per timestamp bin
    [~, ~, groupIdx] = unique(timestampBins(:));
    timestampBinCount = max(groupIdx);
    frameCounts = accumarray(groupIdx, 1, [timestampBinCount, 1]);

    if isempty(params.frameCountSaturation)
        effectiveFrameCount = timestampBinCount;
    else
        saturatedCounts = 1 - exp(-frameCounts ./ params.frameCountSaturation);
        effectiveFrameCount = sum(saturatedCounts).^2 ./ sum(saturatedCounts.^2);
    end
    temporalDiversity = min(1, log(1 + effectiveFrameCount) ./ log(1 + params.timestampMaxBins));
end

function [normalStability, normalDispersion] = patchNormalStability(points, timestampBins, centroid, n, params)
% patchNormalStability: Estimate cross-frame patch stability by
% projecting patch points onto the local PCA normal direction, reducing each
% frame bin to a median normal coordinate, and converting robust MAD
% dispersion across bins into a bounded support multiplier.
%
% Input:
%   points: [Q x 2] patch point coordinates in global/map BEV frame
%   timestampBins: [Q x 1] discrete frame-bin labels for the patch
%   centroid: [1 x 2] patch centroid used as the normal-coordinate origin
%   n: [2 x 1] patch normal PCA direction
%   params: class parameter struct with normalStabilityLength
%
% Output:
%   normalStability: scalar stability score in [0, 1]
%   normalDispersion: robust cross-bin normal-coordinate dispersion
    [~, ~, groupIdx] = unique(timestampBins(:));
    timestampBinCount = max(groupIdx);
    if timestampBinCount < 2
        normalStability = 1.0;
        normalDispersion = 0.0;
        return;
    end

    normalCoordinates = (points - centroid) * n;
    binNormalCoordinates = accumarray(groupIdx, normalCoordinates, [], @median);
    medianNormalCoordinate = median(binNormalCoordinates);
    normalDispersion = 1.4826 .* median(abs(binNormalCoordinates - medianNormalCoordinate));
    normalStability = exp(-0.5 .* (normalDispersion ./ params.normalStabilityLength).^2);
end
