function stability = estimateTemporalReliability(points, timestampBins, patchLocalIndices, patchComponents, params)
% estimateTemporalReliability: Estimate per-point temporal
% reliability before EM from leave-one-bin-out cross-frame spatial support,
% concentration-aware temporal diversity, and patch-local geometric
% consistency. The resulting scores define only a sampling distribution and
% are not passed into the Gaussian-mixture EM objective or M-step.
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   timestampBins: [N x 1] class-local frame-bin labels
%   patchLocalIndices: cell array of buildable patch point indices
%   patchComponents: struct array of patch geometry candidates
%   params: validated class parameter struct with reliability settings
%
% Output:
%   stability: scalar struct with point reliability scores, score factors,
%       patch scores, and model diagnostics
    pointCount = size(points, 1);
    if pointCount <= 0 || numel(timestampBins) ~= pointCount
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityInput", ...
            "Temporal reliability estimation requires nonempty [N x 2] points and one timestamp bin per point.");
    end
    if numel(patchLocalIndices) ~= numel(patchComponents)
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityPatches", ...
            "Temporal reliability estimation requires one patch component per patch index set.");
    end

    [sampleSufficiency, temporalDiversity] = leaveOneBinOutSpatialSupport(points, timestampBins, params);
    geometricConsistency = patchGeometricConsistency(points, timestampBins, patchLocalIndices, patchComponents, params);
    scores = clipUnit(sampleSufficiency(:) .* temporalDiversity(:) .* geometricConsistency(:));
    patchScores = zeros(numel(patchLocalIndices), 1);
    for patchIdx = 1:numel(patchLocalIndices)
        patchScores(patchIdx) = mean(scores(patchLocalIndices{patchIdx}));
    end
    [scoreMin, scoreMedian, scoreMax] = finiteSummary(scores);

    stability = struct();
    stability.scores = scores(:);
    stability.sampleSufficiency = sampleSufficiency(:);
    stability.temporalDiversity = temporalDiversity(:);
    stability.geometricConsistency = geometricConsistency(:);
    stability.patchScores = patchScores(:);
    stability.model = "leaveOneBinOutKernelSupportTimesPatchGeometry";
    stability.kernelBandwidth = params.temporalReliabilityKernelBandwidth;
    stability.kernelRadius = params.temporalReliabilityKernelRadiusMultiplier .* params.temporalReliabilityKernelBandwidth;
    stability.scoreMin = scoreMin;
    stability.scoreMedian = scoreMedian;
    stability.scoreMax = scoreMax;
end


function [sampleSufficiency, temporalDiversity] = leaveOneBinOutSpatialSupport(points, timestampBins, params)
% leaveOneBinOutSpatialSupport: Compute point-level cross-frame spatial
% support by summing Gaussian-kernel evidence from neighboring points in other
% timestamp bins, saturating each bin contribution, and separating total sample
% sufficiency from entropy-based temporal diversity.
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   timestampBins: [N x 1] class-local frame-bin labels
%   params: validated class parameter struct with kernel and frame settings
%
% Output:
%   sampleSufficiency: [N x 1] total cross-frame evidence score in [0, 1]
%   temporalDiversity: [N x 1] cross-frame participation evenness in [0, 1]
    pointCount = size(points, 1);
    sampleSufficiency = zeros(pointCount, 1);
    temporalDiversity = zeros(pointCount, 1);
    [uniqueBins, ~, groupIdx] = unique(timestampBins(:));
    binCount = numel(uniqueBins);
    assert(exist("KDTreeSearcher", "class") == 8, ...
        "KDTreeSearcher is required for temporal reliability spatial support.");
    searcher = KDTreeSearcher(points);
    kernelBandwidth = params.temporalReliabilityKernelBandwidth;
    searchRadius = params.temporalReliabilityKernelRadiusMultiplier .* kernelBandwidth;
    [neighborIdx, neighborDistance] = rangesearch(searcher, points, searchRadius);

    for pointIdx = 1:pointCount
        candidateIdx = neighborIdx{pointIdx}(:);
        candidateDistance = neighborDistance{pointIdx}(:);
        crossBinMask = groupIdx(candidateIdx) ~= groupIdx(pointIdx);
        candidateIdx = candidateIdx(crossBinMask);
        candidateDistance = candidateDistance(crossBinMask);
        if isempty(candidateIdx)
            continue;
        end
        kernelWeights = exp(-0.5 .* (candidateDistance ./ kernelBandwidth).^2);
        rawBinEvidence = accumarray(groupIdx(candidateIdx), kernelWeights, [binCount, 1], @sum, 0);
        saturatedBinEvidence = saturateFrameSupport(rawBinEvidence, params);
        totalSupport = sum(saturatedBinEvidence);
        sampleSufficiency(pointIdx) = computeSampleSufficiency(totalSupport, params);
        activeEvidence = saturatedBinEvidence(saturatedBinEvidence > 0);
        if numel(activeEvidence) >= 2
            q = activeEvidence ./ sum(activeEvidence);
            temporalDiversity(pointIdx) = -sum(q .* log(q + eps)) ./ log(numel(activeEvidence) + eps);
        end
    end
    sampleSufficiency = clipUnit(sampleSufficiency);
    temporalDiversity = clipUnit(temporalDiversity);
end


function geometricConsistency = patchGeometricConsistency(points, timestampBins, patchLocalIndices, patchComponents, params)
% patchGeometricConsistency: Compute per-point patch-local geometric
% consistency outside EM. Elongated patches compare each point's normal
% coordinate against the median normal coordinate observed in other timestamp
% bins, while compact patches compare each point to the cross-bin centroid
% median.
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   timestampBins: [N x 1] class-local frame-bin labels
%   patchLocalIndices: cell array of buildable patch point indices
%   patchComponents: struct array of patch geometry candidates
%   params: validated class parameter struct with geometry settings
%
% Output:
%   geometricConsistency: [N x 1] patch geometry reliability in [0, 1]
    geometricConsistency = zeros(size(points, 1), 1);
    for patchIdx = 1:numel(patchLocalIndices)
        localIndices = patchLocalIndices{patchIdx}(:);
        patchPoints = points(localIndices, :);
        patchBins = timestampBins(localIndices);
        [uniqueBins, ~, patchGroupIdx] = unique(patchBins(:));
        if numel(uniqueBins) < 2
            continue;
        end
        component = patchComponents(patchIdx);
        patchScores = zeros(numel(localIndices), 1);
        if component.covarianceAnisotropy >= params.geometricElongatedAnisotropyThreshold
            normalCoordinates = (patchPoints - component.initialCentroid) * component.n;
            binNormalCoordinates = accumarray(patchGroupIdx, normalCoordinates, [], @median);
            for pointIdx = 1:numel(localIndices)
                otherBinMask = (1:numel(uniqueBins)).' ~= patchGroupIdx(pointIdx);
                crossBinNormalCoordinate = median(binNormalCoordinates(otherBinMask));
                residual = abs(normalCoordinates(pointIdx) - crossBinNormalCoordinate);
                patchScores(pointIdx) = exp(-0.5 .* (residual ./ params.normalStabilityLength).^2);
            end
        else
            binCentroidsX = accumarray(patchGroupIdx, patchPoints(:, 1), [], @median);
            binCentroidsY = accumarray(patchGroupIdx, patchPoints(:, 2), [], @median);
            binCentroids = [binCentroidsX, binCentroidsY];
            for pointIdx = 1:numel(localIndices)
                otherBinMask = (1:numel(uniqueBins)).' ~= patchGroupIdx(pointIdx);
                crossBinCentroid = median(binCentroids(otherBinMask, :), 1);
                residual = norm(patchPoints(pointIdx, :) - crossBinCentroid);
                patchScores(pointIdx) = exp(-0.5 .* (residual ./ params.compactStabilityLength).^2);
            end
        end
        geometricConsistency(localIndices) = clipUnit(patchScores);
    end
end
