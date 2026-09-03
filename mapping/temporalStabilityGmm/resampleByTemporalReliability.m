function [sampledPoints, sampledSourceIndices, resampleDiagnostics] = resampleByTemporalReliability(points, sourceIndices, stability, params)
% resampleByTemporalReliability: Convert temporal reliability scores into
% normalized class-local sampling probabilities and draw an ordinary GMM
% training set with independent Bernoulli sampling. A deterministic minimum
% fill preserves a valid fixed-order GMM training set when a random draw is too
% sparse.
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   sourceIndices: [N x 1] original input indices for class-local points
%   stability: scalar struct with point reliability scores
%   params: validated class parameter struct with sampling settings
%
% Output:
%   sampledPoints: [M x 2] ordinary unweighted GMM training points
%   sampledSourceIndices: [M x 1] source indices for sampled rows
%   resampleDiagnostics: scalar struct with sampling probabilities and counts
    pointCount = size(points, 1);
    if numel(sourceIndices) ~= pointCount || numel(stability.scores) ~= pointCount
        error("buildTemporalStabilityGmmMap:InvalidResampleInput", ...
            "Temporal reliability resampling requires one source index and one reliability score per point.");
    end
    samplingWeights = params.temporalReliabilitySamplingFloor + stability.scores(:) .^ params.temporalReliabilityPower;
    if any(~isfinite(samplingWeights)) || any(samplingWeights < 0) || sum(samplingWeights) <= 0
        error("buildTemporalStabilityGmmMap:InvalidSamplingWeights", ...
            "Temporal reliability sampling weights must be finite nonnegative values with positive total mass.");
    end
    samplingProbabilities = samplingWeights ./ sum(samplingWeights);
    expectedSampleCount = pointCount .* params.temporalResampleExpectedCountMultiplier;
    bernoulliProbabilities = min(1, expectedSampleCount .* samplingProbabilities);
    stream = RandStream("mt19937ar", "Seed", params.temporalResampleRandomSeed);
    sampleMask = rand(stream, pointCount, 1) <= bernoulliProbabilities;
    minimumSampleCount = min(pointCount, max(2, min(numel(stability.patchScores), pointCount)));
    deterministicFillCount = 0;
    if nnz(sampleMask) < minimumSampleCount
        missingCount = minimumSampleCount - nnz(sampleMask);
        [~, fillOrder] = sort(bernoulliProbabilities, "descend");
        fillOrder = fillOrder(~sampleMask(fillOrder));
        fillIdx = fillOrder(1:min(missingCount, numel(fillOrder)));
        sampleMask(fillIdx) = true;
        deterministicFillCount = numel(fillIdx);
    end
    if ~any(sampleMask)
        error("buildTemporalStabilityGmmMap:EmptyResample", ...
            "Temporal reliability resampling produced an empty GMM training set.");
    end
    sampledPoints = points(sampleMask, :);
    sampledSourceIndices = sourceIndices(sampleMask);
    resampleDiagnostics = struct();
    resampleDiagnostics.model = "independentBernoulliTemporalReliabilitySampling";
    resampleDiagnostics.samplingProbabilities = samplingProbabilities(:);
    resampleDiagnostics.bernoulliProbabilities = bernoulliProbabilities(:);
    resampleDiagnostics.resampleCounts = double(sampleMask(:));
    resampleDiagnostics.expectedSampleCount = expectedSampleCount;
    resampleDiagnostics.selectedSampleCount = size(sampledPoints, 1);
    resampleDiagnostics.deterministicFillCount = deterministicFillCount;
end
