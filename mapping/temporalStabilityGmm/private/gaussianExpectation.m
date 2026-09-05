function [responsibilities, logLikelihood] = gaussianExpectation(points, components)
% gaussianExpectation: Compute ordinary Gaussian-mixture responsibilities
% and log-likelihood for a 2-D point matrix and current full-covariance
% components.
%
% Input:
%   points: [N x 2] point matrix evaluated by the mixture
%   components: struct array of Gaussian components with normalized weights
%
% Output:
%   responsibilities: [N x K] ordinary GMM responsibility matrix
%   logLikelihood: scalar Gaussian mixture log-likelihood
    pointCount = size(points, 1);
    componentCount = numel(components);
    logWeightedLikelihoods = -inf(pointCount, componentCount);
    mixtureWeights = [components.mixtureWeight].';
    if isempty(mixtureWeights) || any(~isfinite(mixtureWeights)) || any(mixtureWeights <= 0) || abs(sum(mixtureWeights) - 1) > 1.0e-10
        error("buildTemporalStabilityGmmMap:InvalidGaussianMixtureWeights", ...
            "Ordinary GMM mixture weights must be finite positive values that sum to one.");
    end
    for componentIdx = 1:componentCount
        logWeightedLikelihoods(:, componentIdx) = log(mixtureWeights(componentIdx)) + ...
            gaussianLogDensity(points, components(componentIdx).mean, components(componentIdx).covariance, components(componentIdx).invCovariance);
    end
    logNormalizers = rowLogSumExp(logWeightedLikelihoods);
    if any(~isfinite(logNormalizers))
        error("buildTemporalStabilityGmmMap:InvalidGaussianLikelihoodNormalizer", ...
            "Every ordinary GMM point must have a finite positive likelihood normalizer.");
    end
    responsibilities = exp(logWeightedLikelihoods - logNormalizers);
    if any(~isfinite(responsibilities), "all") || any(responsibilities < 0, "all")
        error("buildTemporalStabilityGmmMap:InvalidResponsibilities", ...
            "Ordinary GMM responsibility computation produced non-finite or negative responsibilities.");
    end
    responsibilityRowSums = sum(responsibilities, 2);
    if any(abs(responsibilityRowSums - 1) > 1.0e-10)
        error("buildTemporalStabilityGmmMap:ResponsibilitiesNotNormalized", ...
            "Ordinary GMM responsibilities must be row-normalized for every point.");
    end
    logLikelihood = sum(logNormalizers);
    if ~isfinite(logLikelihood)
        error("buildTemporalStabilityGmmMap:InvalidLogLikelihood", ...
            "Ordinary GMM log-likelihood must be finite.");
    end
end

function logDensity = gaussianLogDensity(points, meanPoint, covariance, invCovariance)
% gaussianLogDensity: Evaluate normalized two-dimensional Gaussian
% log-density values for ordinary GMM EM and post-hoc support assignment.
    delta = points - meanPoint;
    validateCovarianceMatrix(covariance, "Ordinary Gaussian log-density");
    if ~isnumeric(invCovariance) || ~isequal(size(invCovariance), [2 2]) || any(~isfinite(invCovariance), "all")
        error("buildTemporalStabilityGmmMap:InvalidInverseCovariance", ...
            "Ordinary Gaussian log-density requires a finite [2 x 2] inverse covariance matrix.");
    end
    distanceSquared = sum((delta * invCovariance) .* delta, 2);
    if any(~isfinite(distanceSquared))
        error("buildTemporalStabilityGmmMap:InvalidMahalanobisDistance", ...
            "Ordinary Gaussian log-density produced non-finite Mahalanobis distances.");
    end
    negativeMask = distanceSquared < 0;
    if any(distanceSquared(negativeMask) < -1.0e-10)
        error("buildTemporalStabilityGmmMap:NegativeMahalanobisDistance", ...
            "Ordinary Gaussian log-density produced a meaningfully negative Mahalanobis distance.");
    end
    distanceSquared(negativeMask) = 0;
    logDensity = -0.5 .* (2 .* log(2 .* pi) + log(det(covariance)) + distanceSquared);
    if any(~isfinite(logDensity))
        error("buildTemporalStabilityGmmMap:InvalidGaussianLogDensity", ...
            "Ordinary Gaussian log-density must be finite for every point.");
    end
end

function logSum = rowLogSumExp(logValues)
% rowLogSumExp: Compute row-wise log-sum-exp values for numerically
% stable responsibility normalization and likelihood accumulation.
    rowMax = max(logValues, [], 2);
    logSum = -inf(size(rowMax));
    finiteMask = isfinite(rowMax);
    if any(finiteMask)
        stableValues = exp(logValues(finiteMask, :) - rowMax(finiteMask));
        logSum(finiteMask) = rowMax(finiteMask) + log(sum(stableValues, 2));
    end
end
