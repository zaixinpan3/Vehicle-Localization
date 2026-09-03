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
