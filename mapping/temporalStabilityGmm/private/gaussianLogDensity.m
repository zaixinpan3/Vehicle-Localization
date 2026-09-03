function logDensity = gaussianLogDensity(points, meanPoint, covariance, invCovariance)
% gaussianLogDensity: Evaluate normalized two-dimensional Gaussian
% log-density values for ordinary GMM EM and post-hoc support assignment.
%
% Input:
%   points: [N x 2] point matrix
%   meanPoint: [1 x 2] Gaussian mean
%   covariance: [2 x 2] covariance matrix
%   invCovariance: [2 x 2] inverse covariance matrix
%
% Output:
%   logDensity: [N x 1] normalized Gaussian log-density values
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
