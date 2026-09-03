function [t, n] = covarianceDirections(covarianceMatrix)
% covarianceDirections: Extract deterministic orthonormal directions
% from a two-dimensional covariance matrix. Nearly isotropic covariances use
% the canonical BEV basis because compact Gaussian components do not require a
% unique dominant direction.
%
% Input:
%   covarianceMatrix: [2 x 2] symmetric covariance-like matrix
%
% Output:
%   t: [2 x 1] dominant covariance direction
%   n: [2 x 1] minor covariance direction
    validateCovarianceMatrix(covarianceMatrix, "Covariance direction extraction");
    [vectors, values] = eig(covarianceMatrix);
    eigenvalues = diag(values);
    if any(~isfinite(vectors), "all") || any(~isfinite(eigenvalues))
        error("buildTemporalStabilityGmmMap:InvalidCovarianceEigenDecomposition", ...
            "Covariance eigendecomposition produced non-finite eigenvectors or eigenvalues.");
    end
    [sortedEigenvalues, orderIdx] = sort(eigenvalues, "descend");
    if sortedEigenvalues(1) <= sortedEigenvalues(2) + 1.0e-12 .* max(1, abs(sortedEigenvalues(1)))
        t = [1; 0];
        n = [0; 1];
        return;
    end
    t = vectors(:, orderIdx(1));
    n = vectors(:, orderIdx(2));
end
