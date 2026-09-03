function validateCovarianceMatrix(covariance, contextName)
% validateCovarianceMatrix: Validate that a covariance matrix is finite,
% symmetric, and positive definite before it is used in EM or query
% diagnostics.
%
% Input:
%   covariance: [2 x 2] covariance matrix
%   contextName: character vector identifying the validation context
%
% Output:
%   none
    if ~isnumeric(covariance) || ~isequal(size(covariance), [2 2]) || any(~isfinite(covariance), "all")
        error("buildTemporalStabilityGmmMap:InvalidCovarianceMatrix", ...
            "%s requires a finite [2 x 2] covariance matrix.", contextName);
    end
    if norm(covariance - covariance.', "fro") > 1.0e-10 .* max(1, norm(covariance, "fro"))
        error("buildTemporalStabilityGmmMap:NonSymmetricCovariance", ...
            "%s requires a symmetric covariance matrix.", contextName);
    end
    [~, cholFlag] = chol((covariance + covariance.') ./ 2);
    covarianceDeterminant = det(covariance);
    if cholFlag ~= 0 || ~isfinite(covarianceDeterminant) || covarianceDeterminant <= 0
        error("buildTemporalStabilityGmmMap:NonPositiveDefiniteCovariance", ...
            "%s requires a symmetric positive definite covariance matrix with a finite positive determinant.", contextName);
    end
end
