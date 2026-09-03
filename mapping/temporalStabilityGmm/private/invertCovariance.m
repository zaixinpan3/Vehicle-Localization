function invCovariance = invertCovariance(covariance, pcaEpsilon)
% invertCovariance: Compute a symmetric inverse covariance matrix from a
% valid two-dimensional structured covariance. Invalid, singular, or
% non-positive-definite covariance matrices are errors under fail-fast
% construction.
%
% Input:
%   covariance: [2 x 2] symmetric positive covariance matrix
%   pcaEpsilon: scalar positive regularization scale used only for diagnostics
%
% Output:
%   invCovariance: [2 x 2] symmetric inverse covariance matrix
    assert(isfinite(pcaEpsilon) && pcaEpsilon > 0, ...
        "pcaEpsilon must be positive and finite.");
    validateCovarianceMatrix(covariance, "Covariance inversion");
    invCovariance = covariance \ eye(2);
    if any(~isfinite(invCovariance), "all")
        error("buildTemporalStabilityGmmMap:InvalidInverseCovariance", ...
            "Covariance inversion produced non-finite inverse covariance values.");
    end
    invCovariance = (invCovariance + invCovariance.') ./ 2;
end
