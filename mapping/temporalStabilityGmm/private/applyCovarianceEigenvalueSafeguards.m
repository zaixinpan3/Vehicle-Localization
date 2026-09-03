function [covariance, covarianceDiagnostics] = applyCovarianceEigenvalueSafeguards(covariance, t, n, params)
% applyCovarianceEigenvalueSafeguards: Bound covariance eigenvalues in the
% component's current orthonormal frame so peak-normalized query support cannot
% become an unbounded broad attraction basin. The operation preserves the
% supplied EM orientation, symmetry, and positive definiteness.
%
% Input:
%   covariance: [2 x 2] symmetric positive covariance matrix
%   t: [2 x 1] dominant covariance direction
%   n: [2 x 1] minor covariance direction
%   params: validated class parameter struct with covariance eigenvalue bounds
%
% Output:
%   covariance: [2 x 2] bounded symmetric positive covariance matrix
%   covarianceDiagnostics: scalar struct describing raw and bounded eigenvalues
    validateCovarianceMatrix(covariance, "Covariance eigenvalue safeguarding");
    if ~isnumeric(t) || ~isequal(size(t), [2 1]) || any(~isfinite(t)) || ~isnumeric(n) || ~isequal(size(n), [2 1]) || any(~isfinite(n))
        error("buildTemporalStabilityGmmMap:InvalidCovarianceDirections", ...
            "Covariance eigenvalue safeguarding requires finite [2 x 1] covariance directions.");
    end
    if abs(norm(t) - 1) > 1.0e-10 || abs(norm(n) - 1) > 1.0e-10 || abs(t.' * n) > 1.0e-10
        error("buildTemporalStabilityGmmMap:InvalidCovarianceDirections", ...
            "Covariance eigenvalue safeguarding requires orthonormal covariance directions.");
    end
    eigenvaluesBefore = [t.' * covariance * t; n.' * covariance * n];
    if any(~isfinite(eigenvaluesBefore)) || any(eigenvaluesBefore <= 0)
        error("buildTemporalStabilityGmmMap:InvalidCovarianceEigenvalues", ...
            "Covariance eigenvalue safeguarding requires finite positive covariance eigenvalues.");
    end
    eigenvaluesAfter = min(max(eigenvaluesBefore, params.minCovarianceEigenvalue), params.maxCovarianceEigenvalue);
    covariance = eigenvaluesAfter(1) .* (t * t.') + eigenvaluesAfter(2) .* (n * n.');
    covariance = (covariance + covariance.') ./ 2;
    validateCovarianceMatrix(covariance, "Bounded covariance");
    covarianceDiagnostics = struct();
    covarianceDiagnostics.eigenvaluesBefore = eigenvaluesBefore;
    covarianceDiagnostics.eigenvaluesAfter = eigenvaluesAfter;
    covarianceDiagnostics.lowerLimitApplied = any(eigenvaluesAfter > eigenvaluesBefore);
    covarianceDiagnostics.upperLimitApplied = any(eigenvaluesAfter < eigenvaluesBefore);
    covarianceDiagnostics.limitApplied = covarianceDiagnostics.lowerLimitApplied || covarianceDiagnostics.upperLimitApplied;
end
