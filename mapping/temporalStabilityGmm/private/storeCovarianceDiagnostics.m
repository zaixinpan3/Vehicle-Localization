function component = storeCovarianceDiagnostics(component, covarianceDiagnostics, params)
% storeCovarianceDiagnostics: Store covariance safeguard diagnostics and
% query candidate extent on one foreground support component.
%
% Input:
%   component: scalar structured Gaussian support component
%   covarianceDiagnostics: scalar struct from covariance eigenvalue safeguards
%   params: validated class parameter struct with query support extent
%
% Output:
%   component: scalar component with covariance diagnostics
    eigenvaluesAfter = covarianceDiagnostics.eigenvaluesAfter(:);
    component.covarianceEigenvaluesBeforeSafeguard = covarianceDiagnostics.eigenvaluesBefore(:);
    component.covarianceEigenvaluesAfterSafeguard = eigenvaluesAfter;
    component.covarianceEigenvalueLowerBound = params.minCovarianceEigenvalue;
    component.covarianceEigenvalueUpperBound = params.maxCovarianceEigenvalue;
    component.covarianceEigenvalueLimitApplied = covarianceDiagnostics.limitApplied;
    component.covarianceEigenvalueLowerLimitApplied = covarianceDiagnostics.lowerLimitApplied;
    component.covarianceEigenvalueUpperLimitApplied = covarianceDiagnostics.upperLimitApplied;
    component.covarianceAnisotropy = max(eigenvaluesAfter) ./ min(eigenvaluesAfter);
    component.querySupportRadius = params.querySupportMahalanobisRadius .* sqrt(max(eigenvaluesAfter));
    if ~isfinite(component.covarianceAnisotropy) || component.covarianceAnisotropy < 1 || ~isfinite(component.querySupportRadius) || component.querySupportRadius <= 0
        error("buildTemporalStabilityGmmMap:InvalidCovarianceDiagnostics", ...
            "Covariance safeguard diagnostics must produce finite anisotropy and query support radius values.");
    end
end
