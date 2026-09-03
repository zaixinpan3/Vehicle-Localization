function [components, refinementDiagnostics, responsibilities] = fitGaussianMixture(points, components, params, pcaEpsilon)
% fitGaussianMixture: Run ordinary full-covariance Gaussian
% mixture EM on a 2-D sample matrix. This function intentionally accepts no
% timestamps, source indices, temporal-consensus weights, Student-t weights, or
% uniform background state; its convergence objective is the standard
% resampled-data Gaussian mixture log-likelihood.
%
% Input:
%   points: [M x 2] ordinary GMM training sample matrix
%   components: struct array of Gaussian initialization components
%   params: validated class parameter struct with EM settings
%   pcaEpsilon: scalar covariance diagonal regularization
%
% Output:
%   components: struct array after standard Gaussian-mixture EM
%   refinementDiagnostics: scalar struct with standard EM status
%   responsibilities: [M x K] final ordinary GMM responsibility matrix
    refinementDiagnostics = emptyRefinementDiagnostics();
    if isempty(points) || size(points, 2) ~= 2 || isempty(components)
        error("buildTemporalStabilityGmmMap:InvalidStandardGmmEmInput", ...
            "Standard Gaussian mixture EM requires nonempty [M x 2] points and nonempty initialized components.");
    end
    components = normalizeGaussianSeedMixtureWeights(components);
    [responsibilities, currentLogLikelihood] = gaussianExpectation(points, components);
    logLikelihoodTrace = nan(params.emMaxIterations + 1, 1);
    logLikelihoodTrace(1) = currentLogLikelihood;
    maxMeanShift = 0;
    iterationCount = 0;
    converged = false;

    for iterationIdx = 1:params.emMaxIterations
        previousLogLikelihood = currentLogLikelihood;
        [components, maxMeanShift] = standardGaussianMaximization(points, responsibilities, components, params, pcaEpsilon);
        [responsibilities, currentLogLikelihood] = gaussianExpectation(points, components);
        iterationCount = iterationIdx;
        logLikelihoodTrace(iterationIdx + 1) = currentLogLikelihood;
        logLikelihoodChange = abs(currentLogLikelihood - previousLogLikelihood);
        if hasLogLikelihoodConverged(logLikelihoodChange, previousLogLikelihood, params.emTolerance) || maxMeanShift <= params.emTolerance
            converged = true;
            break;
        end
    end

    logLikelihoodTrace = logLikelihoodTrace(1:(iterationCount + 1));
    responsibilityRowSums = sum(responsibilities, 2);
    if any(abs(responsibilityRowSums - 1) > 1.0e-10)
        error("buildTemporalStabilityGmmMap:ResponsibilitiesNotNormalized", ...
            "Standard Gaussian EM responsibilities must be normalized for every point.");
    end
    componentWeightSums = sum(responsibilities, 1).';
    refinementDiagnostics.iterationCount = iterationCount;
    refinementDiagnostics.converged = converged;
    refinementDiagnostics.maxMeanShift = maxMeanShift;
    refinementDiagnostics.logLikelihoodTrace = logLikelihoodTrace(:);
    refinementDiagnostics.finalLogLikelihood = currentLogLikelihood;
    refinementDiagnostics.objectiveTrace = logLikelihoodTrace(:);
    refinementDiagnostics.finalObjective = currentLogLikelihood;
    refinementDiagnostics.refinementModel = "standardGaussianMixtureEmOnTemporalReliabilityResample";
    refinementDiagnostics.studentTDegreesOfFreedom = nan;
    refinementDiagnostics.usesUniformBackground = false;
    refinementDiagnostics.uniformBackgroundPriorMode = "disabled";
    refinementDiagnostics.backgroundResponsibilities = zeros(size(points, 1), 1);
    refinementDiagnostics.backgroundResponsibilityMass = 0;
    refinementDiagnostics.backgroundResponsibilityMin = 0;
    refinementDiagnostics.backgroundResponsibilityMean = 0;
    refinementDiagnostics.backgroundResponsibilityMax = 0;
    refinementDiagnostics.foregroundResponsibilityRowSums = responsibilityRowSums;
    refinementDiagnostics.completeResponsibilityRowSums = responsibilityRowSums;
    refinementDiagnostics.foregroundResponsibilityMass = sum(responsibilities(:));
    refinementDiagnostics.mixtureWeights = [components.mixtureWeight].';
    refinementDiagnostics.responsibilityRowSums = responsibilityRowSums;
    refinementDiagnostics.responsibilityRowSumSemantics = "ordinaryGmmForegroundOnly";
    refinementDiagnostics.robustScaleWeightMin = nan;
    refinementDiagnostics.robustScaleWeightMean = nan;
    refinementDiagnostics.robustScaleWeightMax = nan;
    refinementDiagnostics.robustWeightedComponentMass = zeros(numel(components), 1);
    refinementDiagnostics.numericalSafeguardMinEffectiveSupport = params.emMinEffectiveSupport;
    refinementDiagnostics.numericalFreezeMask = false(numel(components), 1);
    refinementDiagnostics.numericalFreezeCount = 0;

    for componentIdx = 1:numel(components)
        components(componentIdx).emIterationCount = iterationCount;
        components(componentIdx).emConverged = converged;
        components(componentIdx).emMaxMeanShift = maxMeanShift;
        components(componentIdx).emResponsibilityPointCount = componentWeightSums(componentIdx);
        components(componentIdx).emForegroundResponsibilityPointCount = componentWeightSums(componentIdx);
        components(componentIdx).emRobustWeightedPointCount = 0;
        components(componentIdx).emRobustScaleWeightMin = nan;
        components(componentIdx).emRobustScaleWeightMean = nan;
        components(componentIdx).emRobustScaleWeightMax = nan;
    end
end

function components = normalizeGaussianSeedMixtureWeights(components)
% normalizeGaussianSeedMixtureWeights: Validate and normalize positive
% seed mixture weights before ordinary Gaussian EM.
%
% Input:
%   components: struct array with mixtureWeight fields
%
% Output:
%   components: struct array with weights summing to one
    mixtureWeights = [components.mixtureWeight].';
    if isempty(mixtureWeights) || any(~isfinite(mixtureWeights)) || any(mixtureWeights <= 0) || sum(mixtureWeights) <= 0
        error("buildTemporalStabilityGmmMap:InvalidGaussianSeedMixtureWeights", ...
            "Standard Gaussian EM seed mixture weights must be finite positive values.");
    end
    mixtureWeights = mixtureWeights ./ sum(mixtureWeights);
    for componentIdx = 1:numel(components)
        components(componentIdx).mixtureWeight = mixtureWeights(componentIdx);
        components(componentIdx).initialMixtureWeight = mixtureWeights(componentIdx);
        components(componentIdx).emMixtureWeightBeforePruning = mixtureWeights(componentIdx);
    end
end

function [components, maxMeanShift] = standardGaussianMaximization(points, responsibilities, components, params, pcaEpsilon)
% standardGaussianMaximization: Perform one ordinary Gaussian-mixture
% M-step using only responsibilities over the resampled 2-D training matrix.
%
% Input:
%   points: [M x 2] ordinary GMM training sample matrix
%   responsibilities: [M x K] row-normalized ordinary GMM responsibilities
%   components: struct array of current Gaussian components
%   params: validated class parameter struct with numerical safeguards
%   pcaEpsilon: scalar covariance diagonal regularization
%
% Output:
%   components: struct array after one standard Gaussian M-step
%   maxMeanShift: scalar maximum component mean displacement
    pointCount = size(points, 1);
    componentCount = numel(components);
    if pointCount <= 0 || ~isequal(size(responsibilities), [pointCount, componentCount]) || any(~isfinite(responsibilities), "all") || any(responsibilities < 0, "all")
        error("buildTemporalStabilityGmmMap:InvalidGaussianMstepResponsibilities", ...
            "Standard Gaussian M-step requires a finite nonnegative [M x K] responsibility matrix.");
    end
    componentWeightSums = sum(responsibilities, 1).';
    if any(~isfinite(componentWeightSums)) || any(componentWeightSums < params.emMinEffectiveSupport)
        error("buildTemporalStabilityGmmMap:InsufficientEffectiveResponsibilityMass", ...
            "Standard Gaussian M-step requires sufficient positive responsibility mass for every fixed-order component.");
    end
    mixtureWeights = componentWeightSums ./ pointCount;
    previousMeans = reshape([components.mean], 2, []).';
    maxMeanShift = 0;

    for componentIdx = 1:componentCount
        componentResponsibilities = responsibilities(:, componentIdx);
        proposedMean = sum(points .* componentResponsibilities, 1) ./ componentWeightSums(componentIdx);
        if any(~isfinite(proposedMean))
            error("buildTemporalStabilityGmmMap:InvalidGaussianMstepMean", ...
                "Standard Gaussian M-step produced a non-finite component mean.");
        end
        [covariance, t, n] = fullCovarianceFromGaussianWeights(points, componentResponsibilities, componentWeightSums(componentIdx), proposedMean, pcaEpsilon);
        components(componentIdx).mixtureWeight = mixtureWeights(componentIdx);
        components(componentIdx).emMixtureWeightBeforePruning = mixtureWeights(componentIdx);
        components(componentIdx).emResponsibilityPointCount = componentWeightSums(componentIdx);
        components(componentIdx).emForegroundResponsibilityPointCount = componentWeightSums(componentIdx);
        components(componentIdx).emNumericalEffectiveSupportThreshold = params.emMinEffectiveSupport;
        components(componentIdx).emNumericalFreezeApplied = false;
        components(componentIdx) = setComponentFullCovarianceGeometry(components(componentIdx), proposedMean, covariance, t, n, params, pcaEpsilon);
        meanShift = norm(components(componentIdx).mean - previousMeans(componentIdx, :));
        maxMeanShift = max(maxMeanShift, meanShift);
    end
end

function [covariance, t, n] = fullCovarianceFromGaussianWeights(points, weights, covarianceNormalizer, meanPoint, pcaEpsilon)
% fullCovarianceFromGaussianWeights: Compute the ordinary Gaussian
% mixture M-step full covariance from component responsibilities.
%
% Input:
%   points: [M x 2] ordinary GMM training sample matrix
%   weights: [M x 1] component responsibilities
%   covarianceNormalizer: scalar component responsibility mass
%   meanPoint: [1 x 2] responsibility-weighted component mean
%   pcaEpsilon: scalar covariance diagonal regularization
%
% Output:
%   covariance: [2 x 2] full covariance matrix
%   t: [2 x 1] dominant covariance direction
%   n: [2 x 1] minor covariance direction
    if any(~isfinite(weights)) || any(weights < 0) || ~isscalar(covarianceNormalizer) || ~isfinite(covarianceNormalizer) || covarianceNormalizer <= 0
        error("buildTemporalStabilityGmmMap:InvalidGaussianCovarianceWeights", ...
            "Ordinary Gaussian covariance update requires finite nonnegative weights and positive normalizer.");
    end
    centeredPoints = points - meanPoint;
    covariance = (centeredPoints.' * (centeredPoints .* weights(:))) ./ covarianceNormalizer + pcaEpsilon .* eye(2);
    [t, n] = covarianceDirections(covariance);
end

function component = setComponentFullCovarianceGeometry(component, meanPoint, covariance, t, n, params, pcaEpsilon)
% setComponentFullCovarianceGeometry: Apply a full-covariance Gaussian
% mixture geometry update to one support component.
%
% Input:
%   component: scalar structured Gaussian component
%   meanPoint: [1 x 2] refined component mean
%   covariance: [2 x 2] robust weighted covariance scale matrix
%   t: [2 x 1] dominant covariance direction
%   n: [2 x 1] minor covariance direction
%   params: validated class parameter struct with covariance safeguards
%   pcaEpsilon: scalar covariance diagonal regularization
%
% Output:
%   component: scalar component with full covariance geometry fields
    component.centroid = meanPoint;
    component.mean = meanPoint;
    component.t = t;
    component.n = n;
    [covariance, covarianceDiagnostics] = applyCovarianceEigenvalueSafeguards(covariance, t, n, params);
    component.covariance = covariance;
    component.invCovariance = invertCovariance(covariance, pcaEpsilon);
    component = storeCovarianceDiagnostics(component, covarianceDiagnostics, params);
end

function converged = hasLogLikelihoodConverged(logLikelihoodChange, previousLogLikelihood, emTolerance)
% hasLogLikelihoodConverged: Test integrated EM convergence from relative
% absolute objective change.
%
% Input:
%   logLikelihoodChange: scalar absolute current-minus-previous log-likelihood
%   previousLogLikelihood: scalar previous log-likelihood value
%   emTolerance: scalar relative convergence tolerance
%
% Output:
%   converged: logical scalar indicating likelihood convergence
    convergenceScale = max(1, abs(previousLogLikelihood));
    converged = logLikelihoodChange <= emTolerance .* convergenceScale;
end

function refinementDiagnostics = emptyRefinementDiagnostics()
% emptyRefinementDiagnostics: Create a scalar layer-level integrated EM
% status struct for fail-fast refinement diagnostics.
%
% Input:
%   none
%
% Output:
%   refinementDiagnostics: scalar struct with iteration count, objective,
%       mixture-weight, responsibility, robust scale-weight, and convergence
%       fields
    refinementDiagnostics = struct();
    refinementDiagnostics.iterationCount = 0;
    refinementDiagnostics.converged = true;
    refinementDiagnostics.maxMeanShift = 0;
    refinementDiagnostics.logLikelihoodTrace = zeros(0, 1);
    refinementDiagnostics.finalLogLikelihood = -inf;
    refinementDiagnostics.objectiveTrace = zeros(0, 1);
    refinementDiagnostics.finalObjective = -inf;
    refinementDiagnostics.refinementModel = "";
    refinementDiagnostics.studentTDegreesOfFreedom = nan;
    refinementDiagnostics.usesUniformBackground = false;
    refinementDiagnostics.uniformBackgroundPriorMode = "";
    refinementDiagnostics.fixedUniformBackgroundPrior = 0;
    refinementDiagnostics.uniformBackgroundPrior = 0;
    refinementDiagnostics.initialUniformBackgroundPrior = 0;
    refinementDiagnostics.uniformBackgroundDensity = 0;
    refinementDiagnostics.uniformBackgroundBounds = zeros(1, 4);
    refinementDiagnostics.backgroundResponsibilities = zeros(0, 1);
    refinementDiagnostics.backgroundResponsibilityMass = 0;
    refinementDiagnostics.backgroundResponsibilityMin = 0;
    refinementDiagnostics.backgroundResponsibilityMean = 0;
    refinementDiagnostics.backgroundResponsibilityMax = 0;
    refinementDiagnostics.foregroundResponsibilityRowSums = zeros(0, 1);
    refinementDiagnostics.completeResponsibilityRowSums = zeros(0, 1);
    refinementDiagnostics.foregroundResponsibilityMass = 0;
    refinementDiagnostics.mixtureWeights = zeros(0, 1);
    refinementDiagnostics.responsibilityRowSums = zeros(0, 1);
    refinementDiagnostics.responsibilityRowSumSemantics = "";
    refinementDiagnostics.robustScaleWeightMin = nan;
    refinementDiagnostics.robustScaleWeightMean = nan;
    refinementDiagnostics.robustScaleWeightMax = nan;
    refinementDiagnostics.robustWeightedComponentMass = zeros(0, 1);
    refinementDiagnostics.numericalSafeguardMinEffectiveSupport = 0;
    refinementDiagnostics.numericalFreezeMask = false(0, 1);
    refinementDiagnostics.numericalFreezeCount = 0;
end
