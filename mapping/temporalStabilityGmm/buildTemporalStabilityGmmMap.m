function gmmMap = buildTemporalStabilityGmmMap(points, labels, timestamps, cfg)
% buildTemporalStabilityGmmMap: Build a semantic
% temporal-stability Gaussian support map from BEV positive feature anchors.
% Patches initialize class-specific spatial support candidates and provide
% local shape geometry for point reliability scoring. Per-point temporal
% reliability is estimated before EM from leave-one-bin-out repeated spatial
% support and patch-local geometric consistency. The reliability scores define
% an independent Bernoulli sampling distribution for a standard [M x 2] GMM
% training set. The final mixture training stage is ordinary full-covariance
% Gaussian-mixture EM: it receives only 2-D points, uses no timestamps, no
% temporal-consensus weights, no Student-t latent weights, and no uniform
% background component. After EM converges, temporal support is recomputed on
% the original class points from ordinary GMM responsibilities multiplied by
% the precomputed reliability scores; those post-hoc diagnostics drive support
% amplitude, pruning, optional refit, and query-time stability weighting.
% The implementation follows a fail-fast policy: cfg.classes must be explicit,
% all required class parameters must be present before the builder is called,
% and empty classes, degenerate components, invalid covariance matrices,
% invalid mixture weights, malformed map state, invalid numerical likelihoods,
% and uncomputable cross-frame stability are errors rather than recoverable
% conditions. Temporal information may affect sampling, initialization,
% component pruning, and support amplitude, but not EM responsibility
% normalization, M-step parameter updates, or convergence objective.
%
% Input:
%   points: [N x 2] numeric BEV feature point coordinates
%   labels: [N x 1] semantic class labels convertible to string
%   timestamps: [N x 1] frame ids, keyframe ids, traversal ids, or categorical
%       observation ids. Standard low-frequency point-cloud data should provide
%       one frame id per point, and each unique id is treated as exactly one
%       observation bin for temporal diversity and normal-position stability.
%   cfg: struct from temporalStabilityMapConfig or compatible struct
%       with explicit nonempty classes and complete class parameters
%
% Output:
%   gmmMap: struct with mapType, classLabels, pcaEpsilon, sourcePointCount,
%       and semantic layers containing standard-GMM spatial components,
%       temporal reliability diagnostics, mixture weights, support amplitudes,
%       and post-hoc temporal support diagnostics
    assert(nargin >= 4 && ~isempty(cfg), ...
        "cfg must be provided explicitly; use temporalStabilityMapConfig outside the builder to create defaults.");
    assert(isnumeric(points) && ismatrix(points) && size(points, 2) == 2, ...
        "points must be an [N x 2] numeric array.");
    assert(numel(labels) == size(points, 1), ...
        "labels must contain one class label per point.");
    assert(numel(timestamps) == size(points, 1), ...
        "timestamps must contain one frame or observation id per point.");
    assert(isstruct(cfg) && isfield(cfg, "defaultParams"), ...
        "cfg must be a struct from temporalStabilityMapConfig or a compatible struct.");

    points = double(points);
    assert(all(isfinite(points), "all"), ...
        "points must contain only finite numeric values.");
    assert(isfield(cfg, "pcaEpsilon") && isscalar(cfg.pcaEpsilon) && isnumeric(cfg.pcaEpsilon) && isfinite(cfg.pcaEpsilon) && cfg.pcaEpsilon > 0, ...
        "cfg.pcaEpsilon must be a positive finite numeric scalar.");
    labelKeys = string(labels(:));
    timestampBins = binTimestamps(timestamps);
    classLabels = resolveClassLabels(cfg);
    logEnabled = isLogEnabled(cfg);
    logStep(logEnabled, "start", "points=%d | classes=%s", size(points, 1), char(strjoin(classLabels(:).', ", ")));
    layers = repmat(emptyLayer(), numel(classLabels), 1);

    for classIdx = 1:numel(classLabels)
        classLabel = classLabels(classIdx);
        params = resolveClassParams(cfg, classLabel);
        classMask = labelKeys == classLabel;
        classPoints = points(classMask, :);
        classTimestampBins = timestampBins(classMask);
        sourceIndices = find(classMask);
        logStep(logEnabled, "class.start", "class=%s | points=%d | timestampBins=%d | k=%d | radius=%.4g | maxDiameter=%.4g | minComponentPoints=%d", ...
            char(classLabel), size(classPoints, 1), numel(unique(classTimestampBins)), round(double(params.k)), double(params.radius), double(params.maxDiameter), round(double(params.minComponentPoints)));
        if isempty(classPoints)
            error("buildTemporalStabilityGmmMap:EmptySemanticClass", ...
                "Semantic class %s has no input points; empty semantic layers are invalid under fail-fast construction.", char(classLabel));
        end
        [patchLocalIndices, patchComponents, removedPatchCount] = buildSpatialSupportPatches( ...
            classPoints, sourceIndices, classTimestampBins, params, cfg.pcaEpsilon);
        logPatchFilterSummary(logEnabled, classLabel, patchLocalIndices, removedPatchCount);
        if isempty(patchLocalIndices)
            error("buildTemporalStabilityGmmMap:NoPatches", ...
                "Semantic class %s produced no initialization patches; EM mixture model order is invalid.", char(classLabel));
        end

        stability = estimateTemporalReliability(classPoints, classTimestampBins, patchLocalIndices, patchComponents, params);
        logStep(logEnabled, "class.reliability", "class=%s | scoreMin=%.4g | scoreMedian=%.4g | scoreMax=%.4g", ...
            char(classLabel), stability.scoreMin, stability.scoreMedian, stability.scoreMax);
        [sampledPoints, sampledSourceIndices, resampleDiagnostics] = resampleByTemporalReliability(classPoints, sourceIndices, stability, params);
        logStep(logEnabled, "class.resample", "class=%s | expected=%d | selected=%d | fill=%d", ...
            char(classLabel), round(resampleDiagnostics.expectedSampleCount), size(sampledPoints, 1), resampleDiagnostics.deterministicFillCount);
        seeds = initializeGaussianMixtureSeeds(classPoints, patchLocalIndices, patchComponents, stability, resampleDiagnostics.resampleCounts, params);
        [components, refinementDiagnostics, finalResponsibilities] = fitGaussianMixture(sampledPoints, seeds, params, cfg.pcaEpsilon);
        logStep(logEnabled, "class.standardEm", "class=%s | components=%d | emIter=%d | converged=%d | finalLogLikelihood=%.6g", ...
            char(classLabel), numel(components), refinementDiagnostics.iterationCount, refinementDiagnostics.converged, refinementDiagnostics.finalLogLikelihood);
        supportDiagnostics = computePosthocTemporalSupport(classPoints, sourceIndices, classTimestampBins, stability, components, params);
        supportDiagnostics.emConvergedAtSupportEstimation = refinementDiagnostics.converged;
        components = applyPosthocSupportFields(classPoints, sourceIndices, classTimestampBins, supportDiagnostics, components, params, refinementDiagnostics.converged);
        logAmplitudeSummary(logEnabled, classLabel, components, supportDiagnostics);
        [components, pruningDiagnostics] = pruneComponentsByPosthocSupport(components, params);
        logStep(logEnabled, "class.prune", "class=%s | before=%d | after=%d | removed=%d", ...
            char(classLabel), pruningDiagnostics.inputComponentCount, pruningDiagnostics.outputComponentCount, pruningDiagnostics.removedCount);
        if params.refitAfterTemporalPruning
            seeds = reseedFromRetainedComponents(components);
            [components, refinementDiagnostics, finalResponsibilities] = fitGaussianMixture(sampledPoints, seeds, params, cfg.pcaEpsilon);
            supportDiagnostics = computePosthocTemporalSupport(classPoints, sourceIndices, classTimestampBins, stability, components, params);
            supportDiagnostics.emConvergedAtSupportEstimation = refinementDiagnostics.converged;
            components = applyPosthocSupportFields(classPoints, sourceIndices, classTimestampBins, supportDiagnostics, components, params, refinementDiagnostics.converged);
            logStep(logEnabled, "class.refit", "class=%s | components=%d | emIter=%d | converged=%d | finalLogLikelihood=%.6g", ...
                char(classLabel), numel(components), refinementDiagnostics.iterationCount, refinementDiagnostics.converged, refinementDiagnostics.finalLogLikelihood);
        end
        [componentMeans, componentCovariances, componentBoundingBoxes, componentPointCounts, componentSupportAmplitudes, componentMixtureWeights, componentPatchBoundingBoxes, componentPatchPointCounts] = buildComponentIndexData(components);
        layers(classIdx).classLabel = classLabel;
        layers(classIdx).params = params;
        layers(classIdx).components = components;
        layers(classIdx).priorScore = params.priorSupport;
        layers(classIdx).pointCount = size(classPoints, 1);
        layers(classIdx).sourceIndices = sourceIndices(:);
        layers(classIdx).temporalReliabilityScores = stability.scores(:);
        layers(classIdx).temporalReliabilityModel = stability.model;
        layers(classIdx).samplingProbabilities = resampleDiagnostics.samplingProbabilities(:);
        layers(classIdx).resampleCounts = resampleDiagnostics.resampleCounts(:);
        layers(classIdx).resampledSourceIndices = sampledSourceIndices(:);
        layers(classIdx).resampleModel = resampleDiagnostics.model;
        layers(classIdx).gmmTrainingModel = refinementDiagnostics.refinementModel;
        layers(classIdx).gmmTrainingUsesTimestamps = false;
        layers(classIdx).gmmTrainingUsesTemporalWeights = false;
        layers(classIdx).gmmTrainingUsesStudentTWeights = false;
        layers(classIdx).gmmTrainingUsesUniformBackground = false;
        layers(classIdx).posthocTemporalSupportEstimated = supportDiagnostics.completed;
        layers(classIdx).posthocTemporalSupportResponsibilities = storedResponsibilities(supportDiagnostics.responsibilities, params);
        layers(classIdx).componentMeans = componentMeans;
        layers(classIdx).componentCovariances = componentCovariances;
        layers(classIdx).componentBoundingBoxes = componentBoundingBoxes;
        layers(classIdx).componentPointCounts = componentPointCounts;
        layers(classIdx).componentPatchBoundingBoxes = componentPatchBoundingBoxes;
        layers(classIdx).componentPatchPointCounts = componentPatchPointCounts;
        layers(classIdx).componentSupportAmplitudes = componentSupportAmplitudes;
        layers(classIdx).componentMixtureWeights = componentMixtureWeights;
        layers(classIdx).centroidSearcher = buildCentroidSearcher(componentMeans);
        layers(classIdx).queryCandidateComponentCount = params.queryCandidateComponentCount;
        layers(classIdx).queryCandidateRadius = params.queryCandidateRadius;
        layers(classIdx).queryCandidateRadiusInflation = queryCandidateRadiusInflation(components);
        layers(classIdx).queryCandidateRetrievalModel = "centroidRadiusInflatedByCovarianceExtent";
        layers(classIdx).emIterationCount = refinementDiagnostics.iterationCount;
        layers(classIdx).emConverged = refinementDiagnostics.converged;
        layers(classIdx).emMaxMeanShift = refinementDiagnostics.maxMeanShift;
        layers(classIdx).emLogLikelihoodTrace = refinementDiagnostics.logLikelihoodTrace;
        layers(classIdx).emFinalLogLikelihood = refinementDiagnostics.finalLogLikelihood;
        layers(classIdx).emObjectiveTrace = refinementDiagnostics.objectiveTrace;
        layers(classIdx).emFinalObjective = refinementDiagnostics.finalObjective;
        layers(classIdx).emRefinementModel = refinementDiagnostics.refinementModel;
        layers(classIdx).emStudentTDegreesOfFreedom = refinementDiagnostics.studentTDegreesOfFreedom;
        layers(classIdx).emUsesUniformBackground = refinementDiagnostics.usesUniformBackground;
        layers(classIdx).emUniformBackgroundPriorMode = refinementDiagnostics.uniformBackgroundPriorMode;
        layers(classIdx).emFixedUniformBackgroundPrior = refinementDiagnostics.fixedUniformBackgroundPrior;
        layers(classIdx).emUniformBackgroundPrior = refinementDiagnostics.uniformBackgroundPrior;
        layers(classIdx).emInitialUniformBackgroundPrior = refinementDiagnostics.initialUniformBackgroundPrior;
        layers(classIdx).emUniformBackgroundDensity = refinementDiagnostics.uniformBackgroundDensity;
        layers(classIdx).emUniformBackgroundBounds = refinementDiagnostics.uniformBackgroundBounds;
        layers(classIdx).emBackgroundResponsibilityMass = refinementDiagnostics.backgroundResponsibilityMass;
        layers(classIdx).emBackgroundResponsibilityMin = refinementDiagnostics.backgroundResponsibilityMin;
        layers(classIdx).emBackgroundResponsibilityMean = refinementDiagnostics.backgroundResponsibilityMean;
        layers(classIdx).emBackgroundResponsibilityMax = refinementDiagnostics.backgroundResponsibilityMax;
        layers(classIdx).emBackgroundResponsibilities = storedVector(refinementDiagnostics.backgroundResponsibilities, params);
        layers(classIdx).emForegroundResponsibilityRowSums = refinementDiagnostics.foregroundResponsibilityRowSums;
        layers(classIdx).emCompleteResponsibilityRowSums = refinementDiagnostics.completeResponsibilityRowSums;
        layers(classIdx).emForegroundResponsibilityMass = refinementDiagnostics.foregroundResponsibilityMass;
        layers(classIdx).emMixtureWeightsBeforePruning = refinementDiagnostics.mixtureWeights;
        layers(classIdx).emResponsibilityRowSums = refinementDiagnostics.completeResponsibilityRowSums;
        layers(classIdx).emResponsibilityRowSumSemantics = refinementDiagnostics.responsibilityRowSumSemantics;
        layers(classIdx).emResponsibilities = storedResponsibilities(finalResponsibilities, params);
        layers(classIdx).emRobustScaleWeights = zeros(0, 0);
        layers(classIdx).emRobustScaleWeightMin = refinementDiagnostics.robustScaleWeightMin;
        layers(classIdx).emRobustScaleWeightMean = refinementDiagnostics.robustScaleWeightMean;
        layers(classIdx).emRobustScaleWeightMax = refinementDiagnostics.robustScaleWeightMax;
        layers(classIdx).emRobustWeightedComponentMass = refinementDiagnostics.robustWeightedComponentMass;
        layers(classIdx).emNumericalSafeguardMinEffectiveSupport = refinementDiagnostics.numericalSafeguardMinEffectiveSupport;
        layers(classIdx).emNumericalFreezeMask = refinementDiagnostics.numericalFreezeMask;
        layers(classIdx).emNumericalFreezeCount = refinementDiagnostics.numericalFreezeCount;
        layers(classIdx).integratedSupportEstimated = supportDiagnostics.completed;
        layers(classIdx).integratedSupportComponentCount = supportDiagnostics.componentCount;
        layers(classIdx).integratedSupportEstimatedAfterEmConvergence = supportDiagnostics.emConvergedAtSupportEstimation;
        layers(classIdx).integratedSupportAssignmentModel = supportDiagnostics.assignmentModel;
        layers(classIdx).integratedSupportUsesStudentTInlierWeights = supportDiagnostics.usesStudentTInlierWeights;
        layers(classIdx).integratedSupportAssignmentWeightMin = supportDiagnostics.robustAssignmentWeightMin;
        layers(classIdx).integratedSupportAssignmentWeightMean = supportDiagnostics.robustAssignmentWeightMean;
        layers(classIdx).integratedSupportAssignmentWeightMax = supportDiagnostics.robustAssignmentWeightMax;
        layers(classIdx).integratedSupportWeightedComponentMass = supportDiagnostics.robustWeightedComponentMass;
        layers(classIdx).integratedSupportAssignmentRowSums = supportDiagnostics.robustAssignmentRowSums;
        layers(classIdx).integratedSupportFrameSupportModel = supportDiagnostics.frameSupportModel;
        layers(classIdx).integratedSupportTemporalDiversityModel = supportDiagnostics.temporalDiversityModel;
        layers(classIdx).integratedSupportSampleSufficiencyModel = supportDiagnostics.sampleSufficiencyModel;
        layers(classIdx).integratedSupportGeometricConsistencyModel = supportDiagnostics.geometricConsistencyModel;
        layers(classIdx).pruningAppliedAfterIntegratedSupport = pruningDiagnostics.appliedAfterIntegratedSupport;
        layers(classIdx).componentCountBeforePruning = pruningDiagnostics.inputComponentCount;
        layers(classIdx).componentCountAfterPruning = pruningDiagnostics.outputComponentCount;
        layers(classIdx).integratedSupportPruningKeepMask = pruningDiagnostics.keepMask;
        layers(classIdx).integratedSupportPrunedComponentCount = pruningDiagnostics.removedCount;
        layers(classIdx).integratedSupportPruningInputSupportAmplitudes = pruningDiagnostics.inputSupportAmplitudes;
        layers(classIdx).integratedSupportPruningInputTimestampBinCounts = pruningDiagnostics.inputTimestampBinCounts;
        layers(classIdx).integratedSupportPruningInputEffectiveSupportPointCounts = pruningDiagnostics.inputEffectiveSupportPointCounts;
        layers(classIdx).integratedSupportPruningInputGeometricStability = pruningDiagnostics.inputGeometricStability;
    end

    gmmMap = struct();
    gmmMap.mapType = "semanticTemporalStabilityStructuredGMM";
    gmmMap.classLabels = classLabels(:);
    gmmMap.pcaEpsilon = cfg.pcaEpsilon;
    gmmMap.sourcePointCount = size(points, 1);
    gmmMap.layers = layers;
end

function timestampBins = binTimestamps(timestamps)
% binTimestamps: Convert per-point frame ids, keyframe ids, traversal
% ids, or categorical observation ids into discrete observation-bin labels.
% Numeric and nonnumeric values are treated as already-discrete frame-bin
% identifiers, so different frame ids are never merged by this builder.
%
% Input:
%   timestamps: [N x 1] raw frame id, keyframe id, traversal id, or
%       observation id values
%
% Output:
%   timestampBins: [N x 1] string frame-bin labels used for cross-frame
%       temporal diversity and normal-position stability grouping
    if isnumeric(timestamps) || islogical(timestamps)
        timestampValues = double(timestamps(:));
        assert(all(isfinite(timestampValues)), ...
            "numeric frame or observation ids must be finite.");
        timestampBins = string(timestampValues);
    else
        timestampBins = string(timestamps(:));
    end
end

function classLabels = resolveClassLabels(cfg)
% resolveClassLabels: Validate and return the explicit semantic class
% labels configured for fail-fast map construction.
%
% Input:
%   cfg: map configuration struct with a nonempty classes field
%
% Output:
%   classLabels: [C x 1] configured string labels for semantic map layers
    if ~isfield(cfg, "classes") || isempty(cfg.classes)
        error("buildTemporalStabilityGmmMap:MissingConfiguredClasses", ...
            "cfg.classes must be explicitly provided and nonempty; the builder does not infer semantic classes from observed labels.");
    end
    classLabels = string(cfg.classes(:));
    if any(ismissing(classLabels)) || any(strlength(classLabels) == 0)
        error("buildTemporalStabilityGmmMap:InvalidConfiguredClasses", ...
            "cfg.classes must contain nonmissing nonempty semantic class labels.");
    end
end

function params = resolveClassParams(cfg, classLabel)
% resolveClassParams: Merge default semantic map parameters with the
% class-specific override matching the class label and validate the complete
% hyperparameter set required by patch initialization, reliability resampling,
% ordinary Gaussian mixture EM, post-hoc support estimation, pruning, and query
% evaluation.
% The builder never fills missing class-parameter defaults; defaults must be produced by the external
% configuration constructor before this function is called.
%
% Input:
%   cfg: map configuration struct with defaultParams and classParams fields
%   classLabel: string scalar semantic class label
%
% Output:
%   params: validated scalar parameter struct for one semantic class
    rejectRemovedModeFields(cfg.defaultParams, "cfg.defaultParams");
    params = cfg.defaultParams;
    params.classLabel = string(classLabel);

    if isfield(cfg, "classParams") && ~isempty(cfg.classParams)
        if ~isfield(cfg.classParams, "classLabel")
            error("buildTemporalStabilityGmmMap:MissingClassParamLabel", ...
                "cfg.classParams entries must contain classLabel before class-specific overrides can be merged.");
        end
        classParamLabels = string({cfg.classParams.classLabel});
        matchIdx = find(classParamLabels == string(classLabel), 1);
        if ~isempty(matchIdx)
            override = cfg.classParams(matchIdx);
            rejectRemovedModeFields(override, "cfg.classParams");
            fieldNames = fieldnames(override);
            for fieldIdx = 1:numel(fieldNames)
                fieldName = fieldNames{fieldIdx};
                if string(fieldName) ~= "classLabel"
                    params.(fieldName) = override.(fieldName);
                end
            end
        end
    end

    finiteRequiredFields = ["k", "radius", "maxDiameter", "lengthParallel", "lengthPerp", ...
        "priorSupport", "timestampMaxBins", "normalStabilityLength", ...
        "compactStabilityLength", "geometricElongatedAnisotropyThreshold", ...
        "effectiveFramePrior", "minComponentSupport", "minTimestampBins", ...
        "minComponentPoints", "minNormalStability", "emMaxIterations", "emTolerance", ...
        "minCovarianceEigenvalue", "maxCovarianceEigenvalue", "emMinEffectiveSupport", ...
        "emMinFrameWeight", "querySupportMahalanobisRadius", ...
        "temporalReliabilityKernelBandwidth", "temporalReliabilityKernelRadiusMultiplier", ...
        "temporalReliabilityPower", "temporalReliabilitySamplingFloor", ...
        "temporalResampleExpectedCountMultiplier", "temporalResampleRandomSeed", ...
        "minPatchSupportForGmmSeed"];
    for fieldName = finiteRequiredFields
        fieldNameChar = char(fieldName);
        if ~isfield(params, fieldNameChar)
            error("buildTemporalStabilityGmmMap:MissingClassParameter", ...
                "Missing class parameter field: %s.", fieldNameChar);
        end
        assert(isscalar(params.(fieldNameChar)) && isnumeric(params.(fieldNameChar)) && isfinite(params.(fieldNameChar)), ...
            "Class parameter %s must be a finite numeric scalar.", fieldNameChar);
    end

    boundedRequiredFields = ["queryCandidateComponentCount", "queryCandidateRadius"];
    for fieldName = boundedRequiredFields
        fieldNameChar = char(fieldName);
        if ~isfield(params, fieldNameChar)
            error("buildTemporalStabilityGmmMap:MissingClassParameter", ...
                "Missing class parameter field: %s.", fieldNameChar);
        end
        assert(isscalar(params.(fieldNameChar)) && isnumeric(params.(fieldNameChar)) && ~isnan(params.(fieldNameChar)), ...
            "Class parameter %s must be a numeric scalar.", fieldNameChar);
    end

    if ~isfield(params, "frameCountSaturation")
        error("buildTemporalStabilityGmmMap:MissingClassParameter", ...
            "Missing class parameter field: frameCountSaturation.");
    end
    if ~isempty(params.frameCountSaturation)
        assert(isscalar(params.frameCountSaturation) && isnumeric(params.frameCountSaturation) && isfinite(params.frameCountSaturation) && params.frameCountSaturation > 0, ...
            "frameCountSaturation must be empty or a positive finite numeric scalar.");
    end
    if ~isfield(params, "storeEmAssignments")
        error("buildTemporalStabilityGmmMap:MissingClassParameter", ...
            "Missing class parameter field: storeEmAssignments.");
    end
    assert(isscalar(params.storeEmAssignments) && (islogical(params.storeEmAssignments) || (isnumeric(params.storeEmAssignments) && isfinite(params.storeEmAssignments))), ...
        "storeEmAssignments must be a logical or finite numeric scalar.");
    if ~isfield(params, "refitAfterTemporalPruning")
        error("buildTemporalStabilityGmmMap:MissingClassParameter", ...
            "Missing class parameter field: refitAfterTemporalPruning.");
    end
    assert(isscalar(params.refitAfterTemporalPruning) && (islogical(params.refitAfterTemporalPruning) || (isnumeric(params.refitAfterTemporalPruning) && isfinite(params.refitAfterTemporalPruning))), ...
        "refitAfterTemporalPruning must be a logical or finite numeric scalar.");
    assert(params.k >= 1 && params.k == fix(params.k), ...
        "k must be a positive integer.");
    if params.minTimestampBins < 2 || params.minTimestampBins ~= fix(params.minTimestampBins)
        error("buildTemporalStabilityGmmMap:InvalidMinTimestampBins", ...
            "minTimestampBins must be an integer greater than or equal to 2 for cross-frame temporal-geometric stability.");
    end
    assert(params.minComponentPoints >= 1 && params.minComponentPoints == fix(params.minComponentPoints), ...
        "minComponentPoints must be a positive integer.");
    assert(params.emMaxIterations >= 1 && params.emMaxIterations == fix(params.emMaxIterations), ...
        "emMaxIterations must be a positive integer.");
    if isfinite(params.queryCandidateComponentCount)
        assert(params.queryCandidateComponentCount >= 1 && params.queryCandidateComponentCount == fix(params.queryCandidateComponentCount), ...
            "queryCandidateComponentCount must be a positive integer or Inf.");
    end
    if isnumeric(params.storeEmAssignments)
        assert(params.storeEmAssignments == 0 || params.storeEmAssignments == 1, ...
            "numeric storeEmAssignments must be 0 or 1.");
    end
    params.storeEmAssignments = logical(params.storeEmAssignments);
    if isnumeric(params.refitAfterTemporalPruning)
        assert(params.refitAfterTemporalPruning == 0 || params.refitAfterTemporalPruning == 1, ...
            "numeric refitAfterTemporalPruning must be 0 or 1.");
    end
    params.refitAfterTemporalPruning = logical(params.refitAfterTemporalPruning);
    assert(params.radius > 0 && params.maxDiameter > 0, ...
        "radius and maxDiameter must be positive.");
    assert(params.lengthParallel > 0 && params.lengthPerp > 0, ...
        "lengthParallel and lengthPerp must be positive patch-initialization covariance scales.");
    assert(params.priorSupport >= 0 && params.priorSupport <= 1, ...
        "priorSupport must be in [0, 1].");
    assert(params.timestampMaxBins >= 1, ...
        "timestampMaxBins must be greater than or equal to 1.");
    if params.timestampMaxBins < params.minTimestampBins
        error("buildTemporalStabilityGmmMap:InvalidTimestampMaxBins", ...
            "timestampMaxBins must be greater than or equal to minTimestampBins.");
    end
    if params.normalStabilityLength <= 0
        error("buildTemporalStabilityGmmMap:InvalidGeometricStabilityConfiguration", ...
            "normalStabilityLength must be positive.");
    end
    if params.compactStabilityLength <= 0
        error("buildTemporalStabilityGmmMap:InvalidGeometricStabilityConfiguration", ...
            "compactStabilityLength must be positive.");
    end
    if params.geometricElongatedAnisotropyThreshold <= 1
        error("buildTemporalStabilityGmmMap:InvalidGeometricStabilityConfiguration", ...
            "geometricElongatedAnisotropyThreshold must be greater than one.");
    end
    assert(params.effectiveFramePrior > 0, ...
        "effectiveFramePrior must be positive.");
    if params.minCovarianceEigenvalue <= 0 || params.maxCovarianceEigenvalue < params.minCovarianceEigenvalue
        error("buildTemporalStabilityGmmMap:InvalidCovarianceEigenvalueBounds", ...
            "minCovarianceEigenvalue must be positive and maxCovarianceEigenvalue must be greater than or equal to minCovarianceEigenvalue.");
    end
    assert(params.minComponentSupport >= 0 && params.minComponentSupport <= 1, ...
        "minComponentSupport must be in [0, 1].");
    assert(params.minNormalStability >= 0 && params.minNormalStability <= 1, ...
        "minNormalStability must be in [0, 1].");
    assert(params.queryCandidateComponentCount > 0, ...
        "queryCandidateComponentCount must be positive.");
    assert(params.queryCandidateRadius > 0, ...
        "queryCandidateRadius must be positive.");
    if params.querySupportMahalanobisRadius <= 0
        error("buildTemporalStabilityGmmMap:InvalidQuerySupportMahalanobisRadius", ...
            "querySupportMahalanobisRadius must be positive.");
    end
    if params.temporalReliabilityKernelBandwidth <= 0 || params.temporalReliabilityKernelRadiusMultiplier <= 0
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityConfiguration", ...
            "temporalReliabilityKernelBandwidth and temporalReliabilityKernelRadiusMultiplier must be positive.");
    end
    if params.temporalReliabilityPower < 1
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityConfiguration", ...
            "temporalReliabilityPower must be greater than or equal to one.");
    end
    if params.temporalReliabilitySamplingFloor < 0
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityConfiguration", ...
            "temporalReliabilitySamplingFloor must be nonnegative.");
    end
    if params.temporalResampleExpectedCountMultiplier <= 0
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityConfiguration", ...
            "temporalResampleExpectedCountMultiplier must be positive.");
    end
    if params.temporalResampleRandomSeed < 0 || params.temporalResampleRandomSeed ~= fix(params.temporalResampleRandomSeed)
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityConfiguration", ...
            "temporalResampleRandomSeed must be a nonnegative integer.");
    end
    if params.minPatchSupportForGmmSeed < 0 || params.minPatchSupportForGmmSeed > 1
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityConfiguration", ...
            "minPatchSupportForGmmSeed must be in [0, 1].");
    end
    assert(params.emTolerance > 0, ...
        "emTolerance must be positive.");
    assert(params.emMinEffectiveSupport > 0, ...
        "emMinEffectiveSupport must be positive.");
    assert(params.emMinFrameWeight > 0, ...
        "emMinFrameWeight must be positive.");
end

function rejectRemovedModeFields(params, contextName)
% rejectRemovedModeFields: Reject removed configuration fields that
% previously selected alternate map-building behavior. The builder now exposes
% temporal-reliability resampling followed by ordinary full-covariance Gaussian
% mixture EM and post-hoc temporal support diagnostics.
%
% Input:
%   params: scalar parameter struct to inspect
%   contextName: character vector used in diagnostics
%
% Output:
%   none
    removedFields = ["enableConstrainedEm", "enableComponentPruning", "useSampleSufficiency", "emCovarianceMode", "combineMode", "emMeanDriftLimit", "emCandidateComponentCount", "emCandidateRadius", "emGateThreshold", "emStudentTDegreesOfFreedom", "emUniformBackgroundEnabled", "emUniformBackgroundInitialWeight", "emUniformBackgroundBevBounds"];
    for fieldName = removedFields
        if isfield(params, char(fieldName))
            error("buildTemporalStabilityGmmMap:RemovedModeField", ...
                "%s.%s has been removed; buildTemporalStabilityGmmMap uses temporal-reliability resampling followed by ordinary Gaussian-mixture EM and post-hoc temporal support diagnostics.", contextName, char(fieldName));
        end
    end
end

function layer = emptyLayer()
% emptyLayer: Create an empty semantic GMM layer struct with the
% complete public field set produced by the map builder.
%
% Input:
%   none
%
% Output:
%   layer: scalar struct with initialized semantic layer fields
    layer = struct();
    layer.classLabel = "";
    layer.params = struct();
    layer.components = repmat(emptyComponent(), 0, 1);
    layer.priorScore = 0;
    layer.pointCount = 0;
    layer.sourceIndices = zeros(0, 1);
    layer.temporalReliabilityScores = zeros(0, 1);
    layer.temporalReliabilityModel = "";
    layer.samplingProbabilities = zeros(0, 1);
    layer.resampleCounts = zeros(0, 1);
    layer.resampledSourceIndices = zeros(0, 1);
    layer.resampleModel = "";
    layer.gmmTrainingModel = "";
    layer.gmmTrainingUsesTimestamps = false;
    layer.gmmTrainingUsesTemporalWeights = false;
    layer.gmmTrainingUsesStudentTWeights = false;
    layer.gmmTrainingUsesUniformBackground = false;
    layer.posthocTemporalSupportEstimated = false;
    layer.posthocTemporalSupportResponsibilities = zeros(0, 0);
    layer.componentMeans = zeros(0, 2);
    layer.componentCovariances = zeros(2, 2, 0);
    layer.componentBoundingBoxes = zeros(0, 4);
    layer.componentPointCounts = zeros(0, 1);
    layer.componentPatchBoundingBoxes = zeros(0, 4);
    layer.componentPatchPointCounts = zeros(0, 1);
    layer.componentSupportAmplitudes = zeros(0, 1);
    layer.componentMixtureWeights = zeros(0, 1);
    layer.centroidSearcher = [];
    layer.queryCandidateComponentCount = inf;
    layer.queryCandidateRadius = inf;
    layer.queryCandidateRadiusInflation = 0;
    layer.queryCandidateRetrievalModel = "";
    layer.emIterationCount = 0;
    layer.emConverged = false;
    layer.emMaxMeanShift = 0;
    layer.emLogLikelihoodTrace = zeros(0, 1);
    layer.emFinalLogLikelihood = -inf;
    layer.emObjectiveTrace = zeros(0, 1);
    layer.emFinalObjective = -inf;
    layer.emRefinementModel = "";
    layer.emStudentTDegreesOfFreedom = nan;
    layer.emUsesUniformBackground = false;
    layer.emUniformBackgroundPriorMode = "";
    layer.emFixedUniformBackgroundPrior = 0;
    layer.emUniformBackgroundPrior = 0;
    layer.emInitialUniformBackgroundPrior = 0;
    layer.emUniformBackgroundDensity = 0;
    layer.emUniformBackgroundBounds = zeros(1, 4);
    layer.emBackgroundResponsibilityMass = 0;
    layer.emBackgroundResponsibilityMin = 0;
    layer.emBackgroundResponsibilityMean = 0;
    layer.emBackgroundResponsibilityMax = 0;
    layer.emBackgroundResponsibilities = zeros(0, 1);
    layer.emForegroundResponsibilityRowSums = zeros(0, 1);
    layer.emCompleteResponsibilityRowSums = zeros(0, 1);
    layer.emForegroundResponsibilityMass = 0;
    layer.emMixtureWeightsBeforePruning = zeros(0, 1);
    layer.emResponsibilityRowSums = zeros(0, 1);
    layer.emResponsibilityRowSumSemantics = "";
    layer.emResponsibilities = zeros(0, 0);
    layer.emRobustScaleWeights = zeros(0, 0);
    layer.emRobustScaleWeightMin = nan;
    layer.emRobustScaleWeightMean = nan;
    layer.emRobustScaleWeightMax = nan;
    layer.emRobustWeightedComponentMass = zeros(0, 1);
    layer.emNumericalSafeguardMinEffectiveSupport = 0;
    layer.emNumericalFreezeMask = false(0, 1);
    layer.emNumericalFreezeCount = 0;
    layer.integratedSupportEstimated = false;
    layer.integratedSupportComponentCount = 0;
    layer.integratedSupportEstimatedAfterEmConvergence = false;
    layer.integratedSupportAssignmentModel = "";
    layer.integratedSupportUsesStudentTInlierWeights = false;
    layer.integratedSupportAssignmentWeightMin = nan;
    layer.integratedSupportAssignmentWeightMean = nan;
    layer.integratedSupportAssignmentWeightMax = nan;
    layer.integratedSupportWeightedComponentMass = zeros(0, 1);
    layer.integratedSupportAssignmentRowSums = zeros(0, 1);
    layer.integratedSupportFrameSupportModel = "";
    layer.integratedSupportTemporalDiversityModel = "";
    layer.integratedSupportSampleSufficiencyModel = "";
    layer.integratedSupportGeometricConsistencyModel = "";
    layer.pruningAppliedAfterIntegratedSupport = false;
    layer.componentCountBeforePruning = 0;
    layer.componentCountAfterPruning = 0;
    layer.integratedSupportPruningKeepMask = false(0, 1);
    layer.integratedSupportPrunedComponentCount = 0;
    layer.integratedSupportPruningInputSupportAmplitudes = zeros(0, 1);
    layer.integratedSupportPruningInputTimestampBinCounts = zeros(0, 1);
    layer.integratedSupportPruningInputEffectiveSupportPointCounts = zeros(0, 1);
    layer.integratedSupportPruningInputGeometricStability = zeros(0, 1);
end

function [componentMeans, componentCovariances, componentBoundingBoxes, componentPointCounts, componentSupportAmplitudes, componentMixtureWeights, componentPatchBoundingBoxes, componentPatchPointCounts] = buildComponentIndexData(components)
% buildComponentIndexData: Extract compact per-component metadata for
% query-time candidate selection and diagnostics without changing stored
% patch provenance or refined component support models.
%
% Input:
%   components: struct array of structured Gaussian support components
%
% Output:
%   componentMeans: [J x 2] component mean coordinates
%   componentCovariances: [2 x 2 x J] component covariance matrices
%   componentBoundingBoxes: [J x 4] refined support [minX minY maxX maxY] bounds
%   componentPointCounts: [J x 1] effective support point counts
%   componentSupportAmplitudes: [J x 1] component support amplitudes
%   componentMixtureWeights: [J x 1] EM mixture weights preserved on
%       retained support components
%   componentPatchBoundingBoxes: [J x 4] original patch bounding boxes
%   componentPatchPointCounts: [J x 1] original patch point counts
    componentCount = numel(components);
    componentMeans = zeros(componentCount, 2);
    componentCovariances = zeros(2, 2, componentCount);
    componentBoundingBoxes = zeros(componentCount, 4);
    componentPointCounts = zeros(componentCount, 1);
    componentSupportAmplitudes = zeros(componentCount, 1);
    componentMixtureWeights = zeros(componentCount, 1);
    componentPatchBoundingBoxes = zeros(componentCount, 4);
    componentPatchPointCounts = zeros(componentCount, 1);

    for componentIdx = 1:componentCount
        componentMeans(componentIdx, :) = components(componentIdx).mean;
        componentCovariances(:, :, componentIdx) = components(componentIdx).covariance;
        componentBoundingBoxes(componentIdx, :) = components(componentIdx).supportBoundingBox;
        componentPointCounts(componentIdx) = components(componentIdx).effectiveSupportPointCount;
        componentSupportAmplitudes(componentIdx) = components(componentIdx).supportAmplitude;
        componentMixtureWeights(componentIdx) = components(componentIdx).mixtureWeight;
        componentPatchBoundingBoxes(componentIdx, :) = components(componentIdx).patchBoundingBox;
        componentPatchPointCounts(componentIdx) = components(componentIdx).patchPointCount;
    end
end

function centroidSearcher = buildCentroidSearcher(componentMeans)
% buildCentroidSearcher: Build a KD-tree over component means for
% query-time component preselection. Empty retained component sets are invalid
% under fail-fast map construction.
%
% Input:
%   componentMeans: [J x 2] component mean coordinates
%
% Output:
%   centroidSearcher: KDTreeSearcher object or empty array
    if isempty(componentMeans)
        error("buildTemporalStabilityGmmMap:EmptyCentroidIndex", ...
            "Cannot build a centroid searcher for an empty retained component set.");
    end
    assert(exist("KDTreeSearcher", "class") == 8, ...
        "KDTreeSearcher is required for optimized component indexing.");
    centroidSearcher = KDTreeSearcher(componentMeans);
end

function queryCandidateRadiusInflation = queryCandidateRadiusInflation(components)
% queryCandidateRadiusInflation: Compute a layer-level covariance extent
% used by query-time centroid candidate retrieval so elongated or broad
% components are not missed solely because their means are farther away in
% Euclidean distance than the configured base candidate radius.
%
% Input:
%   components: retained foreground support component struct array
%
% Output:
%   queryCandidateRadiusInflation: scalar maximum component query support radius
    if isempty(components) || ~isfield(components, "querySupportRadius")
        queryCandidateRadiusInflation = 0;
        return;
    end
    querySupportRadii = [components.querySupportRadius].';
    if any(~isfinite(querySupportRadii)) || any(querySupportRadii <= 0)
        error("buildTemporalStabilityGmmMap:InvalidQuerySupportRadius", ...
            "Retained components must contain finite positive query support radii for candidate retrieval diagnostics.");
    end
    queryCandidateRadiusInflation = max(querySupportRadii);
end

function storedResponsibilities = storedResponsibilities(responsibilities, params)
% storedResponsibilities: Store dense EM responsibility-like matrices only
% when requested by diagnostics configuration.
%
% Input:
%   responsibilities: [N x K] final EM responsibility-like matrix
%   params: class parameter struct with storeEmAssignments flag
%
% Output:
%   storedResponsibilities: [N x K] responsibilities or an empty matrix
    if params.storeEmAssignments
        storedResponsibilities = responsibilities;
    else
        storedResponsibilities = zeros(0, 0);
    end
end

function storedValues = storedVector(values, params)
% storedVector: Store a dense EM diagnostic vector only when
% requested by diagnostics configuration.
%
% Input:
%   values: [N x 1] diagnostic vector
%   params: class parameter struct with storeEmAssignments flag
%
% Output:
%   storedValues: [N x 1] diagnostic vector or an empty vector
    if params.storeEmAssignments
        storedValues = values(:);
    else
        storedValues = zeros(0, 1);
    end
end

function tf = isLogEnabled(cfg)
% isLogEnabled: Resolve the optional command-window diagnostic logging
% switch from the map builder configuration.
%
% Input:
%   cfg: map builder configuration struct with optional logEnabled field
%
% Output:
%   tf: logical scalar indicating whether structured builder logs are enabled
    tf = false;
    if isstruct(cfg) && isfield(cfg, "logEnabled") && isscalar(cfg.logEnabled)
        tf = logical(cfg.logEnabled);
    end
end

function logStep(logEnabled, logKey, message, varargin)
% logStep: Print one structured command-window diagnostic line for the
% semantic temporal-stability GMM builder when logging is enabled.
%
% Input:
%   logEnabled: logical scalar diagnostic switch
%   logKey: string scalar diagnostic key
%   message: sprintf-compatible message format
%   varargin: values consumed by the message format
%
% Output:
%   none
    if ~logEnabled
        return;
    end
    fprintf("[gmm-map][%s] %s\n", char(string(logKey)), sprintf(char(string(message)), varargin{:}));
end

function logPatchFilterSummary(logEnabled, classLabel, patchLocalIndices, removedPatchCount)
% logPatchFilterSummary: Print the retained patch count after removing
% patches that cannot support PCA-based component initialization.
%
% Input:
%   logEnabled: logical scalar diagnostic switch
%   classLabel: string scalar semantic class label
%   patchLocalIndices: retained cell array of class-local patch index vectors
%   removedPatchCount: scalar count of patches removed before component build
%
% Output:
%   none
    patchSizes = cellfun(@numel, patchLocalIndices(:));
    [minPatchSize, medianPatchSize, maxPatchSize] = finiteSummary(patchSizes);
    logStep(logEnabled, "class.patches.filtered", "class=%s | retainedPatches=%d | removedPatches=%d | patchSize[min/med/max]=%.4g/%.4g/%.4g", ...
        char(string(classLabel)), numel(patchLocalIndices), round(double(removedPatchCount)), minPatchSize, medianPatchSize, maxPatchSize);
end

function logAmplitudeSummary(logEnabled, classLabel, components, amplitudeDiagnostics)
% logAmplitudeSummary: Print post-hoc temporal support amplitude
% diagnostics for one semantic class before pruning.
%
% Input:
%   logEnabled: logical scalar diagnostic switch
%   classLabel: string scalar semantic class label
%   components: struct array after ordinary GMM and post-hoc support scoring
%   amplitudeDiagnostics: scalar post-hoc support diagnostics
%
% Output:
%   none
    supportAmplitudes = [components.supportAmplitude].';
    effectiveSupportPointCounts = [components.effectiveSupportPointCount].';
    timestampBinCounts = [components.timestampBinCount].';
    [minSupport, medianSupport, maxSupport] = finiteSummary(supportAmplitudes);
    [minEffectivePoints, medianEffectivePoints, maxEffectivePoints] = finiteSummary(effectiveSupportPointCounts);
    [minTimestampBins, medianTimestampBins, maxTimestampBins] = finiteSummary(timestampBinCounts);
    logStep(logEnabled, "class.temporalSupport", "class=%s | completed=%d | components=%d | support[min/med/max]=%.4g/%.4g/%.4g | effectivePoints[min/med/max]=%.4g/%.4g/%.4g | timestampBins[min/med/max]=%.4g/%.4g/%.4g", ...
        char(string(classLabel)), amplitudeDiagnostics.completed, numel(components), minSupport, medianSupport, maxSupport, ...
        minEffectivePoints, medianEffectivePoints, maxEffectivePoints, minTimestampBins, medianTimestampBins, maxTimestampBins);
end
