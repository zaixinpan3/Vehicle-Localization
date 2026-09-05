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
% XYZ input additionally fits a responsibility-weighted conditional Gaussian
% p(z|XY,component) on the same resampled returns after the XY model is final.
% This retains full XYZ covariance and leaves the XY fit and query unchanged.
%
% Input:
%   points: [N x 2] BEV or [N x 3] global XYZ feature point coordinates
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
    assert(isnumeric(points) && ismatrix(points) && ismember(size(points, 2), [2 3]), ...
        "points must be an [N x 2] or [N x 3] numeric array.");
    assert(numel(labels) == size(points, 1), ...
        "labels must contain one class label per point.");
    assert(numel(timestamps) == size(points, 1), ...
        "timestamps must contain one frame or observation id per point.");
    assert(isstruct(cfg) && isfield(cfg, "defaultParams"), ...
        "cfg must be a struct from temporalStabilityMapConfig or a compatible struct.");

    points = double(points);
    assert(all(isfinite(points), "all"), ...
        "points must contain only finite numeric values.");
    sourcePoints = points;
    points = points(:, 1:2);
    heightVarianceFloor = 1.0e-4;
    if isfield(cfg, "minimumConditionalHeightVariance")
        heightVarianceFloor = cfg.minimumConditionalHeightVariance;
    end
    assert(isscalar(heightVarianceFloor) && isfinite(heightVarianceFloor) && heightVarianceFloor > 0);
    assert(isfield(cfg, "pcaEpsilon") && isscalar(cfg.pcaEpsilon) && isnumeric(cfg.pcaEpsilon) && isfinite(cfg.pcaEpsilon) && cfg.pcaEpsilon > 0, ...
        "cfg.pcaEpsilon must be a positive finite numeric scalar.");
    labelKeys = string(labels(:));
    timestampBins = binTimestamps(timestamps);
    classLabels = resolveClassLabels(cfg);
    logEnabled = mappingSupport.isLogEnabled(cfg, "logEnabled");
    mappingSupport.printLog(logEnabled, "gmm-map", "start", "points=%d | classes=%s", size(points, 1), char(strjoin(classLabels(:).', ", ")));
    layers = repmat(emptyLayer(), numel(classLabels), 1);

    for classIdx = 1:numel(classLabels)
        classLabel = classLabels(classIdx);
        params = resolveClassParams(cfg, classLabel);
        classMask = labelKeys == classLabel;
        classPoints = points(classMask, :);
        classTimestampBins = timestampBins(classMask);
        sourceIndices = find(classMask);
        mappingSupport.printLog(logEnabled, "gmm-map", "class.start", "class=%s | points=%d | timestampBins=%d | k=%d | radius=%.4g | maxDiameter=%.4g | minComponentPoints=%d", ...
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
        mappingSupport.printLog(logEnabled, "gmm-map", "class.reliability", "class=%s | scoreMin=%.4g | scoreMedian=%.4g | scoreMax=%.4g", ...
            char(classLabel), stability.scoreMin, stability.scoreMedian, stability.scoreMax);
        [sampledPoints, sampledSourceIndices, resampleDiagnostics] = resampleByTemporalReliability(classPoints, sourceIndices, stability, params);
        mappingSupport.printLog(logEnabled, "gmm-map", "class.resample", "class=%s | expected=%d | selected=%d | fill=%d", ...
            char(classLabel), round(resampleDiagnostics.expectedSampleCount), size(sampledPoints, 1), resampleDiagnostics.deterministicFillCount);
        seeds = initializeGaussianMixtureSeeds(classPoints, patchLocalIndices, patchComponents, stability, resampleDiagnostics.resampleCounts, params);
        [components, refinementDiagnostics, finalResponsibilities] = fitGaussianMixture(sampledPoints, seeds, params, cfg.pcaEpsilon);
        mappingSupport.printLog(logEnabled, "gmm-map", "class.standardEm", "class=%s | components=%d | emIter=%d | converged=%d | finalLogLikelihood=%.6g", ...
            char(classLabel), numel(components), refinementDiagnostics.iterationCount, refinementDiagnostics.converged, refinementDiagnostics.finalLogLikelihood);
        supportDiagnostics = computePosthocTemporalSupport(classPoints, sourceIndices, classTimestampBins, stability, components, params);
        supportDiagnostics.emConvergedAtSupportEstimation = refinementDiagnostics.converged;
        components = applyPosthocSupportFields(classPoints, sourceIndices, classTimestampBins, supportDiagnostics, components, params, refinementDiagnostics.converged);
        logAmplitudeSummary(logEnabled, classLabel, components, supportDiagnostics);
        [components, pruningDiagnostics] = pruneComponentsByPosthocSupport(components, params);
        mappingSupport.printLog(logEnabled, "gmm-map", "class.prune", "class=%s | before=%d | after=%d | removed=%d", ...
            char(classLabel), pruningDiagnostics.inputComponentCount, pruningDiagnostics.outputComponentCount, pruningDiagnostics.removedCount);
        if params.refitAfterTemporalPruning
            seeds = reseedFromRetainedComponents(components);
            [components, refinementDiagnostics, finalResponsibilities] = fitGaussianMixture(sampledPoints, seeds, params, cfg.pcaEpsilon);
            supportDiagnostics = computePosthocTemporalSupport(classPoints, sourceIndices, classTimestampBins, stability, components, params);
            supportDiagnostics.emConvergedAtSupportEstimation = refinementDiagnostics.converged;
            components = applyPosthocSupportFields(classPoints, sourceIndices, classTimestampBins, supportDiagnostics, components, params, refinementDiagnostics.converged);
            mappingSupport.printLog(logEnabled, "gmm-map", "class.refit", "class=%s | components=%d | emIter=%d | converged=%d | finalLogLikelihood=%.6g", ...
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
        if size(sourcePoints, 2) == 3
            [meansXYZ, covariancesXYZ] = fitConditionalHeight( ...
                sourcePoints(sampledSourceIndices, :), components, heightVarianceFloor);
            layers(classIdx).componentMeansXYZ = meansXYZ;
            layers(classIdx).componentCovariancesXYZ = covariancesXYZ;
        end
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
    gmmMap.spatialDimension = size(sourcePoints, 2);
    gmmMap.heightModel = "unavailable";
    if size(sourcePoints, 2) == 3
        gmmMap.heightModel = "conditionalGaussianGivenXY";
    end
end

function timestampBins = binTimestamps(timestamps)
% binTimestamps: Convert per-point frame ids, keyframe ids, traversal
% ids, or categorical observation ids into discrete observation-bin labels.
% Numeric and nonnumeric values are treated as already-discrete frame-bin
% identifiers, so different frame ids are never merged by this builder.
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
    if params.storeEmAssignments
        storedResponsibilities = responsibilities;
    else
        storedResponsibilities = zeros(0, 0);
    end
end

function storedValues = storedVector(values, params)
% storedVector: Store a dense EM diagnostic vector only when
% requested by diagnostics configuration.
    if params.storeEmAssignments
        storedValues = values(:);
    else
        storedValues = zeros(0, 1);
    end
end

function logPatchFilterSummary(logEnabled, classLabel, patchLocalIndices, removedPatchCount)
% logPatchFilterSummary: Print the retained patch count after removing
% patches that cannot support PCA-based component initialization.
    patchSizes = cellfun(@numel, patchLocalIndices(:));
    [minPatchSize, medianPatchSize, maxPatchSize] = mappingSupport.finiteSummary(patchSizes);
    mappingSupport.printLog(logEnabled, "gmm-map", "class.patches.filtered", "class=%s | retainedPatches=%d | removedPatches=%d | patchSize[min/med/max]=%.4g/%.4g/%.4g", ...
        char(string(classLabel)), numel(patchLocalIndices), round(double(removedPatchCount)), minPatchSize, medianPatchSize, maxPatchSize);
end

function logAmplitudeSummary(logEnabled, classLabel, components, amplitudeDiagnostics)
% logAmplitudeSummary: Print post-hoc temporal support amplitude
% diagnostics for one semantic class before pruning.
    supportAmplitudes = [components.supportAmplitude].';
    effectiveSupportPointCounts = [components.effectiveSupportPointCount].';
    timestampBinCounts = [components.timestampBinCount].';
    [minSupport, medianSupport, maxSupport] = mappingSupport.finiteSummary(supportAmplitudes);
    [minEffectivePoints, medianEffectivePoints, maxEffectivePoints] = mappingSupport.finiteSummary(effectiveSupportPointCounts);
    [minTimestampBins, medianTimestampBins, maxTimestampBins] = mappingSupport.finiteSummary(timestampBinCounts);
    mappingSupport.printLog(logEnabled, "gmm-map", "class.temporalSupport", "class=%s | completed=%d | components=%d | support[min/med/max]=%.4g/%.4g/%.4g | effectivePoints[min/med/max]=%.4g/%.4g/%.4g | timestampBins[min/med/max]=%.4g/%.4g/%.4g", ...
        char(string(classLabel)), amplitudeDiagnostics.completed, numel(components), minSupport, medianSupport, maxSupport, ...
        minEffectivePoints, medianEffectivePoints, maxEffectivePoints, minTimestampBins, medianTimestampBins, maxTimestampBins);
end

function [sampledPoints, sampledSourceIndices, resampleDiagnostics] = resampleByTemporalReliability(points, sourceIndices, stability, params)
% resampleByTemporalReliability: Convert temporal reliability scores into
% normalized class-local sampling probabilities and draw an ordinary GMM
% training set with independent Bernoulli sampling. A deterministic minimum
% fill preserves a valid fixed-order GMM training set when a random draw is too
% sparse.
    pointCount = size(points, 1);
    if numel(sourceIndices) ~= pointCount || numel(stability.scores) ~= pointCount
        error("buildTemporalStabilityGmmMap:InvalidResampleInput", ...
            "Temporal reliability resampling requires one source index and one reliability score per point.");
    end
    samplingWeights = params.temporalReliabilitySamplingFloor + stability.scores(:) .^ params.temporalReliabilityPower;
    if any(~isfinite(samplingWeights)) || any(samplingWeights < 0) || sum(samplingWeights) <= 0
        error("buildTemporalStabilityGmmMap:InvalidSamplingWeights", ...
            "Temporal reliability sampling weights must be finite nonnegative values with positive total mass.");
    end
    samplingProbabilities = samplingWeights ./ sum(samplingWeights);
    expectedSampleCount = pointCount .* params.temporalResampleExpectedCountMultiplier;
    bernoulliProbabilities = min(1, expectedSampleCount .* samplingProbabilities);
    stream = RandStream("mt19937ar", "Seed", params.temporalResampleRandomSeed);
    sampleMask = rand(stream, pointCount, 1) <= bernoulliProbabilities;
    minimumSampleCount = min(pointCount, max(2, min(numel(stability.patchScores), pointCount)));
    deterministicFillCount = 0;
    if nnz(sampleMask) < minimumSampleCount
        missingCount = minimumSampleCount - nnz(sampleMask);
        [~, fillOrder] = sort(bernoulliProbabilities, "descend");
        fillOrder = fillOrder(~sampleMask(fillOrder));
        fillIdx = fillOrder(1:min(missingCount, numel(fillOrder)));
        sampleMask(fillIdx) = true;
        deterministicFillCount = numel(fillIdx);
    end
    if ~any(sampleMask)
        error("buildTemporalStabilityGmmMap:EmptyResample", ...
            "Temporal reliability resampling produced an empty GMM training set.");
    end
    sampledPoints = points(sampleMask, :);
    sampledSourceIndices = sourceIndices(sampleMask);
    resampleDiagnostics = struct();
    resampleDiagnostics.model = "independentBernoulliTemporalReliabilitySampling";
    resampleDiagnostics.samplingProbabilities = samplingProbabilities(:);
    resampleDiagnostics.bernoulliProbabilities = bernoulliProbabilities(:);
    resampleDiagnostics.resampleCounts = double(sampleMask(:));
    resampleDiagnostics.expectedSampleCount = expectedSampleCount;
    resampleDiagnostics.selectedSampleCount = size(sampledPoints, 1);
    resampleDiagnostics.deterministicFillCount = deterministicFillCount;
end

function seeds = initializeGaussianMixtureSeeds(points, patchLocalIndices, patchComponents, stability, resampleCounts, params)
% initializeGaussianMixtureSeeds: Select and weight patch-initialized Gaussian
% components before standard EM. Patch geometry and temporal reliability may
% influence initialization and model order, but these seeds do not constrain
% later ordinary EM updates.
    patchCount = numel(patchComponents);
    if patchCount < 1 || numel(patchLocalIndices) ~= patchCount || numel(stability.patchScores) ~= patchCount
        error("buildTemporalStabilityGmmMap:InvalidGmmSeedInput", ...
            "GMM seed construction requires matching nonempty patches, patch components, and patch scores.");
    end
    patchResampledMass = zeros(patchCount, 1);
    for patchIdx = 1:patchCount
        localIndices = patchLocalIndices{patchIdx}(:);
        patchResampledMass(patchIdx) = sum(resampleCounts(localIndices));
    end
    keepMask = stability.patchScores(:) >= params.minPatchSupportForGmmSeed & patchResampledMass > 0;
    if ~any(keepMask)
        sampledPatchMask = patchResampledMass > 0;
        if ~any(sampledPatchMask)
            error("buildTemporalStabilityGmmMap:NoResampledSeedPatch", ...
                "GMM seed construction requires at least one retained patch with resampled training mass.");
        end
        [~, bestPatchOffset] = max(stability.patchScores(sampledPatchMask));
        sampledPatchIdx = find(sampledPatchMask);
        keepMask(sampledPatchIdx(bestPatchOffset)) = true;
    end
    keptPatchIdx = find(keepMask);
    patchMass = patchResampledMass + eps;
    [~, orderIdx] = sort(patchMass(keptPatchIdx), "descend");
    keptPatchIdx = keptPatchIdx(orderIdx);
    maxSeedCount = min(numel(keptPatchIdx), max(1, nnz(resampleCounts)));
    seeds = patchComponents(keptPatchIdx(1:maxSeedCount));
    seedMass = patchMass(keptPatchIdx(1:maxSeedCount));
    if any(~isfinite(seedMass)) || any(seedMass <= 0) || sum(seedMass) <= 0
        error("buildTemporalStabilityGmmMap:InvalidGmmSeedMass", ...
            "Patch-based GMM seed masses must be finite positive values.");
    end
    mixtureWeights = seedMass ./ sum(seedMass);
    for componentIdx = 1:numel(seeds)
        seeds(componentIdx).mixtureWeight = mixtureWeights(componentIdx);
        seeds(componentIdx).initialMixtureWeight = mixtureWeights(componentIdx);
        seeds(componentIdx).emMixtureWeightBeforePruning = mixtureWeights(componentIdx);
        seeds(componentIdx).pointCount = size(points, 1) .* mixtureWeights(componentIdx);
    end
end

function [components, pruningDiagnostics] = pruneComponentsByPosthocSupport(components, params)
% pruneComponentsByPosthocSupport: Remove structured GMM components whose
% post-hoc support amplitude, frame-bin count, point count, or geometric
% stability is below configured class thresholds after ordinary EM has
% completed.
    pruningDiagnostics = struct();
    pruningDiagnostics.appliedAfterIntegratedSupport = true;
    pruningDiagnostics.appliedAfterPosthocSupport = true;
    pruningDiagnostics.inputComponentCount = numel(components);
    pruningDiagnostics.outputComponentCount = numel(components);
    pruningDiagnostics.keepMask = false(numel(components), 1);
    pruningDiagnostics.removedCount = 0;
    pruningDiagnostics.inputSupportAmplitudes = zeros(numel(components), 1);
    pruningDiagnostics.inputTimestampBinCounts = zeros(numel(components), 1);
    pruningDiagnostics.inputEffectiveSupportPointCounts = zeros(numel(components), 1);
    pruningDiagnostics.inputGeometricStability = zeros(numel(components), 1);

    if isempty(components)
        error("buildTemporalStabilityGmmMap:NoComponentsBeforePruning", ...
            "No post-hoc temporal support components are available before pruning; empty component layers are invalid.");
    end

    keepMask = false(numel(components), 1);
    for componentIdx = 1:numel(components)
        pruningDiagnostics.inputSupportAmplitudes(componentIdx) = components(componentIdx).supportAmplitude;
        pruningDiagnostics.inputTimestampBinCounts(componentIdx) = components(componentIdx).timestampBinCount;
        pruningDiagnostics.inputEffectiveSupportPointCounts(componentIdx) = components(componentIdx).effectiveSupportPointCount;
        pruningDiagnostics.inputGeometricStability(componentIdx) = components(componentIdx).geometricStability;
        keepMask(componentIdx) = components(componentIdx).supportAmplitude >= params.minComponentSupport && ...
            components(componentIdx).timestampBinCount >= params.minTimestampBins && ...
            components(componentIdx).effectiveSupportPointCount >= params.minComponentPoints && ...
            components(componentIdx).normalStability >= params.minNormalStability;
    end
    pruningDiagnostics.keepMask = keepMask;
    components = components(keepMask);
    pruningDiagnostics.outputComponentCount = numel(components);
    pruningDiagnostics.removedCount = pruningDiagnostics.inputComponentCount - pruningDiagnostics.outputComponentCount;
    if isempty(components)
        error("buildTemporalStabilityGmmMap:NoRetainedComponents", ...
            "Post-hoc temporal support pruning removed every component; empty retained component layers are invalid.");
    end
end

function seeds = reseedFromRetainedComponents(components)
% reseedFromRetainedComponents: Convert post-hoc retained components
% into a normalized initialization for the final ordinary Gaussian EM refit.
    if isempty(components)
        error("buildTemporalStabilityGmmMap:InvalidRefitSeeds", ...
            "Final standard GMM refit requires at least one retained component.");
    end
    seeds = components;
    mixtureWeights = [seeds.mixtureWeight].';
    if any(~isfinite(mixtureWeights)) || any(mixtureWeights <= 0) || sum(mixtureWeights) <= 0
        mixtureWeights = max([seeds.supportAmplitude].', eps);
    end
    mixtureWeights = mixtureWeights ./ sum(mixtureWeights);
    for componentIdx = 1:numel(seeds)
        seeds(componentIdx).mixtureWeight = mixtureWeights(componentIdx);
        seeds(componentIdx).initialMixtureWeight = mixtureWeights(componentIdx);
    end
end

function [meansXYZ, covariancesXYZ] = fitConditionalHeight(points, components, varianceFloor)
% fitConditionalHeight: Fit p(z|XY,k) after the XY temporal GMM is finalized.
% Use the same temporally resampled returns and final XY responsibilities.
% This conditional regression preserves the existing XY marginal exactly.
    normalized = components;
    mass = sum([normalized.mixtureWeight]);
    for k = 1:numel(normalized)
        normalized(k).mixtureWeight = normalized(k).mixtureWeight/mass;
    end
    responsibility = gaussianExpectation(points(:, 1:2), normalized);
    meansXYZ = zeros(numel(components), 3);
    covariancesXYZ = zeros(3, 3, numel(components));
    for k = 1:numel(components)
        weight = responsibility(:, k);
        assert(sum(weight) > 0, 'Height component has no sample support.');
        weight = weight/sum(weight);
        origin = components(k).mean;
        xy = points(:, 1:2)-origin;
        meanXY = weight.'*xy;
        zOrigin = points(1, 3);
        z = points(:, 3)-zOrigin;
        meanZ = weight.'*z;
        dx = xy-meanXY;
        dz = z-meanZ;
        scatter = dx.'*(weight.*dx);
        slope = pinv(scatter)*(dx.'*(weight.*dz));
        residual = dz-dx*slope;
        conditionalVariance = max(sum(weight.*residual.^2), varianceFloor);
        covarianceXY = components(k).covariance;
        cross = covarianceXY*slope;
        meansXYZ(k, :) = [origin, zOrigin+meanZ-meanXY*slope];
        covariancesXYZ(:, :, k) = [covarianceXY, cross; cross.', conditionalVariance+slope.'*cross];
    end
end

function [covariance, covarianceDiagnostics] = applyCovarianceEigenvalueSafeguards(covariance, t, n, params)
% applyCovarianceEigenvalueSafeguards: Bound covariance eigenvalues in the
% component's current orthonormal frame so peak-normalized query support cannot
% become an unbounded broad attraction basin. The operation preserves the
% supplied EM orientation, symmetry, and positive definiteness.
    mappingSupport.validateCovarianceMatrix(covariance, "Covariance eigenvalue safeguarding");
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
    mappingSupport.validateCovarianceMatrix(covariance, "Bounded covariance");
    covarianceDiagnostics = struct();
    covarianceDiagnostics.eigenvaluesBefore = eigenvaluesBefore;
    covarianceDiagnostics.eigenvaluesAfter = eigenvaluesAfter;
    covarianceDiagnostics.lowerLimitApplied = any(eigenvaluesAfter > eigenvaluesBefore);
    covarianceDiagnostics.upperLimitApplied = any(eigenvaluesAfter < eigenvaluesBefore);
    covarianceDiagnostics.limitApplied = covarianceDiagnostics.lowerLimitApplied || covarianceDiagnostics.upperLimitApplied;
end

function components = applyPosthocSupportFields(points, sourceIndices, timestampBins, supportDiagnostics, components, params, emConverged)
% applyPosthocSupportFields: Store post-hoc support amplitude,
% temporal-diversity, sample-sufficiency, geometric-stability, and assignment
% diagnostics on ordinary GMM components.
    assignmentEvidence = supportDiagnostics.assignmentEvidence;
    if ~isequal(size(assignmentEvidence), [size(points, 1), numel(components)])
        error("buildTemporalStabilityGmmMap:InvalidPosthocSupportEvidence", ...
            "Post-hoc assignment evidence must match original point and component counts.");
    end
    for componentIdx = 1:numel(components)
        components(componentIdx) = applyOnePosthocSupportComponent(points, sourceIndices, timestampBins, assignmentEvidence(:, componentIdx), components(componentIdx), params, emConverged);
    end
end

function component = applyOnePosthocSupportComponent(points, sourceIndices, timestampBins, supportWeights, component, params, emConverged)
% applyOnePosthocSupportComponent: Compute and store temporal support
% diagnostics for one trained ordinary Gaussian component without modifying its
% mixture weight, mean, covariance, or inverse covariance.
    if isempty(supportWeights) || any(~isfinite(supportWeights)) || any(supportWeights < 0)
        error("buildTemporalStabilityGmmMap:InvalidPosthocAssignmentEvidence", ...
            "Post-hoc support assignment evidence must be finite and nonnegative.");
    end
    effectiveSupportPointCount = sum(supportWeights);
    if effectiveSupportPointCount >= params.emMinEffectiveSupport
        assignedPointIdx = integratedSupportEvidencePointIndices(supportWeights, effectiveSupportPointCount, params);
        assignedSupportPointCount = numel(assignedPointIdx);
        supportBoundingBox = computeSupportBoundingBox(points(assignedPointIdx, :), component.mean, component.covariance);
        frameSupport = integratedFrameSupport(timestampBins, supportWeights, params);
        [geometricStability, geometricDispersion, geometricMode, geometricLength] = shapeDependentGeometricStability(points, timestampBins, supportWeights, component, params);
        sampleSufficiency = computeSampleSufficiency(frameSupport.totalSaturatedFrameSupport, params);
        temporalDiversity = frameSupport.temporalDiversity;
        timestampBinCount = frameSupport.activeFrameBinCount;
        effectiveFrameCount = frameSupport.effectiveFrameSupport;
        supportAmplitude = mappingSupport.clipUnit(sampleSufficiency .* temporalDiversity .* geometricStability);
        reliability = temporalDiversity .* geometricStability;
    else
        assignedPointIdx = zeros(0, 1);
        assignedSupportPointCount = 0;
        supportBoundingBox = [component.mean, component.mean];
        frameSupport = emptyFrameSupport();
        geometricStability = 0;
        geometricDispersion = 0;
        geometricMode = "insufficientPosthocSupport";
        geometricLength = 0;
        sampleSufficiency = 0;
        temporalDiversity = 0;
        timestampBinCount = 0;
        effectiveFrameCount = 0;
        supportAmplitude = 0;
        reliability = 0;
    end

    component.integratedSupportEffectivePointCount = effectiveSupportPointCount;
    component.integratedSupportAssignedPointCount = assignedSupportPointCount;
    component.effectiveSupportPointCount = effectiveSupportPointCount;
    component.assignedSupportPointCount = assignedSupportPointCount;
    component.pointCount = effectiveSupportPointCount;
    component.supportBoundingBox = supportBoundingBox;
    component.boundingBox = supportBoundingBox;
    component.timestampBinCount = timestampBinCount;
    component.temporalDiversity = temporalDiversity;
    component.normalStability = geometricStability;
    component.normalDispersion = geometricDispersion;
    component.supportAmplitude = supportAmplitude;
    component.reliability = reliability;
    component.sampleSufficiency = sampleSufficiency;
    component.effectiveFrameCount = effectiveFrameCount;
    component.frameCounts = frameSupport.rawFrameSupportEvidence(:);
    component.rawFrameSupportEvidence = frameSupport.rawFrameSupportEvidence(:);
    component.saturatedFrameSupport = frameSupport.saturatedFrameSupport(:);
    component.totalRawFrameSupportEvidence = frameSupport.totalRawFrameSupportEvidence;
    component.totalSaturatedFrameSupport = frameSupport.totalSaturatedFrameSupport;
    component.activeFrameBinCount = frameSupport.activeFrameBinCount;
    component.effectiveFrameSupport = frameSupport.effectiveFrameSupport;
    component.temporalDiversityModel = "posthocReliabilityWeightedGmmSaturatedFrameSupportParticipationEvenness";
    component.sampleSufficiencyModel = "posthocReliabilityWeightedGmmTotalSaturatedFrameSupport";
    component.geometricStabilityMode = geometricMode;
    component.geometricStability = geometricStability;
    component.geometricDispersion = geometricDispersion;
    component.geometricStabilityLength = geometricLength;
    component.integratedSupportEstimated = true;
    component.integratedSupportEstimatedAfterEmConvergence = logical(emConverged);
    component.integratedSupportAssignmentModel = "ordinaryGmmResponsibilitiesTimesPrecomputedTemporalReliability";
    component.integratedSupportUsesStudentTInlierWeights = false;
    component.integratedSupportWeightedEffectivePointCount = effectiveSupportPointCount;
    component.integratedSupportAssignmentWeightMin = min(supportWeights);
    component.integratedSupportAssignmentWeightMean = mean(supportWeights);
    component.integratedSupportAssignmentWeightMax = max(supportWeights);
    component.posthocSupportEvidenceMass = effectiveSupportPointCount;
    component.posthocSupportAssignmentModel = "ordinaryGmmResponsibilitiesTimesPrecomputedTemporalReliability";

    if params.storeEmAssignments && ~isempty(assignedPointIdx)
        component.softAssignmentSourceIndices = sourceIndices(assignedPointIdx);
        component.softAssignmentWeights = supportWeights(assignedPointIdx);
    else
        component.softAssignmentSourceIndices = zeros(0, 1);
        component.softAssignmentWeights = zeros(0, 1);
    end
end

function frameSupport = emptyFrameSupport()
% emptyFrameSupport: Create an empty frame-support diagnostic struct for
% components with insufficient post-hoc temporal evidence.
    frameSupport = struct();
    frameSupport.rawFrameSupportEvidence = zeros(0, 1);
    frameSupport.saturatedFrameSupport = zeros(0, 1);
    frameSupport.totalRawFrameSupportEvidence = 0;
    frameSupport.totalSaturatedFrameSupport = 0;
    frameSupport.activeFrameBinCount = 0;
    frameSupport.effectiveFrameSupport = 0;
    frameSupport.temporalDiversity = 0;
end

function assignedPointIdx = integratedSupportEvidencePointIndices(assignedWeights, effectiveSupportPointCount, params)
% integratedSupportEvidencePointIndices: Select points with numerically
% positive post-hoc foreground support evidence for support bounding boxes and
% stored soft-assignment diagnostics. The selection avoids a fixed per-point
% threshold because a component can be valid through many low-weight ordinary
% GMM assignments.
    if isempty(assignedWeights) || any(~isfinite(assignedWeights)) || any(assignedWeights < 0)
        error("buildTemporalStabilityGmmMap:InvalidIntegratedAssignmentEvidence", ...
            "Support point selection requires finite nonnegative foreground assignment evidence.");
    end
    if ~isscalar(effectiveSupportPointCount) || ~isfinite(effectiveSupportPointCount) || effectiveSupportPointCount < params.emMinEffectiveSupport
        error("buildTemporalStabilityGmmMap:InsufficientIntegratedEffectiveSupport", ...
            "Integrated component support evidence mass must be finite and at least emMinEffectiveSupport.");
    end
    positiveThreshold = max(realmin, eps(max(assignedWeights)));
    assignedPointIdx = find(assignedWeights > positiveThreshold);
    if isempty(assignedPointIdx)
        error("buildTemporalStabilityGmmMap:NoAssignedSupportPoints", ...
            "Support bounding boxes require at least one point with positive foreground assignment evidence.");
    end
end

function supportBoundingBox = computeSupportBoundingBox(assignedPoints, ~, covariance)
% computeSupportBoundingBox: Compute a refined support extent for a component
% from final assigned points. Missing assigned support points are invalid
% because post-hoc support amplitudes require valid assignment evidence.
    mappingSupport.validateCovarianceMatrix(covariance, "Support bounding-box computation");
    if isempty(assignedPoints)
        error("buildTemporalStabilityGmmMap:NoAssignedSupportPoints", ...
            "Support bounding boxes require at least one assigned point.");
    end
    minPoint = min(assignedPoints, [], 1);
    maxPoint = max(assignedPoints, [], 1);
    supportBoundingBox = [minPoint, maxPoint];
end

function frameSupport = integratedFrameSupport(timestampBins, weights, params)
% integratedFrameSupport: Aggregate post-hoc foreground assignment
% evidence into frame bins, saturate each frame's raw evidence, and compute
% repeated-frame sample and concentration diagnostics. Sample sufficiency uses
% total saturated support, while temporal diversity uses the participation
% ratio of saturated support across frames and saturates once the configured
% timestampMaxBins support horizon is reached.
    if isempty(weights) || any(~isfinite(weights)) || any(weights < 0) || sum(weights) < params.emMinEffectiveSupport
        error("buildTemporalStabilityGmmMap:InvalidFrameSupportWeights", ...
            "Integrated frame support requires finite nonnegative foreground evidence with sufficient effective mass.");
    end

    [~, ~, groupIdx] = unique(timestampBins(:));
    rawFrameSupportEvidence = accumarray(groupIdx, weights(:), [], @sum, 0);
    saturatedFrameSupport = saturateFrameSupport(rawFrameSupportEvidence, params);
    activeMask = saturatedFrameSupport >= params.emMinFrameWeight;
    if ~any(activeMask)
        error("buildTemporalStabilityGmmMap:NoActiveTimestampBins", ...
            "Integrated frame support requires at least one active timestamp bin.");
    end
    activeRawFrameSupportEvidence = rawFrameSupportEvidence(activeMask);
    activeSaturatedFrameSupport = saturatedFrameSupport(activeMask);
    totalRawFrameSupportEvidence = sum(activeRawFrameSupportEvidence(:));
    totalSaturatedFrameSupport = sum(activeSaturatedFrameSupport(:));
    if ~isfinite(totalRawFrameSupportEvidence) || ~isfinite(totalSaturatedFrameSupport) || totalSaturatedFrameSupport <= 0
        error("buildTemporalStabilityGmmMap:InvalidFrameSupport", ...
            "Integrated frame support totals must be finite positive values.");
    end
    effectiveFrameSupport = double(totalSaturatedFrameSupport)^2 / double(sum(activeSaturatedFrameSupport(:).^2));
    if ~isscalar(effectiveFrameSupport) || ~isfinite(effectiveFrameSupport) || effectiveFrameSupport < 1 - 1.0e-10
        error("buildTemporalStabilityGmmMap:InvalidEffectiveFrameSupport", ...
            "Integrated effective frame support must be finite and at least one for active frame support.");
    end
    if sum(activeMask) < 2
        temporalDiversity = 0;
    else
        temporalDiversity = min(1, (effectiveFrameSupport - 1) ./ max(1, params.timestampMaxBins - 1));
    end
    temporalDiversity = mappingSupport.clipUnit(temporalDiversity);

    frameSupport = struct();
    frameSupport.rawFrameSupportEvidence = activeRawFrameSupportEvidence(:);
    frameSupport.saturatedFrameSupport = activeSaturatedFrameSupport(:);
    frameSupport.totalRawFrameSupportEvidence = totalRawFrameSupportEvidence;
    frameSupport.totalSaturatedFrameSupport = totalSaturatedFrameSupport;
    frameSupport.activeFrameBinCount = sum(activeMask);
    frameSupport.effectiveFrameSupport = effectiveFrameSupport;
    frameSupport.temporalDiversity = temporalDiversity;
end

function [geometricStability, geometricDispersion, geometricMode, geometricLength] = shapeDependentGeometricStability(points, timestampBins, weights, component, params)
% shapeDependentGeometricStability: Select elongated normal-position
% stability or compact 2-D centroid-position stability from the component
% covariance anisotropy, then compute a bounded cross-frame geometric
% consistency multiplier.
    anisotropy = component.covarianceAnisotropy;
    if ~isfinite(anisotropy) || anisotropy < 1
        error("buildTemporalStabilityGmmMap:InvalidCovarianceAnisotropy", ...
            "Shape-dependent geometric stability requires finite covariance anisotropy greater than or equal to one.");
    end
    if anisotropy >= params.geometricElongatedAnisotropyThreshold
        [geometricStability, geometricDispersion] = normalPositionStability(points, timestampBins, weights, component.mean, component.n, params);
        geometricMode = "normalPosition";
        geometricLength = params.normalStabilityLength;
    else
        [geometricStability, geometricDispersion] = centroidPositionStability(points, timestampBins, weights, component.mean, params);
        geometricMode = "centroidPosition2D";
        geometricLength = params.compactStabilityLength;
    end
    geometricStability = mappingSupport.clipUnit(geometricStability);
    if ~isfinite(geometricDispersion) || geometricDispersion < 0 || ~isfinite(geometricLength) || geometricLength <= 0
        error("buildTemporalStabilityGmmMap:InvalidGeometricStability", ...
            "Integrated geometric stability diagnostics must be finite and nonnegative with a positive stability length.");
    end
end

function [normalStability, normalDispersion] = normalPositionStability(points, timestampBins, weights, centroid, n, params)
% normalPositionStability: Estimate elongated-component cross-frame
% normal-position stability from weighted per-frame median normal coordinates.
% Fewer than two active frames carry no geometric disagreement, so temporal
% diversity and pruning handle the lack of repeated support.
    [frameWeights, activeGroupIdx, groupIdx] = activeFrameWeights(timestampBins, weights, params);
    if numel(activeGroupIdx) < 2
        normalStability = 1;
        normalDispersion = 0;
        return;
    end
    normalCoordinates = (points - centroid) * n;
    binNormalCoordinates = zeros(numel(activeGroupIdx), 1);
    for activeIdx = 1:numel(activeGroupIdx)
        pointMask = groupIdx == activeGroupIdx(activeIdx);
        binNormalCoordinates(activeIdx) = weightedMedian(normalCoordinates(pointMask), weights(pointMask));
    end
    binWeights = frameWeights(activeGroupIdx);
    medianNormalCoordinate = weightedMedian(binNormalCoordinates, binWeights);
    normalDispersion = 1.4826 .* weightedMedian(abs(binNormalCoordinates - medianNormalCoordinate), binWeights);
    normalStability = exp(-0.5 .* (normalDispersion ./ params.normalStabilityLength).^2);
end

function [positionStability, positionDispersion] = centroidPositionStability(points, timestampBins, weights, centroid, params)
% centroidPositionStability: Estimate compact-component cross-frame
% positional stability from weighted per-frame 2-D centroids instead of a PCA
% normal coordinate that is unstable for nearly isotropic components.
    [frameWeights, activeGroupIdx, groupIdx] = activeFrameWeights(timestampBins, weights, params);
    if numel(activeGroupIdx) < 2
        positionStability = 1;
        positionDispersion = 0;
        return;
    end
    binCentroids = repmat(centroid, numel(activeGroupIdx), 1);
    for activeIdx = 1:numel(activeGroupIdx)
        pointMask = groupIdx == activeGroupIdx(activeIdx);
        pointWeights = weights(pointMask);
        binCentroids(activeIdx, :) = sum(points(pointMask, :) .* pointWeights, 1) ./ sum(pointWeights);
    end
    binWeights = frameWeights(activeGroupIdx);
    weightedCenter = sum(binCentroids .* binWeights, 1) ./ sum(binWeights);
    centeredBinCentroids = binCentroids - weightedCenter;
    positionDispersion = sqrt(sum(binWeights .* sum(centeredBinCentroids.^2, 2)) ./ sum(binWeights));
    positionStability = exp(-0.5 .* (positionDispersion ./ params.compactStabilityLength).^2);
end

function [frameWeights, activeGroupIdx, groupIdx] = activeFrameWeights(timestampBins, weights, params)
% activeFrameWeights: Validate integrated foreground weights and return raw
% frame evidence totals plus active frame indices for geometric statistics.
    if isempty(weights) || any(~isfinite(weights)) || any(weights < 0) || sum(weights) < params.emMinEffectiveSupport
        error("buildTemporalStabilityGmmMap:InvalidGeometricStabilityWeights", ...
            "Integrated geometric stability requires finite nonnegative foreground evidence with sufficient effective mass.");
    end
    [uniqueBins, ~, groupIdx] = unique(timestampBins(:));
    frameWeights = accumarray(groupIdx, weights(:), [numel(uniqueBins), 1], @sum, 0);
    activeGroupIdx = find(saturateFrameSupport(frameWeights, params) >= params.emMinFrameWeight);
    if isempty(activeGroupIdx)
        error("buildTemporalStabilityGmmMap:NoActiveTimestampBins", ...
            "Integrated geometric stability requires at least one active timestamp bin.");
    end
end

function medianValue = weightedMedian(values, weights)
% weightedMedian: Compute a deterministic weighted median for robust
% responsibility-weighted temporal and normal-position statistics.
    values = values(:);
    weights = weights(:);
    validMask = weights > 0 & isfinite(weights) & isfinite(values);
    values = values(validMask);
    weights = weights(validMask);

    if isempty(values)
        error("buildTemporalStabilityGmmMap:InvalidWeightedMedianInput", ...
            "Weighted median requires at least one finite value with positive finite weight.");
    end

    [values, orderIdx] = sort(values, "ascend");
    weights = weights(orderIdx);
    cumulativeWeights = cumsum(weights);
    medianIdx = find(cumulativeWeights >= 0.5 .* cumulativeWeights(end), 1, "first");
    medianValue = values(medianIdx);
end

function [patchLocalIndices, patchComponents, removedPatchCount] = buildSpatialSupportPatches(classPoints, sourceIndices, timestampBins, params, pcaEpsilon)
% buildSpatialSupportPatches: Build class-local spatial support candidates
% outside EM. Patches are connected components of a radius-pruned mutual
% kNN graph, recursively split along their dominant PCA axis until each
% patch is compact, filtered to a buildable point count, and converted into
% PCA-oriented initialization components with patch-level temporal
% diagnostics. The patches influence reliability scoring, sampling,
% model order, and EM initialization, never the EM updates themselves.
    patchLocalIndices = buildMutualKnnPatches(classPoints, params, pcaEpsilon);
    [patchLocalIndices, removedPatchCount] = filterBuildablePatches(patchLocalIndices, max(2, round(double(params.minComponentPoints))));
    patchComponents = repmat(emptyComponent(), numel(patchLocalIndices), 1);
    for patchIdx = 1:numel(patchLocalIndices)
        localIndices = patchLocalIndices{patchIdx};
        patchComponents(patchIdx) = buildComponent(classPoints(localIndices, :), sourceIndices(localIndices), timestampBins(localIndices), params, pcaEpsilon);
    end
end

function patchLocalIndices = buildMutualKnnPatches(points, params, pcaEpsilon)
% buildMutualKnnPatches: Build initial class-local patches as connected
% components of the mutual-kNN graph with radius pruning, then recursively
% split each component along its dominant PCA axis until every final patch
% satisfies the configured Euclidean diameter bound.
    pointCount = size(points, 1);

    if pointCount == 0
        patchLocalIndices = cell(0, 1);
        return;
    end

    if pointCount == 1
        patchLocalIndices = {1};
        return;
    end

    kEff = min(params.k, pointCount - 1);

    if kEff == 0
        componentIds = (1:pointCount).';
        componentCount = pointCount;
    else
        [neighborIdx, neighborDistance] = knnSearchExcludingSelf(points, kEff);
        sourceIdx = repmat((1:pointCount).', 1, kEff);
        validNeighborMask = neighborIdx > 0 & neighborDistance <= params.radius;
        directedAdjacency = sparse(sourceIdx(validNeighborMask), neighborIdx(validNeighborMask), true, pointCount, pointCount);
        adjacency = directedAdjacency & directedAdjacency.';
        graphObj = graph(adjacency, "upper");
        componentIds = conncomp(graphObj).';
        componentCount = max(componentIds);
    end

    patchLocalIndices = cell(pointCount, 1);
    patchCount = 0;
    for componentIdx = 1:componentCount
        componentLocalIndices = find(componentIds == componentIdx);
        splitPatches = splitPatchByDiameter(points, componentLocalIndices, params.maxDiameter, pcaEpsilon);
        nextPatchIdx = patchCount + (1:numel(splitPatches));
        patchLocalIndices(nextPatchIdx, 1) = splitPatches(:);
        patchCount = patchCount + numel(splitPatches);
    end
    patchLocalIndices = patchLocalIndices(1:patchCount);
end

function [neighborIdx, neighborDistance] = knnSearchExcludingSelf(points, kEff)
% knnSearchExcludingSelf: Query a KD-tree for each point's nearest
% spatial neighbors while removing the point itself from its neighbor list.
    pointCount = size(points, 1);
    assert(exist("KDTreeSearcher", "class") == 8, ...
        "KDTreeSearcher is required for optimized mutual-kNN patch construction.");
    searcher = KDTreeSearcher(points);
    [rawIdx, rawDistance] = knnsearch(searcher, points, "K", min(pointCount, kEff + 1));
    neighborIdx = zeros(pointCount, kEff);
    neighborDistance = inf(pointCount, kEff);

    for pointIdx = 1:pointCount
        rowIdx = rawIdx(pointIdx, :);
        rowDistance = rawDistance(pointIdx, :);
        nonSelfMask = rowIdx ~= pointIdx;
        rowIdx = rowIdx(nonSelfMask);
        rowDistance = rowDistance(nonSelfMask);
        keepCount = min(kEff, numel(rowIdx));
        if keepCount > 0
            neighborIdx(pointIdx, 1:keepCount) = rowIdx(1:keepCount);
            neighborDistance(pointIdx, 1:keepCount) = rowDistance(1:keepCount);
        end
    end
end

function splitPatches = splitPatchByDiameter(points, localIndices, maxDiameter, pcaEpsilon)
% splitPatchByDiameter: Recursively split a candidate patch along the
% patch dominant PCA axis at the median projection until each output patch
% has diameter less than or equal to the configured maximum.
    maxQueueLength = max(1, 2 .* numel(localIndices) - 1);
    queue = cell(maxQueueLength, 1);
    queue{1} = localIndices(:);
    queueCount = 1;
    splitPatches = cell(numel(localIndices), 1);
    patchCount = 0;
    headIdx = 1;

    while headIdx <= queueCount
        currentIndices = queue{headIdx};
        headIdx = headIdx + 1;

        if numel(currentIndices) <= 1 || patchDiameter(points(currentIndices, :)) <= maxDiameter
            patchCount = patchCount + 1;
            splitPatches{patchCount, 1} = currentIndices;
        else
            [t, ~] = pcaDirections(points(currentIndices, :), pcaEpsilon);
            alpha = points(currentIndices, :) * t;
            alphaMedian = median(alpha);
            minusIndices = currentIndices(alpha <= alphaMedian);
            plusIndices = currentIndices(alpha > alphaMedian);

            if isempty(minusIndices) || isempty(plusIndices)
                [~, orderIdx] = sort(alpha, "ascend");
                splitAt = floor(numel(currentIndices) / 2);
                minusIndices = currentIndices(orderIdx(1:splitAt));
                plusIndices = currentIndices(orderIdx(splitAt + 1:end));
            end

            queueCount = queueCount + 1;
            queue{queueCount, 1} = minusIndices(:);
            queueCount = queueCount + 1;
            queue{queueCount, 1} = plusIndices(:);
        end
    end

    splitPatches = splitPatches(1:patchCount);
end

function diameter = patchDiameter(points)
% patchDiameter: Compute the bounding-box diagonal upper bound for a
% candidate patch diameter, returning zero for singleton or empty patches.
    if size(points, 1) <= 1
        diameter = 0;
    else
        span = max(points, [], 1) - min(points, [], 1);
        diameter = hypot(span(1), span(2));
    end
end

function [patchLocalIndices, removedPatchCount] = filterBuildablePatches(patchLocalIndices, minPatchPointCount)
% filterBuildablePatches: Remove initialization patches whose point
% count is too small to construct the PCA-oriented covariance used by
% EM GMM component initialization.
    if isempty(patchLocalIndices)
        removedPatchCount = 0;
        return;
    end
    patchSizes = cellfun(@numel, patchLocalIndices(:));
    keepMask = patchSizes >= minPatchPointCount;
    removedPatchCount = nnz(~keepMask);
    patchLocalIndices = patchLocalIndices(keepMask);
end

function component = buildComponent(points, sourceIndices, timestampBins, params, pcaEpsilon)
% buildComponent: Convert one final semantic-geometric patch into an EM
% mixture initialization component with PCA-oriented covariance geometry, patch
% provenance, and patch-level temporal diagnostics. The initial temporal
% diagnostics are retained only for debugging and do not affect the integrated
% likelihood, responsibilities, M-step, covariance update, mixture-weight
% update, or convergence logic.
    [t, n] = pcaDirections(points, pcaEpsilon);
    centroid = mean(points, 1);
    minPoint = min(points, [], 1);
    maxPoint = max(points, [], 1);
    boundingBox = [minPoint, maxPoint];
    covariance = params.lengthParallel.^2 .* (t * t.') + params.lengthPerp.^2 .* (n * n.') + pcaEpsilon .* eye(2);
    [covariance, covarianceDiagnostics] = applyCovarianceEigenvalueSafeguards(covariance, t, n, params);
    invCovariance = invertCovariance(covariance, pcaEpsilon);
    [temporalDiversity, timestampBinCount, effectiveFrameCount, frameCounts] = patchTemporalDiversity(timestampBins, params);
    [normalStability, normalDispersion] = patchNormalStability(points, timestampBins, centroid, n, params);
    reliability = temporalDiversity .* normalStability;
    sampleSufficiency = computeSampleSufficiency(sum(saturateFrameSupport(frameCounts(:), params)), params);
    supportAmplitude = mappingSupport.clipUnit(reliability .* sampleSufficiency);

    component = emptyComponent();
    component.points = points;
    component.sourceIndices = sourceIndices(:);
    component.timestampBins = timestampBins(:);
    component.patchPoints = points;
    component.patchSourceIndices = sourceIndices(:);
    component.patchTimestampBins = timestampBins(:);
    component.patchBoundingBox = boundingBox;
    component.patchPointCount = size(points, 1);
    component.timestampBinCount = 0;
    component.temporalDiversity = 0;
    component.normalStability = 1;
    component.normalDispersion = 0;
    component.supportAmplitude = 0;
    component.reliability = 0;
    component.sampleSufficiency = 0;
    component.initialTimestampBinCount = timestampBinCount;
    component.initialTemporalDiversity = temporalDiversity;
    component.initialNormalStability = normalStability;
    component.initialNormalDispersion = normalDispersion;
    component.initialSupportAmplitude = supportAmplitude;
    component.initialReliability = reliability;
    component.initialSampleSufficiency = sampleSufficiency;
    component.initialCentroid = centroid;
    component.centroid = centroid;
    component.mean = centroid;
    component.mixtureWeight = 0;
    component.initialMixtureWeight = 0;
    component.emMixtureWeightBeforePruning = 0;
    component.emResponsibilityPointCount = 0;
    component.emRobustWeightedPointCount = 0;
    component.emRobustScaleWeightMin = nan;
    component.emRobustScaleWeightMean = nan;
    component.emRobustScaleWeightMax = nan;
    component.emForegroundResponsibilityPointCount = 0;
    component.emNumericalFreezeApplied = false;
    component.emNumericalEffectiveSupportThreshold = params.emMinEffectiveSupport;
    component.covariance = covariance;
    component.invCovariance = invCovariance;
    component = storeCovarianceDiagnostics(component, covarianceDiagnostics, params);
    component.t = t;
    component.n = n;
    component.boundingBox = zeros(1, 4);
    component.pointCount = 0;
    component.supportBoundingBox = zeros(1, 4);
    component.effectiveSupportPointCount = 0;
    component.assignedSupportPointCount = 0;
    component.effectiveFrameCount = effectiveFrameCount;
    component.frameCounts = frameCounts(:);
    component.patchDiagnosticTimestampBinCount = timestampBinCount;
    component.patchDiagnosticTemporalDiversity = temporalDiversity;
    component.patchDiagnosticNormalStability = normalStability;
    component.patchDiagnosticNormalDispersion = normalDispersion;
    component.patchDiagnosticReliability = reliability;
    component.patchDiagnosticSampleSufficiency = sampleSufficiency;
    component.patchDiagnosticSupportAmplitude = supportAmplitude;
end

function [temporalDiversity, timestampBinCount, effectiveFrameCount, frameCounts] = patchTemporalDiversity(timestampBins, params)
% patchTemporalDiversity: Compute patch-level cross-frame temporal
% diversity from distinct frame-bin coverage. When frameCountSaturation is
% configured, the score uses a saturated effective-frame count; otherwise it
% uses the distinct frame-bin count.
    [~, ~, groupIdx] = unique(timestampBins(:));
    timestampBinCount = max(groupIdx);
    frameCounts = accumarray(groupIdx, 1, [timestampBinCount, 1]);

    if isempty(params.frameCountSaturation)
        effectiveFrameCount = timestampBinCount;
    else
        saturatedCounts = 1 - exp(-frameCounts ./ params.frameCountSaturation);
        effectiveFrameCount = sum(saturatedCounts).^2 ./ sum(saturatedCounts.^2);
    end
    temporalDiversity = min(1, log(1 + effectiveFrameCount) ./ log(1 + params.timestampMaxBins));
end

function [normalStability, normalDispersion] = patchNormalStability(points, timestampBins, centroid, n, params)
% patchNormalStability: Estimate cross-frame patch stability by
% projecting patch points onto the local PCA normal direction, reducing each
% frame bin to a median normal coordinate, and converting robust MAD
% dispersion across bins into a bounded support multiplier.
    [~, ~, groupIdx] = unique(timestampBins(:));
    timestampBinCount = max(groupIdx);
    if timestampBinCount < 2
        normalStability = 1.0;
        normalDispersion = 0.0;
        return;
    end

    normalCoordinates = (points - centroid) * n;
    binNormalCoordinates = accumarray(groupIdx, normalCoordinates, [], @median);
    medianNormalCoordinate = median(binNormalCoordinates);
    normalDispersion = 1.4826 .* median(abs(binNormalCoordinates - medianNormalCoordinate));
    normalStability = exp(-0.5 .* (normalDispersion ./ params.normalStabilityLength).^2);
end

function [t, n] = pcaDirections(points, pcaEpsilon)
% pcaDirections: Estimate patch-level dominant and minor PCA directions
% from the regularized two-dimensional covariance matrix. Degenerate patches
% without a well-defined dominant direction are invalid under fail-fast
% construction.
    if size(points, 1) <= 1
        error("buildTemporalStabilityGmmMap:DegeneratePatchPca", ...
            "Patch PCA requires at least two points with a well-defined dominant direction.");
    end

    centeredPoints = points - mean(points, 1);
    covarianceMatrix = (centeredPoints.' * centeredPoints) ./ size(points, 1) + pcaEpsilon .* eye(2);
    [t, n] = covarianceDirections(covarianceMatrix);
end

function supportDiagnostics = computePosthocTemporalSupport(points, sourceIndices, timestampBins, stability, components, params)
% computePosthocTemporalSupport: Evaluate the trained ordinary GMM on the
% original class points, multiply responsibilities by precomputed temporal
% reliability scores, and package post-hoc temporal support evidence without
% changing any GMM parameters.
    if numel(sourceIndices) ~= size(points, 1) || numel(timestampBins) ~= size(points, 1) || numel(stability.scores) ~= size(points, 1)
        error("buildTemporalStabilityGmmMap:InvalidPosthocSupportInput", ...
            "Post-hoc temporal support requires one source index, timestamp bin, and reliability score per original point.");
    end
    [responsibilities, logLikelihood] = gaussianExpectation(points, components);
    assignmentEvidence = responsibilities .* stability.scores(:);
    supportDiagnostics = struct();
    supportDiagnostics.completed = true;
    supportDiagnostics.componentCount = numel(components);
    supportDiagnostics.emConvergedAtSupportEstimation = false;
    supportDiagnostics.assignmentModel = "ordinaryGmmResponsibilitiesTimesPrecomputedTemporalReliability";
    supportDiagnostics.usesStudentTInlierWeights = false;
    supportDiagnostics.frameSupportModel = "posthocReliabilityWeightedGmmEvidenceWithPerFrameSaturation";
    supportDiagnostics.temporalDiversityModel = "posthocSaturatedFrameSupportParticipationEvenness";
    supportDiagnostics.sampleSufficiencyModel = "posthocTotalSaturatedFrameSupport";
    supportDiagnostics.geometricConsistencyModel = "posthocShapeDependentNormalOrCentroidStability";
    supportDiagnostics.responsibilities = responsibilities;
    supportDiagnostics.assignmentEvidence = assignmentEvidence;
    supportDiagnostics.originalPointLogLikelihood = logLikelihood;
    supportDiagnostics.numericalSafeguardMinEffectiveSupport = params.emMinEffectiveSupport;
    supportDiagnostics.robustAssignmentWeightMin = min(assignmentEvidence(:));
    supportDiagnostics.robustAssignmentWeightMean = mean(assignmentEvidence(:));
    supportDiagnostics.robustAssignmentWeightMax = max(assignmentEvidence(:));
    supportDiagnostics.robustWeightedComponentMass = sum(assignmentEvidence, 1).';
    supportDiagnostics.robustAssignmentRowSums = sum(assignmentEvidence, 2);
    if any(~isfinite(assignmentEvidence), "all") || any(assignmentEvidence < 0, "all")
        error("buildTemporalStabilityGmmMap:InvalidPosthocSupportEvidence", ...
            "Post-hoc temporal support evidence must be finite and nonnegative.");
    end
end

function sampleSufficiency = computeSampleSufficiency(totalSaturatedFrameSupport, params)
% computeSampleSufficiency: Compute the repeated-frame sample sufficiency
% multiplier from total saturated foreground frame support using the configured
% prior scale. This term measures enough repeated support evidence and is
% distinct from concentration-aware temporal diversity.
    if ~isscalar(totalSaturatedFrameSupport) || ~isfinite(totalSaturatedFrameSupport) || totalSaturatedFrameSupport < 0
        error("buildTemporalStabilityGmmMap:InvalidSampleSufficiencyInput", ...
            "Sample sufficiency requires finite nonnegative total saturated frame support.");
    end
    sampleSufficiency = mappingSupport.clipUnit(1 - exp(-totalSaturatedFrameSupport ./ params.effectiveFramePrior));
end

function [t, n] = covarianceDirections(covarianceMatrix)
% covarianceDirections: Extract deterministic orthonormal directions
% from a two-dimensional covariance matrix. Nearly isotropic covariances use
% the canonical BEV basis because compact Gaussian components do not require a
% unique dominant direction.
    mappingSupport.validateCovarianceMatrix(covarianceMatrix, "Covariance direction extraction");
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

function component = emptyComponent()
% emptyComponent: Create an empty structured Gaussian support component
% with patch diagnostics, EM mixture parameters, covariance geometry, and
% post-hoc temporal support amplitude fields.
    component = struct();
    component.points = zeros(0, 2);
    component.sourceIndices = zeros(0, 1);
    component.timestampBins = strings(0, 1);
    component.patchPoints = zeros(0, 2);
    component.patchSourceIndices = zeros(0, 1);
    component.patchTimestampBins = strings(0, 1);
    component.patchBoundingBox = zeros(1, 4);
    component.patchPointCount = 0;
    component.timestampBinCount = 0;
    component.temporalDiversity = 0;
    component.normalStability = 1;
    component.normalDispersion = 0;
    component.supportAmplitude = 0;
    component.reliability = 0;
    component.sampleSufficiency = 1;
    component.initialTimestampBinCount = 0;
    component.initialTemporalDiversity = 0;
    component.initialNormalStability = 1;
    component.initialNormalDispersion = 0;
    component.initialSupportAmplitude = 0;
    component.initialReliability = 0;
    component.initialSampleSufficiency = 0;
    component.initialCentroid = zeros(1, 2);
    component.centroid = zeros(1, 2);
    component.mean = zeros(1, 2);
    component.mixtureWeight = 0;
    component.initialMixtureWeight = 0;
    component.emMixtureWeightBeforePruning = 0;
    component.emResponsibilityPointCount = 0;
    component.emRobustWeightedPointCount = 0;
    component.emRobustScaleWeightMin = nan;
    component.emRobustScaleWeightMean = nan;
    component.emRobustScaleWeightMax = nan;
    component.emForegroundResponsibilityPointCount = 0;
    component.emNumericalFreezeApplied = false;
    component.emNumericalEffectiveSupportThreshold = 0;
    component.covariance = eye(2);
    component.invCovariance = eye(2);
    component.covarianceEigenvaluesBeforeSafeguard = ones(2, 1);
    component.covarianceEigenvaluesAfterSafeguard = ones(2, 1);
    component.covarianceEigenvalueLowerBound = 0;
    component.covarianceEigenvalueUpperBound = inf;
    component.covarianceEigenvalueLimitApplied = false;
    component.covarianceEigenvalueLowerLimitApplied = false;
    component.covarianceEigenvalueUpperLimitApplied = false;
    component.covarianceAnisotropy = 1;
    component.querySupportRadius = 0;
    component.t = [1; 0];
    component.n = [0; 1];
    component.boundingBox = zeros(1, 4);
    component.pointCount = 0;
    component.supportBoundingBox = zeros(1, 4);
    component.effectiveSupportPointCount = 0;
    component.assignedSupportPointCount = 0;
    component.effectiveFrameCount = 0;
    component.frameCounts = zeros(0, 1);
    component.rawFrameSupportEvidence = zeros(0, 1);
    component.saturatedFrameSupport = zeros(0, 1);
    component.totalRawFrameSupportEvidence = 0;
    component.totalSaturatedFrameSupport = 0;
    component.activeFrameBinCount = 0;
    component.effectiveFrameSupport = 0;
    component.temporalDiversityModel = "";
    component.sampleSufficiencyModel = "";
    component.geometricStabilityMode = "";
    component.geometricStability = 1;
    component.geometricDispersion = 0;
    component.geometricStabilityLength = 0;
    component.emIterationCount = 0;
    component.emConverged = false;
    component.emMaxMeanShift = 0;
    component.integratedSupportEffectivePointCount = 0;
    component.integratedSupportAssignedPointCount = 0;
    component.softAssignmentSourceIndices = zeros(0, 1);
    component.softAssignmentWeights = zeros(0, 1);
    component.integratedSupportEstimated = false;
    component.integratedSupportEstimatedAfterEmConvergence = false;
    component.integratedSupportAssignmentModel = "";
    component.integratedSupportUsesStudentTInlierWeights = false;
    component.integratedSupportWeightedEffectivePointCount = 0;
    component.integratedSupportAssignmentWeightMin = nan;
    component.integratedSupportAssignmentWeightMean = nan;
    component.integratedSupportAssignmentWeightMax = nan;
    component.posthocSupportEvidenceMass = 0;
    component.posthocSupportAssignmentModel = "";
    component.patchDiagnosticTimestampBinCount = 0;
    component.patchDiagnosticTemporalDiversity = 0;
    component.patchDiagnosticNormalStability = 1;
    component.patchDiagnosticNormalDispersion = 0;
    component.patchDiagnosticReliability = 0;
    component.patchDiagnosticSampleSufficiency = 0;
    component.patchDiagnosticSupportAmplitude = 0;
end

function stability = estimateTemporalReliability(points, timestampBins, patchLocalIndices, patchComponents, params)
% estimateTemporalReliability: Estimate per-point temporal
% reliability before EM from leave-one-bin-out cross-frame spatial support,
% concentration-aware temporal diversity, and patch-local geometric
% consistency. The resulting scores define only a sampling distribution and
% are not passed into the Gaussian-mixture EM objective or M-step.
    pointCount = size(points, 1);
    if pointCount <= 0 || numel(timestampBins) ~= pointCount
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityInput", ...
            "Temporal reliability estimation requires nonempty [N x 2] points and one timestamp bin per point.");
    end
    if numel(patchLocalIndices) ~= numel(patchComponents)
        error("buildTemporalStabilityGmmMap:InvalidTemporalReliabilityPatches", ...
            "Temporal reliability estimation requires one patch component per patch index set.");
    end

    [sampleSufficiency, temporalDiversity] = leaveOneBinOutSpatialSupport(points, timestampBins, params);
    geometricConsistency = patchGeometricConsistency(points, timestampBins, patchLocalIndices, patchComponents, params);
    scores = mappingSupport.clipUnit(sampleSufficiency(:) .* temporalDiversity(:) .* geometricConsistency(:));
    patchScores = zeros(numel(patchLocalIndices), 1);
    for patchIdx = 1:numel(patchLocalIndices)
        patchScores(patchIdx) = mean(scores(patchLocalIndices{patchIdx}));
    end
    [scoreMin, scoreMedian, scoreMax] = mappingSupport.finiteSummary(scores);

    stability = struct();
    stability.scores = scores(:);
    stability.sampleSufficiency = sampleSufficiency(:);
    stability.temporalDiversity = temporalDiversity(:);
    stability.geometricConsistency = geometricConsistency(:);
    stability.patchScores = patchScores(:);
    stability.model = "leaveOneBinOutKernelSupportTimesPatchGeometry";
    stability.kernelBandwidth = params.temporalReliabilityKernelBandwidth;
    stability.kernelRadius = params.temporalReliabilityKernelRadiusMultiplier .* params.temporalReliabilityKernelBandwidth;
    stability.scoreMin = scoreMin;
    stability.scoreMedian = scoreMedian;
    stability.scoreMax = scoreMax;
end


function [sampleSufficiency, temporalDiversity] = leaveOneBinOutSpatialSupport(points, timestampBins, params)
% leaveOneBinOutSpatialSupport: Compute point-level cross-frame spatial
% support by summing Gaussian-kernel evidence from neighboring points in other
% timestamp bins, saturating each bin contribution, and separating total sample
% sufficiency from entropy-based temporal diversity.
    pointCount = size(points, 1);
    sampleSufficiency = zeros(pointCount, 1);
    temporalDiversity = zeros(pointCount, 1);
    [uniqueBins, ~, groupIdx] = unique(timestampBins(:));
    binCount = numel(uniqueBins);
    assert(exist("KDTreeSearcher", "class") == 8, ...
        "KDTreeSearcher is required for temporal reliability spatial support.");
    searcher = KDTreeSearcher(points);
    kernelBandwidth = params.temporalReliabilityKernelBandwidth;
    searchRadius = params.temporalReliabilityKernelRadiusMultiplier .* kernelBandwidth;
    [neighborIdx, neighborDistance] = rangesearch(searcher, points, searchRadius);

    for pointIdx = 1:pointCount
        candidateIdx = neighborIdx{pointIdx}(:);
        candidateDistance = neighborDistance{pointIdx}(:);
        crossBinMask = groupIdx(candidateIdx) ~= groupIdx(pointIdx);
        candidateIdx = candidateIdx(crossBinMask);
        candidateDistance = candidateDistance(crossBinMask);
        if isempty(candidateIdx)
            continue;
        end
        kernelWeights = exp(-0.5 .* (candidateDistance ./ kernelBandwidth).^2);
        rawBinEvidence = accumarray(groupIdx(candidateIdx), kernelWeights, [binCount, 1], @sum, 0);
        saturatedBinEvidence = saturateFrameSupport(rawBinEvidence, params);
        totalSupport = sum(saturatedBinEvidence);
        sampleSufficiency(pointIdx) = computeSampleSufficiency(totalSupport, params);
        activeEvidence = saturatedBinEvidence(saturatedBinEvidence > 0);
        if numel(activeEvidence) >= 2
            q = activeEvidence ./ sum(activeEvidence);
            temporalDiversity(pointIdx) = -sum(q .* log(q + eps)) ./ log(numel(activeEvidence) + eps);
        end
    end
    sampleSufficiency = mappingSupport.clipUnit(sampleSufficiency);
    temporalDiversity = mappingSupport.clipUnit(temporalDiversity);
end


function geometricConsistency = patchGeometricConsistency(points, timestampBins, patchLocalIndices, patchComponents, params)
% patchGeometricConsistency: Compute per-point patch-local geometric
% consistency outside EM. Elongated patches compare each point's normal
% coordinate against the median normal coordinate observed in other timestamp
% bins, while compact patches compare each point to the cross-bin centroid
% median.
    geometricConsistency = zeros(size(points, 1), 1);
    for patchIdx = 1:numel(patchLocalIndices)
        localIndices = patchLocalIndices{patchIdx}(:);
        patchPoints = points(localIndices, :);
        patchBins = timestampBins(localIndices);
        [uniqueBins, ~, patchGroupIdx] = unique(patchBins(:));
        if numel(uniqueBins) < 2
            continue;
        end
        component = patchComponents(patchIdx);
        patchScores = zeros(numel(localIndices), 1);
        if component.covarianceAnisotropy >= params.geometricElongatedAnisotropyThreshold
            normalCoordinates = (patchPoints - component.initialCentroid) * component.n;
            binNormalCoordinates = accumarray(patchGroupIdx, normalCoordinates, [], @median);
            for pointIdx = 1:numel(localIndices)
                otherBinMask = (1:numel(uniqueBins)).' ~= patchGroupIdx(pointIdx);
                crossBinNormalCoordinate = median(binNormalCoordinates(otherBinMask));
                residual = abs(normalCoordinates(pointIdx) - crossBinNormalCoordinate);
                patchScores(pointIdx) = exp(-0.5 .* (residual ./ params.normalStabilityLength).^2);
            end
        else
            binCentroidsX = accumarray(patchGroupIdx, patchPoints(:, 1), [], @median);
            binCentroidsY = accumarray(patchGroupIdx, patchPoints(:, 2), [], @median);
            binCentroids = [binCentroidsX, binCentroidsY];
            for pointIdx = 1:numel(localIndices)
                otherBinMask = (1:numel(uniqueBins)).' ~= patchGroupIdx(pointIdx);
                crossBinCentroid = median(binCentroids(otherBinMask, :), 1);
                residual = norm(patchPoints(pointIdx, :) - crossBinCentroid);
                patchScores(pointIdx) = exp(-0.5 .* (residual ./ params.compactStabilityLength).^2);
            end
        end
        geometricConsistency(localIndices) = mappingSupport.clipUnit(patchScores);
    end
end

function [components, refinementDiagnostics, responsibilities] = fitGaussianMixture(points, components, params, pcaEpsilon)
% fitGaussianMixture: Run ordinary full-covariance Gaussian
% mixture EM on a 2-D sample matrix. This function intentionally accepts no
% timestamps, source indices, temporal-consensus weights, Student-t weights, or
% uniform background state; its convergence objective is the standard
% resampled-data Gaussian mixture log-likelihood.
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
    convergenceScale = max(1, abs(previousLogLikelihood));
    converged = logLikelihoodChange <= emTolerance .* convergenceScale;
end

function refinementDiagnostics = emptyRefinementDiagnostics()
% emptyRefinementDiagnostics: Create a scalar layer-level integrated EM
% status struct for fail-fast refinement diagnostics.
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

function [responsibilities, logLikelihood] = gaussianExpectation(points, components)
% gaussianExpectation: Compute ordinary Gaussian-mixture responsibilities
% and log-likelihood for a 2-D point matrix and current full-covariance
% components.
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
    mappingSupport.validateCovarianceMatrix(covariance, "Ordinary Gaussian log-density");
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

function invCovariance = invertCovariance(covariance, pcaEpsilon)
% invertCovariance: Compute a symmetric inverse covariance matrix from a
% valid two-dimensional structured covariance. Invalid, singular, or
% non-positive-definite covariance matrices are errors under fail-fast
% construction.
    assert(isfinite(pcaEpsilon) && pcaEpsilon > 0, ...
        "pcaEpsilon must be positive and finite.");
    mappingSupport.validateCovarianceMatrix(covariance, "Covariance inversion");
    invCovariance = covariance \ eye(2);
    if any(~isfinite(invCovariance), "all")
        error("buildTemporalStabilityGmmMap:InvalidInverseCovariance", ...
            "Covariance inversion produced non-finite inverse covariance values.");
    end
    invCovariance = (invCovariance + invCovariance.') ./ 2;
end

function saturatedFrameSupport = saturateFrameSupport(rawFrameSupportEvidence, params)
% saturateFrameSupport: Convert raw per-frame foreground evidence into
% bounded frame support so one dense frame cannot dominate repeated-frame map
% support purely through many feature anchors.
    if any(~isfinite(rawFrameSupportEvidence)) || any(rawFrameSupportEvidence < 0)
        error("buildTemporalStabilityGmmMap:InvalidFrameSupport", ...
            "Raw integrated frame support evidence must be finite and nonnegative.");
    end
    if isempty(params.frameCountSaturation)
        saturatedFrameSupport = rawFrameSupportEvidence;
    else
        saturatedFrameSupport = 1 - exp(-rawFrameSupportEvidence ./ params.frameCountSaturation);
    end
    if any(~isfinite(saturatedFrameSupport)) || any(saturatedFrameSupport < 0)
        error("buildTemporalStabilityGmmMap:InvalidFrameSupport", ...
            "Saturated integrated frame support must be finite and nonnegative.");
    end
end

function component = storeCovarianceDiagnostics(component, covarianceDiagnostics, params)
% storeCovarianceDiagnostics: Store covariance safeguard diagnostics and
% query candidate extent on one foreground support component.
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
