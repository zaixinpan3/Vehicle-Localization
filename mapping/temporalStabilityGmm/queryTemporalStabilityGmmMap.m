function scores = queryTemporalStabilityGmmMap(firstInput, secondInput, thirdInput, varargin)
% queryTemporalStabilityGmmMap: Evaluate a structured
% semantic temporal-stability Gaussian mixture support map or saved
% sliding-window probability-cloud map at BEV query locations. Single-layer
% query returns stability-weighted support scores from ordinary-GMM spatial
% components, post-hoc temporal support amplitudes, peak-normalized
% Gaussian-shaped component support, class prior support, and bounded noisy-OR
% fusion: score = 1 - prod(1 - componentSupport). Saved sliding-window map
% query evaluates every batch map containing the requested class and fuses
% overlapping or neighboring batch responses by max-envelope, so batch
% boundaries do not create hard support discontinuities and overlapping windows
% do not double-count the same physical evidence. The query implementation
% exposes support-map scoring only. GMM mixture weights are diagnostics for EM
% and do not modulate support. Candidate retrieval uses component centroids
% with builder-provided covariance-extent inflation so elongated support
% components are not missed solely by a small centroid radius.
% The query path follows a fail-fast policy and expects maps produced by the
% current builder; the query path expects maps produced by the current builder.
% Missing current fields, empty layers, malformed candidate
% configuration, invalid covariance matrices, or invalid numerical states are
% errors rather than legacy compatibility cases.
%
% Input:
%   firstInput: queryPoints [M x 2], saved sliding-window probability-cloud
%       map, semantic GMM feature map, or semantic GMM layer
%   secondInput: saved probability-cloud map, semantic GMM feature map, or
%       layer when firstInput is queryPoints; otherwise queryPoints [M x 2]
%   thirdInput: optional class label scalar or [M x 1] query labels when
%       evaluating a full map
%
% Output:
%   scores: [M x 1] class-specific scores, [M x C] all-class scores when a
%       full map is queried without labels, or [M x 1] label-matched support
%       scores
    if nargin < 2 || ~isempty(varargin)
        error("queryTemporalStabilityGmmMap:InvalidInputCount", ...
            "queryTemporalStabilityGmmMap accepts only a map or layer, query points, and optional labels.");
    end

    if isnumeric(firstInput)
        queryPoints = firstInput;
        layerOrMap = secondInput;
        if nargin >= 3
            requestedLabels = thirdInput;
        else
            requestedLabels = strings(0, 1);
        end
    else
        layerOrMap = firstInput;
        queryPoints = secondInput;
        if nargin >= 3
            requestedLabels = thirdInput;
        else
            requestedLabels = strings(0, 1);
        end
    end

    assert(isnumeric(queryPoints) && ismatrix(queryPoints) && size(queryPoints, 2) == 2, ...
        "queryPoints must be an [M x 2] numeric array.");
    queryPoints = double(queryPoints);
    if isempty(queryPoints) || any(~isfinite(queryPoints), "all")
        error("queryTemporalStabilityGmmMap:InvalidQueryPoints", ...
            "queryPoints must be nonempty and finite.");
    end

    if isstruct(layerOrMap) && isfield(layerOrMap, "batchMaps")
        scores = evaluateSlidingWindowProbabilityCloudMap(layerOrMap, queryPoints, requestedLabels);
    elseif isstruct(layerOrMap) && isfield(layerOrMap, "layers")
        scores = evaluateMap(layerOrMap, queryPoints, requestedLabels);
    else
        if ~isempty(requestedLabels)
            error("queryTemporalStabilityGmmMap:UnexpectedLayerLabels", ...
                "Class labels are only accepted when querying a full semantic GMM map.");
        end
        scores = evaluateLayer(layerOrMap, queryPoints);
    end
end

function scores = evaluateSlidingWindowProbabilityCloudMap(probabilityCloudMap, queryPoints, requestedLabels)
% evaluateSlidingWindowProbabilityCloudMap: Evaluate a saved
% sliding-window probability-cloud artifact by querying every nonempty batch
% map that contains the requested semantic class and fusing batch scores by
% max-envelope. Component-level query inside each batch still uses canonical
% noisy-OR; the cross-batch max prevents overlap windows from double-counting
% duplicated components for the same physical landmark.
%
% Input:
%   probabilityCloudMap: saved map artifact with batchMaps and GMM batch maps
%   queryPoints: [M x 2] numeric BEV query point coordinates
%   requestedLabels: empty, scalar class label, or [M x 1] query labels
%
% Output:
%   scores: [M x 1] or [M x C] support score array
    validateSlidingWindowProbabilityCloudMap(probabilityCloudMap);
    queryCount = size(queryPoints, 1);
    requestedLabels = string(requestedLabels);

    if isempty(requestedLabels)
        classLabels = resolveSlidingWindowClassLabels(probabilityCloudMap);
        scores = zeros(queryCount, numel(classLabels));
        for classIdx = 1:numel(classLabels)
            scores(:, classIdx) = evaluateSlidingWindowClass(probabilityCloudMap, queryPoints, classLabels(classIdx));
        end
    elseif isscalar(requestedLabels)
        scores = evaluateSlidingWindowClass(probabilityCloudMap, queryPoints, requestedLabels);
    else
        queryLabels = requestedLabels(:);
        assert(numel(queryLabels) == queryCount, ...
            "queryLabels must contain one class label per query point.");
        scores = zeros(queryCount, 1);
        uniqueLabels = unique(queryLabels, "stable");
        for labelIdx = 1:numel(uniqueLabels)
            queryMask = queryLabels == uniqueLabels(labelIdx);
            scores(queryMask) = evaluateSlidingWindowClass(probabilityCloudMap, queryPoints(queryMask, :), uniqueLabels(labelIdx));
        end
    end
end

function validateSlidingWindowProbabilityCloudMap(probabilityCloudMap)
% validateSlidingWindowProbabilityCloudMap: Validate the saved
% sliding-window probability-cloud artifact fields required for cross-batch
% support query.
%
% Input:
%   probabilityCloudMap: saved map artifact candidate
%
% Output:
%   none
    requiredMapFields = ["mapType", "batchMaps"];
    if ~isstruct(probabilityCloudMap) || ~isscalar(probabilityCloudMap)
        error("queryTemporalStabilityGmmMap:InvalidMap", ...
            "probabilityCloudMap must be a scalar saved sliding-window probability-cloud map struct.");
    end
    for fieldName = requiredMapFields
        if ~isfield(probabilityCloudMap, char(fieldName))
            error("queryTemporalStabilityGmmMap:MissingMapField", ...
                "Sliding-window map is missing required field: %s.", char(fieldName));
        end
    end
    mapType = string(probabilityCloudMap.mapType);
    if ~isscalar(mapType) || mapType ~= "missisipiSlidingWindowIntegratedTemporalGMMProbabilityCloudMap"
        error("queryTemporalStabilityGmmMap:InvalidMapType", ...
            "Sliding-window map mapType must be missisipiSlidingWindowIntegratedTemporalGMMProbabilityCloudMap.");
    end
    if isempty(probabilityCloudMap.batchMaps) || ~isstruct(probabilityCloudMap.batchMaps)
        error("queryTemporalStabilityGmmMap:InvalidMap", ...
            "Sliding-window maps must contain a nonempty batchMaps struct array.");
    end
    for batchIdx = 1:numel(probabilityCloudMap.batchMaps)
        batchMap = probabilityCloudMap.batchMaps(batchIdx);
        if ~isfield(batchMap, "frameIndices") || ~isnumeric(batchMap.frameIndices) || isempty(batchMap.frameIndices) || any(~isfinite(batchMap.frameIndices))
            error("queryTemporalStabilityGmmMap:InvalidMap", ...
                "Each batch map must contain finite frameIndices.");
        end
        if ~isfield(batchMap, "gmmMap")
            error("queryTemporalStabilityGmmMap:MissingMapField", ...
                "Each batch map must contain gmmMap.");
        end
        if ~isempty(batchMap.gmmMap)
            validateCurrentMap(batchMap.gmmMap);
        end
    end
end

function classLabels = resolveSlidingWindowClassLabels(probabilityCloudMap)
% resolveSlidingWindowClassLabels: Resolve the ordered semantic class
% labels exposed by a saved sliding-window probability-cloud map, preferring
% the artifact featureNames field and otherwise deriving labels from batch
% layers.
%
% Input:
%   probabilityCloudMap: validated saved sliding-window map artifact
%
% Output:
%   classLabels: [C x 1] string vector of class labels
    if isfield(probabilityCloudMap, "featureNames") && ~isempty(probabilityCloudMap.featureNames)
        classLabels = string(probabilityCloudMap.featureNames(:));
    else
        labelParts = cell(numel(probabilityCloudMap.batchMaps), 1);
        labelPartCount = 0;
        for batchIdx = 1:numel(probabilityCloudMap.batchMaps)
            batchMap = probabilityCloudMap.batchMaps(batchIdx).gmmMap;
            if ~isempty(batchMap) && isfield(batchMap, "layers")
                labelPartCount = labelPartCount + 1;
                labelParts{labelPartCount} = string({batchMap.layers.classLabel}).';
            end
        end
        if labelPartCount > 0
            classLabels = vertcat(labelParts{1:labelPartCount});
            classLabels = unique(classLabels, "stable");
        else
            classLabels = strings(0, 1);
        end
    end
    assert(~isempty(classLabels), "Sliding-window probability-cloud map does not expose any semantic class labels.");
end

function scores = evaluateSlidingWindowClass(probabilityCloudMap, queryPoints, classLabel)
% evaluateSlidingWindowClass: Evaluate one semantic class across all
% saved batch maps and fuse batch responses by max-envelope.
%
% Input:
%   probabilityCloudMap: saved sliding-window map artifact
%   queryPoints: [M x 2] numeric BEV query point coordinates
%   classLabel: requested semantic class label
%
% Output:
%   scores: [M x 1] max-envelope support scores in [0, 1]
    queryCount = size(queryPoints, 1);
    batchScores = zeros(queryCount, numel(probabilityCloudMap.batchMaps));
    contributingBatchCount = 0;

    for batchIdx = 1:numel(probabilityCloudMap.batchMaps)
        batchGmmMap = probabilityCloudMap.batchMaps(batchIdx).gmmMap;
        if isempty(batchGmmMap)
            continue;
        end
        layerIdx = findLayerIndex(batchGmmMap, classLabel);
        if layerIdx < 1
            continue;
        end
        contributingBatchCount = contributingBatchCount + 1;
        batchScores(:, contributingBatchCount) = evaluateLayer(batchGmmMap.layers(layerIdx), queryPoints);
    end

    if contributingBatchCount < 1
        error("queryTemporalStabilityGmmMap:MissingClassLayer", ...
            "Sliding-window probability-cloud map does not contain class label: %s.", char(string(classLabel)));
    end
    scores = max(clipQueryUnit(batchScores(:, 1:contributingBatchCount)), [], 2);
    scores = clipQueryUnit(scores);
end

function layerIdx = findLayerIndex(gmmMap, classLabel)
% findLayerIndex: Return the semantic layer index matching a requested
% class label without erroring when a particular batch lacks that class.
%
% Input:
%   gmmMap: current-builder structured GMM map
%   classLabel: requested semantic class label
%
% Output:
%   layerIdx: scalar positive layer index or zero when absent
    if ~isfield(gmmMap.layers, "classLabel")
        error("queryTemporalStabilityGmmMap:MissingLayerField", ...
            "Map layers must contain classLabel.");
    end
    targetLabel = string(classLabel);
    layerLabels = string({gmmMap.layers.classLabel});
    layerIdx = find(layerLabels == targetLabel, 1);
    if isempty(layerIdx)
        layerIdx = 0;
    end
end

function scores = evaluateMap(gmmMap, queryPoints, requestedLabels)
% evaluateMap: Evaluate a full structured semantic GMM map for one
% requested class, per-query requested labels, or all semantic layers.
%
% Input:
%   gmmMap: struct with layers produced by buildTemporalStabilityGmmMap
%   queryPoints: [M x 2] numeric BEV query point coordinates
%   requestedLabels: empty, scalar class label, or [M x 1] query labels
%
% Output:
%   scores: [M x 1] or [M x C] support score array
    validateCurrentMap(gmmMap);
    queryCount = size(queryPoints, 1);
    requestedLabels = string(requestedLabels);

    if isempty(requestedLabels)
        layerCount = numel(gmmMap.layers);
        scores = zeros(queryCount, layerCount);
        for layerIdx = 1:layerCount
            scores(:, layerIdx) = evaluateLayer(gmmMap.layers(layerIdx), queryPoints);
        end
    elseif isscalar(requestedLabels)
        layer = selectLayer(gmmMap, requestedLabels);
        scores = evaluateLayer(layer, queryPoints);
    else
        queryLabels = requestedLabels(:);
        assert(numel(queryLabels) == queryCount, ...
            "queryLabels must contain one class label per query point.");
        scores = zeros(queryCount, 1);
        uniqueLabels = unique(queryLabels, "stable");
        for labelIdx = 1:numel(uniqueLabels)
            queryMask = queryLabels == uniqueLabels(labelIdx);
            layer = selectLayer(gmmMap, uniqueLabels(labelIdx));
            scores(queryMask) = evaluateLayer(layer, queryPoints(queryMask, :));
        end
    end
end

function validateCurrentMap(gmmMap)
% validateCurrentMap: Validate the top-level current-builder map fields
% before layer selection or query evaluation.
%
% Input:
%   gmmMap: structured semantic GMM map candidate
%
% Output:
%   none
    requiredMapFields = ["mapType", "classLabels", "layers"];
    if ~isstruct(gmmMap)
        error("queryTemporalStabilityGmmMap:InvalidMap", ...
            "gmmMap must be a nonempty struct produced by the current buildTemporalStabilityGmmMap.");
    end
    for fieldName = requiredMapFields
        if ~isfield(gmmMap, char(fieldName))
            error("queryTemporalStabilityGmmMap:MissingMapField", ...
                "Map is missing required current-builder field: %s.", char(fieldName));
        end
    end
    mapType = string(gmmMap.mapType);
    if ~isscalar(mapType) || mapType ~= "semanticTemporalStabilityStructuredGMM"
        error("queryTemporalStabilityGmmMap:InvalidMapType", ...
            "Map mapType must be semanticTemporalStabilityStructuredGMM.");
    end
    if isempty(gmmMap.layers)
        error("queryTemporalStabilityGmmMap:InvalidMap", ...
            "Current-builder maps must contain at least one nonempty semantic layer.");
    end
    classLabels = string(gmmMap.classLabels(:));
    if isempty(classLabels) || numel(classLabels) ~= numel(gmmMap.layers)
        error("queryTemporalStabilityGmmMap:InvalidClassLabels", ...
            "Map classLabels must contain one label for each semantic layer.");
    end
end

function layer = selectLayer(gmmMap, classLabel)
% selectLayer: Select one semantic layer from a full structured GMM map
% using a class label convertible to string.
%
% Input:
%   gmmMap: struct with layers produced by buildTemporalStabilityGmmMap
%   classLabel: requested semantic class label
%
% Output:
%   layer: semantic GMM layer for the requested class
    targetLabel = string(classLabel);
    layerLabels = string({gmmMap.layers.classLabel});
    layerIdx = find(layerLabels == targetLabel, 1);
    if isempty(layerIdx)
        error("queryTemporalStabilityGmmMap:MissingClassLayer", ...
            "Structured GMM map does not contain class label: %s.", char(targetLabel));
    end
    layer = gmmMap.layers(layerIdx);
end

function scores = evaluateLayer(layer, queryPoints)
% evaluateLayer: Evaluate one semantic GMM layer at query points using
% stability-weighted support-map scoring.
%
% Input:
%   layer: semantic GMM layer struct with components
%   queryPoints: [M x 2] numeric BEV query point coordinates
%
% Output:
%   scores: [M x 1] support scores in [0, 1]
    validateCurrentLayer(layer);
    queryCount = size(queryPoints, 1);
    scores = repmat(layer.priorScore, queryCount, 1);

    candidateLists = findCandidateComponents(queryPoints, layer);
    for queryIdx = 1:queryCount
        candidateIdx = candidateLists{queryIdx};
        componentSupports = zeros(numel(candidateIdx), 1);
        for candidateListIdx = 1:numel(candidateIdx)
            componentIdx = candidateIdx(candidateListIdx);
            componentSupports(candidateListIdx) = componentSupport(layer.components(componentIdx), queryPoints(queryIdx, :));
        end
        componentScore = combineComponentSupports(componentSupports);
        if componentScore > 0
            scores(queryIdx) = max(layer.priorScore, componentScore);
        end
    end
    scores = clipQueryUnit(scores);
end

function validateCurrentLayer(layer)
% validateCurrentLayer: Validate that a query layer has the current
% builder-produced structure required by fail-fast support evaluation.
%
% Input:
%   layer: semantic GMM layer candidate
%
% Output:
%   none
    requiredLayerFields = ["components", "priorScore", "queryCandidateComponentCount", ...
        "queryCandidateRadius", "queryCandidateRadiusInflation", "centroidSearcher"];
    if ~isstruct(layer)
        error("queryTemporalStabilityGmmMap:InvalidLayer", ...
            "Layer input must be a current semantic GMM layer struct.");
    end
    for fieldName = requiredLayerFields
        if ~isfield(layer, char(fieldName))
            error("queryTemporalStabilityGmmMap:MissingLayerField", ...
                "Layer is missing required current-builder field: %s.", char(fieldName));
        end
    end
    if isempty(layer.components)
        error("queryTemporalStabilityGmmMap:EmptyLayer", ...
            "Semantic GMM layers must contain at least one retained component.");
    end
    if ~isscalar(layer.priorScore) || ~isnumeric(layer.priorScore) || ~isfinite(layer.priorScore) || layer.priorScore < 0 || layer.priorScore > 1
        error("queryTemporalStabilityGmmMap:InvalidPriorScore", ...
            "Layer priorScore must be a finite scalar in [0, 1].");
    end
    for componentIdx = 1:numel(layer.components)
        validateCurrentComponent(layer.components(componentIdx));
    end
end

function validateCurrentComponent(component)
% validateCurrentComponent: Validate one retained component from the
% current support-map builder before query evaluation.
%
% Input:
%   component: scalar retained semantic support component
%
% Output:
%   none
    requiredComponentFields = ["mean", "covariance", "invCovariance", "supportAmplitude", ...
        "mixtureWeight"];
    for fieldName = requiredComponentFields
        if ~isfield(component, char(fieldName))
            error("queryTemporalStabilityGmmMap:MissingComponentField", ...
                "Component is missing required current-builder field: %s.", char(fieldName));
        end
    end
    if ~isnumeric(component.mean) || ~isequal(size(component.mean), [1 2]) || any(~isfinite(component.mean))
        error("queryTemporalStabilityGmmMap:InvalidComponentMean", ...
            "Component mean must be a finite [1 x 2] vector.");
    end
    if ~isscalar(component.supportAmplitude) || ~isnumeric(component.supportAmplitude) || ~isfinite(component.supportAmplitude) || component.supportAmplitude < 0 || component.supportAmplitude > 1
        error("queryTemporalStabilityGmmMap:InvalidSupportAmplitude", ...
            "Component supportAmplitude must be a finite scalar in [0, 1].");
    end
    if ~isscalar(component.mixtureWeight) || ~isnumeric(component.mixtureWeight) || ~isfinite(component.mixtureWeight) || component.mixtureWeight < 0
        error("queryTemporalStabilityGmmMap:InvalidEmMixtureWeight", ...
            "Component mixtureWeight must be a finite nonnegative EM mixture weight.");
    end
    validateQueryCovarianceMatrix(component.covariance, "Query component covariance");
    if ~isnumeric(component.invCovariance) || ~isequal(size(component.invCovariance), [2 2]) || any(~isfinite(component.invCovariance), "all")
        error("queryTemporalStabilityGmmMap:InvalidInverseCovariance", ...
            "Component invCovariance must be a finite [2 x 2] matrix.");
    end
end

function validateQueryCovarianceMatrix(covariance, contextName)
% validateQueryCovarianceMatrix: Validate finite symmetric positive definite
% covariance matrices before support query evaluation.
%
% Input:
%   covariance: [2 x 2] covariance matrix
%   contextName: character vector identifying the validation context
%
% Output:
%   none
    if ~isnumeric(covariance) || ~isequal(size(covariance), [2 2]) || any(~isfinite(covariance), "all")
        error("queryTemporalStabilityGmmMap:InvalidCovarianceMatrix", ...
            "%s requires a finite [2 x 2] covariance matrix.", contextName);
    end
    if norm(covariance - covariance.', "fro") > 1.0e-10 .* max(1, norm(covariance, "fro"))
        error("queryTemporalStabilityGmmMap:NonSymmetricCovariance", ...
            "%s requires a symmetric covariance matrix.", contextName);
    end
    [~, cholFlag] = chol((covariance + covariance.') ./ 2);
    covarianceDeterminant = det(covariance);
    if cholFlag ~= 0 || ~isfinite(covarianceDeterminant) || covarianceDeterminant <= 0
        error("queryTemporalStabilityGmmMap:NonPositiveDefiniteCovariance", ...
            "%s requires a symmetric positive definite covariance matrix with a finite positive determinant.", contextName);
    end
end

function candidateLists = findCandidateComponents(queryPoints, layer)
% findCandidateComponents: Retrieve candidate component indices for each
% query point using the layer centroid KD-tree when configured, inflating finite
% centroid radii by the builder's maximum covariance support extent, or use all
% components when exact all-component support evaluation is requested.
%
% Input:
%   queryPoints: [M x 2] numeric BEV query point coordinates
%   layer: semantic GMM layer with components and optional centroidSearcher
%
% Output:
%   candidateLists: [M x 1] cell array of component index vectors
    queryCount = size(queryPoints, 1);
    componentCount = numel(layer.components);
    candidateCount = resolveCandidateCount(layer, componentCount);
    candidateRadius = resolveCandidateRadius(layer);

    if ~isfinite(candidateRadius) && (~isfinite(candidateCount) || candidateCount >= componentCount)
        candidateLists = repmat({1:componentCount}, queryCount, 1);
        return;
    end
    requireCentroidSearcher(layer);

    if isfinite(candidateRadius)
        candidateLists = rangeCandidateComponents(queryPoints, layer.centroidSearcher, candidateRadius, candidateCount);
    else
        candidateIdx = knnsearch(layer.centroidSearcher, queryPoints, "K", candidateCount);
        candidateLists = cell(queryCount, 1);
        for queryIdx = 1:queryCount
            candidateLists{queryIdx} = candidateIdx(queryIdx, :);
        end
    end
end

function candidateLists = rangeCandidateComponents(queryPoints, centroidSearcher, candidateRadius, ~)
% rangeCandidateComponents: Retrieve radius-limited centroid candidates
% for each query point. Radius-limited retrieval intentionally keeps every
% component in the inflated radius so anisotropic support components are not
% dropped by a centroid-only nearest-count truncation.
%
% Input:
%   queryPoints: [M x 2] numeric BEV query point coordinates
%   centroidSearcher: KDTreeSearcher over component means
%   candidateRadius: scalar centroid search radius
%   candidateCount: scalar maximum number of candidate components or Inf
%
% Output:
%   candidateLists: [M x 1] cell array of component index vectors
    [rangeIdx, ~] = rangesearch(centroidSearcher, queryPoints, candidateRadius);
    queryCount = size(queryPoints, 1);
    candidateLists = cell(queryCount, 1);

    for queryIdx = 1:queryCount
        candidateLists{queryIdx} = rangeIdx{queryIdx};
    end
end

function requireCentroidSearcher(layer)
% requireCentroidSearcher: Require a current centroid searcher whenever
% query candidate limits request indexed component preselection.
%
% Input:
%   layer: semantic GMM layer
%
% Output:
%   none
    if ~isfield(layer, "centroidSearcher") || isempty(layer.centroidSearcher)
        error("queryTemporalStabilityGmmMap:MissingCentroidSearcher", ...
            "Query candidate preselection requires a valid centroidSearcher from the current builder.");
    end
    if ~isa(layer.centroidSearcher, "KDTreeSearcher")
        error("queryTemporalStabilityGmmMap:InvalidCentroidSearcher", ...
            "Query candidate preselection requires a KDTreeSearcher from the current builder.");
    end
end

function candidateCount = resolveCandidateCount(layer, componentCount)
% resolveCandidateCount: Resolve the configured maximum number of
% centroid candidate components from the current builder layer.
%
% Input:
%   layer: semantic GMM layer struct
%   componentCount: scalar total component count
%
% Output:
%   candidateCount: scalar candidate component count or Inf
    if ~isfield(layer, "queryCandidateComponentCount")
        error("queryTemporalStabilityGmmMap:MissingQueryCandidateComponentCount", ...
            "Current builder layers must contain queryCandidateComponentCount.");
    end
    candidateCount = layer.queryCandidateComponentCount;
    if ~isscalar(candidateCount) || ~isnumeric(candidateCount) || isnan(candidateCount) || candidateCount <= 0
        error("queryTemporalStabilityGmmMap:InvalidQueryCandidateComponentCount", ...
            "queryCandidateComponentCount must be a positive numeric scalar or Inf.");
    end
    if isfinite(candidateCount) && candidateCount ~= round(candidateCount)
        error("queryTemporalStabilityGmmMap:InvalidQueryCandidateComponentCount", ...
            "Finite queryCandidateComponentCount must be an integer.");
    end

    if isfinite(candidateCount)
        candidateCount = min(componentCount, candidateCount);
    end
end

function candidateRadius = resolveCandidateRadius(layer)
% resolveCandidateRadius: Resolve the configured centroid candidate
% radius from the current builder layer.
%
% Input:
%   layer: semantic GMM layer struct
%
% Output:
%   candidateRadius: scalar positive radius or Inf
    if ~isfield(layer, "queryCandidateRadius")
        error("queryTemporalStabilityGmmMap:MissingQueryCandidateRadius", ...
            "Current builder layers must contain queryCandidateRadius.");
    end
    candidateRadius = layer.queryCandidateRadius;
    if ~isscalar(candidateRadius) || ~isnumeric(candidateRadius) || isnan(candidateRadius) || candidateRadius <= 0
        error("queryTemporalStabilityGmmMap:InvalidQueryCandidateRadius", ...
            "queryCandidateRadius must be a positive numeric scalar or Inf.");
    end
    if ~isfield(layer, "queryCandidateRadiusInflation")
        error("queryTemporalStabilityGmmMap:MissingQueryCandidateRadiusInflation", ...
            "Current builder layers must contain queryCandidateRadiusInflation.");
    end
    radiusInflation = layer.queryCandidateRadiusInflation;
    if ~isscalar(radiusInflation) || ~isnumeric(radiusInflation) || ~isfinite(radiusInflation) || radiusInflation < 0
        error("queryTemporalStabilityGmmMap:InvalidQueryCandidateRadiusInflation", ...
            "queryCandidateRadiusInflation must be a finite nonnegative numeric scalar.");
    end
    if isfinite(candidateRadius)
        candidateRadius = candidateRadius + radiusInflation;
    end
end

function support = componentSupport(component, queryPoint)
% componentSupport: Evaluate one peak-normalized Gaussian support
% component at one query point using the component inverse covariance and
% post-hoc temporal support amplitude only. GMM mixture weights do not modulate
% support scores by default, so small but stable landmarks can still produce
% strong support.
%
% Input:
%   component: scalar structured Gaussian support component
%   queryPoint: [1 x 2] numeric BEV query point coordinate
%
% Output:
%   support: scalar amplitude-weighted component support in [0, 1]
    delta = queryPoint - component.mean;
    distanceSquared = delta * component.invCovariance * delta.';
    if ~isfinite(distanceSquared)
        error("queryTemporalStabilityGmmMap:InvalidMahalanobisDistance", ...
            "Support output produced a non-finite Mahalanobis distance.");
    end
    if distanceSquared < -1.0e-10
        error("queryTemporalStabilityGmmMap:NegativeMahalanobisDistance", ...
            "Support output produced a meaningfully negative Mahalanobis distance.");
    end
    if distanceSquared < 0
        distanceSquared = 0;
    end
    support = clipQueryUnit(component.supportAmplitude .* exp(-0.5 .* distanceSquared));
end

function score = combineComponentSupports(componentSupports)
% combineComponentSupports: Combine candidate component supports into one
% semantic support score using canonical noisy-OR.
%
% Input:
%   componentSupports: [K x 1] amplitude-weighted component supports
%
% Output:
%   score: scalar combined support score in [0, 1]
    componentSupports = clipQueryUnit(componentSupports(:));
    if isempty(componentSupports)
        score = 0;
    else
        score = 1 - prod(1 - componentSupports);
    end
    score = clipQueryUnit(score);
end

function value = clipQueryUnit(value)
% clipQueryUnit: Validate support values against the closed unit interval,
% allowing only harmless round-off at the boundary.
%
% Input:
%   value: numeric support value or array
%
% Output:
%   value: numeric support value or array after boundary round-off correction
    if any(~isfinite(value), "all")
        error("queryTemporalStabilityGmmMap:InvalidUnitValue", ...
            "Support values must be finite.");
    end
    lowerMask = value < 0;
    upperMask = value > 1;
    if any(value(lowerMask) < -1.0e-12, "all") || any(value(upperMask) > 1 + 1.0e-12, "all")
        error("queryTemporalStabilityGmmMap:UnitValueOutOfRange", ...
            "Support values must lie in [0, 1].");
    end
    value(lowerMask) = 0;
    value(upperMask) = 1;
end
