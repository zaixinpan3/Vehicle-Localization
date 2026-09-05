function components = applyPosthocSupportFields(points, sourceIndices, timestampBins, supportDiagnostics, components, params, emConverged)
% applyPosthocSupportFields: Store post-hoc support amplitude,
% temporal-diversity, sample-sufficiency, geometric-stability, and assignment
% diagnostics on ordinary GMM components.
%
% Input:
%   points: [N x 2] original class-local BEV feature point coordinates
%   sourceIndices: [N x 1] original input indices for class-local points
%   timestampBins: [N x 1] class-local frame-bin labels
%   supportDiagnostics: scalar post-hoc support evidence struct
%   components: struct array of trained ordinary Gaussian components
%   params: validated class parameter struct
%   emConverged: logical scalar indicating EM convergence status
%
% Output:
%   components: struct array with post-hoc support fields applied
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
%
% Input:
%   points: [N x 2] original class-local BEV feature point coordinates
%   sourceIndices: [N x 1] original input indices for class-local points
%   timestampBins: [N x 1] class-local frame-bin labels
%   supportWeights: [N x 1] nonnegative post-hoc evidence for one component
%   component: scalar ordinary Gaussian support component
%   params: validated class parameter struct
%   emConverged: logical scalar indicating EM convergence status
%
% Output:
%   component: scalar component with post-hoc support diagnostics
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
        supportAmplitude = clipUnit(sampleSufficiency .* temporalDiversity .* geometricStability);
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
%
% Input:
%   none
%
% Output:
%   frameSupport: scalar frame-support diagnostics with zero support
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
%
% Input:
%   assignedWeights: [N x 1] integrated foreground assignment evidence for one
%       component
%   effectiveSupportPointCount: scalar total foreground evidence mass
%   params: validated class parameter struct with emMinEffectiveSupport
%
% Output:
%   assignedPointIdx: [Q x 1] indices of points with numerical support evidence
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
%
% Input:
%   assignedPoints: [Q x 2] points with nonzero final assignment
%   ~: unused refined component mean retained for call-site clarity
%   covariance: [2 x 2] refined structured component covariance
%
% Output:
%   supportBoundingBox: [1 x 4] refined support [minX minY maxX maxY] extent
    validateCovarianceMatrix(covariance, "Support bounding-box computation");
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
%
% Input:
%   timestampBins: [Q x 1] string frame-bin labels for assigned points
%   weights: [Q x 1] nonnegative integrated foreground assignment evidence
%   params: class parameter struct with frame-support settings
%
% Output:
%   frameSupport: scalar struct with raw, saturated, total, active, effective,
%       and temporal-diversity frame-support diagnostics
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
    temporalDiversity = clipUnit(temporalDiversity);

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
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   timestampBins: [N x 1] class-local frame-bin labels
%   weights: [N x 1] integrated foreground assignment evidence for one component
%   component: scalar foreground component with covariance geometry
%   params: validated class parameter struct with geometry settings
%
% Output:
%   geometricStability: scalar geometric consistency score in [0, 1]
%   geometricDispersion: scalar cross-frame geometric dispersion
%   geometricMode: string identifying the selected geometric mode
%   geometricLength: scalar stability length used by the selected mode
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
    geometricStability = clipUnit(geometricStability);
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
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   timestampBins: [N x 1] class-local frame-bin labels
%   weights: [N x 1] integrated foreground assignment evidence
%   centroid: [1 x 2] refined component mean
%   n: [2 x 1] component normal direction
%   params: validated class parameter struct
%
% Output:
%   normalStability: scalar geometric stability score in [0, 1]
%   normalDispersion: scalar robust normal-coordinate dispersion
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
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   timestampBins: [N x 1] class-local frame-bin labels
%   weights: [N x 1] integrated foreground assignment evidence
%   centroid: [1 x 2] refined component mean used as fallback center
%   params: validated class parameter struct
%
% Output:
%   positionStability: scalar geometric stability score in [0, 1]
%   positionDispersion: scalar weighted 2-D centroid dispersion
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
%
% Input:
%   timestampBins: [N x 1] class-local frame-bin labels
%   weights: [N x 1] integrated foreground assignment evidence
%   params: validated class parameter struct
%
% Output:
%   frameWeights: [B x 1] raw frame support evidence totals
%   activeGroupIdx: [A x 1] active frame-bin indices
%   groupIdx: [N x 1] frame-bin index for each point
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
%
% Input:
%   values: [Q x 1] numeric values
%   weights: [Q x 1] nonnegative weights
%
% Output:
%   medianValue: scalar weighted median value
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
