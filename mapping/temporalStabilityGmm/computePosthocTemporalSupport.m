function supportDiagnostics = computePosthocTemporalSupport(points, sourceIndices, timestampBins, stability, components, params)
% computePosthocTemporalSupport: Evaluate the trained ordinary GMM on the
% original class points, multiply responsibilities by precomputed temporal
% reliability scores, and package post-hoc temporal support evidence without
% changing any GMM parameters.
%
% Input:
%   points: [N x 2] original class-local BEV feature point coordinates
%   sourceIndices: [N x 1] original input indices for class-local points
%   timestampBins: [N x 1] class-local frame-bin labels
%   stability: scalar struct with precomputed reliability scores
%   components: struct array of trained ordinary Gaussian components
%   params: validated class parameter struct
%
% Output:
%   supportDiagnostics: scalar diagnostics for post-hoc temporal support
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
