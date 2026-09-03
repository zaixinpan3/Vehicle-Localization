function component = emptyComponent()
% emptyComponent: Create an empty structured Gaussian support component
% with patch diagnostics, EM mixture parameters, covariance geometry, and
% post-hoc temporal support amplitude fields.
%
% Input:
%   none
%
% Output:
%   component: scalar struct with initialized component fields
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
