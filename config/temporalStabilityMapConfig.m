function cfg = temporalStabilityMapConfig()
% temporalStabilityMapConfig: Parameters of the semantic temporal-stability
% Gaussian support map. Per class, mutual-kNN patches propose spatial
% support candidates, leave-one-bin-out cross-frame support scores point
% reliability, reliability drives an independent Bernoulli resampling of the
% EM training set, an ordinary full-covariance Gaussian mixture is fitted by
% EM, and temporal support is recomputed post hoc for amplitude, pruning,
% refit, and query-time weighting. The values are the effective parameters
% used to build the saved Mississippi probability-cloud map: the shared
% overrides applied by the original map-building script are folded into the
% defaults, and traffic signs keep their broader patch settings.
%
% Input:
%   none
%
% Output:
%   cfg: struct with classes (set by the caller), pcaEpsilon, defaultParams,
%       and classParams consumed by buildTemporalStabilityGmmMap
    cfg = struct();
    cfg.classes = strings(0, 1);
    cfg.pcaEpsilon = 1.0e-6;
    cfg.minimumConditionalHeightVariance = 1.0e-4;
    cfg.logEnabled = false;
    cfg.defaultParams = classParameters("", 3.0, 1.0, 16, 2.5, 8.0, 2);
    cfg.classParams = [
        classParameters("curb", 4.0, 0.7, 16, 2.5, 8.0, 2)
        classParameters("roadMarking", 4.5, 0.8, 16, 2.5, 8.0, 2)
        classParameters("facade", 6.0, 1.0, 16, 2.5, 8.0, 2)
        classParameters("pole", 1.2, 1.2, 16, 2.5, 8.0, 2)
        classParameters("trafficSign", 1.5, 1.5, 16, 4.0, 8.0, 20)
    ];
end

function params = classParameters(classLabel, lengthParallel, lengthPerp, k, radius, maxDiameter, minComponentPoints)
% classParameters: Package one complete semantic class parameter set.
%
% Input:
%   classLabel: string scalar class key, or "" for the shared defaults
%   lengthParallel, lengthPerp: patch-based initialization covariance
%       lengths along and across the local PCA direction, in meters
%   k, radius, maxDiameter: mutual-kNN neighbor count, pruning radius, and
%       maximum patch diameter, in meters
%   minComponentPoints: minimum points for a buildable patch or component
%
% Output:
%   params: struct with the full parameter set validated by the map builder
    params = struct();
    params.classLabel = string(classLabel);

    % Spatial support patches (mutual kNN graph, radius pruning, PCA splitting)
    params.k = k;
    params.radius = radius;
    params.maxDiameter = maxDiameter;
    params.lengthParallel = lengthParallel;
    params.lengthPerp = lengthPerp;
    params.minComponentPoints = minComponentPoints;
    params.minPatchSupportForGmmSeed = 0.0;

    % Temporal reliability scoring and Bernoulli resampling of the EM training set
    params.timestampMaxBins = 10.0;
    params.frameCountSaturation = 1.0;
    params.effectiveFramePrior = 1.0;
    params.normalStabilityLength = 1.5;
    params.compactStabilityLength = 1.5;
    params.geometricElongatedAnisotropyThreshold = 3.0;
    params.temporalReliabilityKernelBandwidth = 2.5;
    params.temporalReliabilityKernelRadiusMultiplier = 3.0;
    params.temporalReliabilityPower = 2.0;
    params.temporalReliabilitySamplingFloor = 0.05;
    params.temporalResampleExpectedCountMultiplier = 1.0;
    params.temporalResampleRandomSeed = 1;

    % Ordinary Gaussian-mixture EM and covariance safeguards
    params.emMaxIterations = 80;
    params.emTolerance = 1.0e-2;
    params.emMinEffectiveSupport = 1.0e-6;
    params.emMinFrameWeight = 1.0e-9;
    params.minCovarianceEigenvalue = 1.0e-6;
    params.maxCovarianceEigenvalue = 1.0e4;
    params.storeEmAssignments = false;

    % Post-hoc temporal support pruning and refit
    params.minComponentSupport = 0.0;
    params.minTimestampBins = 2;
    params.minNormalStability = 0.0;
    params.refitAfterTemporalPruning = true;

    % Query-time support evaluation
    params.priorSupport = 0.05;
    params.querySupportMahalanobisRadius = 3.0;
    params.queryCandidateComponentCount = inf;
    params.queryCandidateRadius = 1.0;
end
