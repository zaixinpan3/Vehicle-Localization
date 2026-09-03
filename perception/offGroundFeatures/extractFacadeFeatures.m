function facade = extractFacadeFeatures(columnMaps, fineVoxelGrid, cfg)
% extractFacadeFeatures: Detect building facades in the non-ground columns
% with a global-to-local scheme. Globally, facade lines are dominant peaks
% of an oriented, weighted Hough transform of the fine-column line score,
% and every supporting column is assigned to its owning line. Locally, the
% fine voxel grid under each assigned column is checked patch by patch for
% planarity and normal alignment with the line, and columns without such
% support are released. Facade detection is a dataset policy switch
% (cfg.facadeDetectionEnabled); when it is off the result is empty but
% keeps the full field set.
%
% Input:
%   columnMaps: struct from buildFineColumnFeatureMaps extended with
%       runLayerMap, lineScore, and normalOrientation
%   fineVoxelGrid: struct from buildFineColumnFeatureMaps
%   cfg: struct from offGroundFeatureConfig
%
% Output:
%   facade: struct with enabled, mask [Ny x Nx], lineMap [Ny x Nx] uint16,
%       pillarLinIdx, pillarLineIdx, detectedLines [L x 4] meters,
%       detectedLinesRaw, detectorMask, detectorDiagnostics, and
%       refineDiagnostics
    mapSize = size(columnMaps.occupiedMask);
    enabled = isFacadeDetectionEnabled(cfg);
    if enabled
        [detParams, fineFacadeCfg] = resolveFacadeDetectionParams(cfg, columnMaps.dx, columnMaps.dy);
        [detectedLinesRaw, detectorDiagnostics] = detectFacadeLines(columnMaps.lineScore, columnMaps.normalOrientation, ...
            columnMaps.occupiedMask, columnMaps.runLayerMap, [], columnMaps.origin, columnMaps.dx, columnMaps.dy, detParams);
        detectedLines = detectedLinesRaw;
        detectorMask = detectorDiagnostics.facadeMask;
        [lineMap, mask] = assignFacadeColumnsToLines( ...
            detectorDiagnostics, detectedLinesRaw, detectedLines, detectorMask, ...
            columnMaps.xMap, columnMaps.yMap, columnMaps.dx, columnMaps.dy, fineFacadeCfg);
        [lineMap, mask, refineDiagnostics] = refineFacadeWithFineGrid( ...
            lineMap, mask, fineVoxelGrid, detectedLines, fineFacadeCfg);
    else
        detectedLinesRaw = zeros(0, 4);
        detectedLines = zeros(0, 4);
        detectorDiagnostics = buildDisabledFacadeDetectorDiagnostics(mapSize);
        detectorMask = false(mapSize);
        lineMap = zeros(mapSize, "uint16");
        mask = false(mapSize);
        refineDiagnostics = struct("enabled", false, "reason", "facadeDetectionDisabled");
    end

    facade = struct();
    facade.enabled = enabled;
    facade.mask = mask;
    facade.lineMap = lineMap;
    facade.pillarLinIdx = find(mask);
    facade.pillarLineIdx = double(lineMap(facade.pillarLinIdx));
    facade.detectedLines = detectedLines;
    facade.detectedLinesRaw = detectedLinesRaw;
    facade.detectorMask = detectorMask;
    facade.detectorDiagnostics = detectorDiagnostics;
    facade.refineDiagnostics = refineDiagnostics;
end

function [detParams, fineFacadeCfg] = resolveFacadeDetectionParams(cfg, dx, dy)
% resolveFacadeDetectionParams: Scale facade detector radii and
% pixel-count thresholds from a 1-meter reference cell to the current
% fine-grid XY spacing while preserving metric behavior for line assignment.
%
% Input:
%   cfg: off-ground processing configuration struct
%   dx, dy: fine-grid XY spacing in meters
%
% Output:
%   detParams: scaled facade detector parameter struct
%   fineFacadeCfg: cfg copy with scaled fallback assignment threshold
    detParams = buildFacadeDetectionParams(cfg);
    fineFacadeCfg = cfg;
    referenceCellSize = 1.0;
    cellScale = max(1, referenceCellSize ./ max(max(double(dx), double(dy)), eps));
    detParams.lineDistanceThresholdVoxels = max(1, double(detParams.lineDistanceThresholdVoxels) .* cellScale);
    detParams.facadeLineFillGapVoxels = max(1, round(double(detParams.facadeLineFillGapVoxels) .* cellScale));
    detParams.facadeLineMinLengthVoxels = max(6, round(double(detParams.facadeLineMinLengthVoxels) .* cellScale));
    detParams.houghSuppressionSize = max(1, round(double(detParams.houghSuppressionSize) .* cellScale));
    if isfield(fineFacadeCfg, "lineDistanceThresholdVoxels")
        fineFacadeCfg.lineDistanceThresholdVoxels = detParams.lineDistanceThresholdVoxels;
    end
end

function detParams = buildFacadeDetectionParams(cfg)
% buildFacadeDetectionParams: Pack off-ground configuration fields
% into the detectFacadeLines parameter struct.
%
% Input:
%   cfg: struct from offGroundFeatureConfig
%
% Output:
%   detParams: struct passed to detectFacadeLines
    detParams = struct();
    detParams.scoreGamma = max(eps, double(cfg.scoreGamma));
    detParams.supportGammaMin = max(0, double(cfg.supportGammaMin));
    detParams.supportQuantile = min(max(double(cfg.supportQuantile), 0), 1);
    detParams.supportAbs = max(0, double(cfg.supportAbs));
    detParams.maxManifoldWeight = max(0, double(cfg.maxManifoldWeight));
    detParams.useLogAccumulator = logical(cfg.useLogAccumulator);
    detParams.houghLineWeightExponent = 1.0;
    if isfield(cfg, "facadeHoughLineExponent") && isscalar(cfg.facadeHoughLineExponent) && isfinite(cfg.facadeHoughLineExponent)
        detParams.houghLineWeightExponent = max(0, double(cfg.facadeHoughLineExponent));
    end
    detParams.houghEnergyWeightExponent = 1.0;
    if isfield(cfg, "facadeHoughEnergyExponent") && isscalar(cfg.facadeHoughEnergyExponent) && isfinite(cfg.facadeHoughEnergyExponent)
        detParams.houghEnergyWeightExponent = max(0, double(cfg.facadeHoughEnergyExponent));
    end
    detParams.houghVoteWeightQuantile = 0.95;
    if isfield(cfg, "facadeHoughVoteWeightQuantile") && isscalar(cfg.facadeHoughVoteWeightQuantile) && isfinite(cfg.facadeHoughVoteWeightQuantile)
        detParams.houghVoteWeightQuantile = min(max(double(cfg.facadeHoughVoteWeightQuantile), eps), 1);
    end
    detParams.houghPeakThresholdRatio = max(0, double(cfg.houghPeakThresholdRatio));
    detParams.houghPeakQuantizationScale = max(1, round(double(cfg.houghPeakQuantizationScale)));
    detParams.maxHoughPeaks = max(1, round(double(cfg.maxHoughPeaks)));
    detParams.houghSuppressionSize = normalizeHoughSuppressionSize(cfg.houghSuppressionSize);
    detParams.lineDistanceThresholdVoxels = max(0, double(cfg.lineDistanceThresholdVoxels));
    detParams.facadeLineFillGapVoxels = max(1, round(double(cfg.facadeLineFillGapVoxels)));
    detParams.facadeLineMinLengthVoxels = max(6, round(double(cfg.facadeLineMinLengthVoxels)));
    detParams.splitValidationEnabled = logical(cfg.splitValidationEnabled);
    detParams.minAssignedPixels = max(1, round(double(cfg.minAssignedPixels)));
    detParams.minPeakLengthMeters = max(0, double(cfg.minPeakLengthMeters));
    detParams.mergeThetaTolDeg = max(0, double(cfg.mergeThetaTolDeg));
    detParams.mergeRhoTolMeters = max(0, double(cfg.mergeRhoTolMeters));
    detParams.maxOutputLines = max(0, round(double(cfg.maxOutputLines)));
    detParams.fitWeightExponent = max(0, double(cfg.fitWeightExponent));
    detParams.thetaClusterTolDeg = max(0, double(cfg.thetaClusterTolDeg));
    detParams.maxThetaClusters = max(0, round(double(cfg.maxThetaClusters)));
    detParams.maxLinesPerThetaCluster = max(0, round(double(cfg.maxLinesPerThetaCluster)));
    detParams.facadeMaskUseKeptPeaks = logical(cfg.facadeMaskUseKeptPeaks);
    thetaVals = -90:double(cfg.thetaResolutionDeg):89.5;
    if isfield(cfg, "thetaPriorRangeDeg") && ~isempty(cfg.thetaPriorRangeDeg)
        prior = double(cfg.thetaPriorRangeDeg(:).');
        if numel(prior) == 2 && all(isfinite(prior))
            tMin = max(-90, min(90, prior(1)));
            tMax = max(-90, min(90, prior(2)));
            if tMin <= tMax
                thetaVals = thetaVals(thetaVals >= tMin & thetaVals <= tMax);
            else
                thetaVals = thetaVals(thetaVals >= tMin | thetaVals <= tMax);
            end
            if isempty(thetaVals)
                thetaVals = -90:double(cfg.thetaResolutionDeg):89.5;
            end
        end
    end
    detParams.thetaVals = thetaVals;
    detParams.orientationToleranceDeg = 25.0;
    if isfield(cfg, "orientationToleranceDeg") && isscalar(cfg.orientationToleranceDeg) && isfinite(cfg.orientationToleranceDeg)
        detParams.orientationToleranceDeg = min(max(0, double(cfg.orientationToleranceDeg)), 25.0);
    end
end

function hood = normalizeHoughSuppressionSize(hood)
% normalizeHoughSuppressionSize: Ensure Hough non-maximum suppression size
% is a 1x2 vector of odd positive integers.
%
% Input:
%   hood: numeric vector, expected [rhoN thetaN]
%
% Output:
%   hood: [1 x 2] double vector of odd positive integers
    hood = double(hood(:).');
    if numel(hood) ~= 2
        hood = [21, 11];
    end

    hood = max(1, round(hood));
    hood = hood + mod(hood + 1, 2);
end

function debug = buildDisabledFacadeDetectorDiagnostics(mapSize)
% buildDisabledFacadeDetectorDiagnostics: Build a stable detector-debug
% placeholder for scenarios where facade detection is intentionally
% disabled, preserving downstream debug field availability.
%
% Input:
%   mapSize: [1 x 2] size of the fine-column map [Ny Nx]
%
% Output:
%   debug: struct matching the facade detector debug shape with empty masks
    if numel(mapSize) < 2
        mapSize = [0, 0];
    end
    debug = struct("scoreMap", zeros(mapSize, "single"), ...
        "supportMask", false(mapSize), "facadeMask", false(mapSize), ...
        "assignmentMap", zeros(mapSize, "uint16"), ...
        "lineSegments", struct("point1", {}, "point2", {}, "theta", {}, "rho", {}), ...
        "voteWeightMap", zeros(mapSize, "single"), ...
        "fittedLines", zeros(0, 4), "peakStats", struct("peakId", zeros(0, 1), ...
        "numPixels", zeros(0, 1), "weightSum", zeros(0, 1), "lengthMeters", zeros(0, 1), ...
        "linearity", zeros(0, 1), "residualMedianMeters", zeros(0, 1)), ...
        "fittedLinesRaw", zeros(0, 4), "peakStatsRaw", struct("peakId", zeros(0, 1), ...
        "numPixels", zeros(0, 1), "weightSum", zeros(0, 1), "lengthMeters", zeros(0, 1), ...
        "linearity", zeros(0, 1), "residualMedianMeters", zeros(0, 1)), ...
        "selectedIdx", zeros(0, 1), "peaks", zeros(0, 2), "peakRho", zeros(0, 1), ...
        "peakTheta", zeros(0, 1), "peakScores", zeros(0, 1), ...
        "enabled", false, "reason", "facadeDetectionDisabled");
end
