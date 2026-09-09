function facade = extractFacadeFeatures(columnMaps, cfg)
% extractFacadeFeatures: Detect facade lines from whole-column XY evidence.
% Coarse and fine callers supply their own supportEvidence. Per-point plane
% validation belongs to validateFacadeCandidatePoints in offline perception.
    [detParams, assignmentCfg] = resolveFacadeDetectionParams(cfg, columnMaps.dx, columnMaps.dy);
    [detectedLinesRaw, detectorDiagnostics] = detectFacadeLines(columnMaps.lineScore, columnMaps.normalOrientation, ...
        columnMaps.occupiedMask, columnMaps.supportEvidence, [], columnMaps.origin, columnMaps.dx, columnMaps.dy, detParams);
    detectedLines = detectedLinesRaw;
    detectorMask = detectorDiagnostics.facadeMask;
    [lineMap, mask] = assignFacadeColumnsToLines( ...
        detectorDiagnostics, detectedLinesRaw, detectedLines, detectorMask, ...
        columnMaps.xMap, columnMaps.yMap, columnMaps.dx, columnMaps.dy, assignmentCfg);

    facade = struct();
    facade.enabled = true;
    facade.mask = mask;
    facade.lineMap = lineMap;
    facade.pillarLinIdx = find(mask);
    facade.pillarLineIdx = double(lineMap(facade.pillarLinIdx));
    facade.detectedLines = detectedLines;
    facade.detectedLinesRaw = detectedLinesRaw;
    facade.detectorMask = detectorMask;
    facade.detectorDiagnostics = detectorDiagnostics;
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
%   cfg: struct from fineStructuralConfig
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
