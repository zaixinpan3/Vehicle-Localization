function [facadePillarLineMap, facadeMaskAssigned] = assignFacadeColumnsToLines(detDebug, detectedLinesRaw, detectedLinesFinal, facadeMask, xMap, yMap, dx, dy, cfg)
% assignFacadeColumnsToLines: Build facade pillar line ownership
% primarily from detector assignmentMap + kept peak ids, then map raw line
% ids to final merged line ids; fallback to geometric nearest-line
% assignment when detector ids are unavailable.
%
% Input:
%   detDebug: struct from detectFacadeLines with assignmentMap and peakStats
%   detectedLinesRaw: [Lr x 4] double raw fitted lines before final merge
%   detectedLinesFinal: [Lf x 4] double final output lines
%   facadeMask: [Ny x Nx] logical facade mask
%   xMap: [Ny x Nx] double pillar x coordinates in meters
%   yMap: [Ny x Nx] double pillar y coordinates in meters
%   dx: scalar pillar size in x (meters)
%   dy: scalar pillar size in y (meters)
%   cfg: struct with lineDistanceThresholdVoxels
%
% Output:
%   facadePillarLineMap: [Ny x Nx] uint16 line-id map
%   facadeMaskAssigned: [Ny x Nx] logical mask with valid line ownership
    facadePillarLineMap = zeros(size(facadeMask), "uint16");
    facadeMaskAssigned = false(size(facadeMask));
    if ~any(facadeMask(:))
        return;
    end

    if nargin >= 1 && isstruct(detDebug) && isfield(detDebug, "assignmentMap") && isfield(detDebug, "peakStats") && ...
            ~isempty(detDebug.assignmentMap) && ~isempty(detDebug.peakStats) && isfield(detDebug.peakStats, "peakId") && ...
            ~isempty(detDebug.peakStats.peakId)
        assignmentMap = double(detDebug.assignmentMap);
        peakIds = double(detDebug.peakStats.peakId(:));
        peakIds = peakIds(isfinite(peakIds) & (peakIds > 0));
        if ~isempty(peakIds)
            maxPeakId = max(max(assignmentMap(:)), max(peakIds));
            peakToRawLine = zeros(maxPeakId, 1);
            for k = 1:numel(detDebug.peakStats.peakId)
                pId = double(detDebug.peakStats.peakId(k));
                if isfinite(pId) && (pId >= 1) && (pId <= maxPeakId)
                    peakToRawLine(pId) = k;
                end
            end

            rawToFinal = mapRawLinesToFinal(detectedLinesRaw, detectedLinesFinal);
            mask = facadeMask & (assignmentMap > 0);
            lin = find(mask);
            if ~isempty(lin)
                assignedPeak = assignmentMap(lin);
                validPeak = assignedPeak >= 1 & assignedPeak <= numel(peakToRawLine);
                rawLineId = zeros(size(assignedPeak));
                rawLineId(validPeak) = peakToRawLine(assignedPeak(validPeak));
                validRaw = rawLineId >= 1 & rawLineId <= numel(rawToFinal);
                finalLineId = zeros(size(rawLineId));
                finalLineId(validRaw) = rawToFinal(rawLineId(validRaw));
                finalLineId(~isfinite(finalLineId) | (finalLineId < 0)) = 0;
                facadePillarLineMap(lin) = uint16(finalLineId);
                facadeMaskAssigned = facadePillarLineMap > 0;
                if any(facadeMaskAssigned(:))
                    return;
                end
            end
        end
    end

    [facadePillarLineMap, facadeMaskAssigned] = assignPillarsByLineDistance(facadeMask, detectedLinesFinal, xMap, yMap, dx, dy, cfg);
end

function rawToFinal = mapRawLinesToFinal(rawLines, finalLines)
% mapRawLinesToFinal: Map each raw detected line to the closest final
% output line in Hough space using angle + rho distance.
%
% Input:
%   rawLines: [Lr x 4] double raw lines
%   finalLines: [Lf x 4] double final merged lines
%
% Output:
%   rawToFinal: [Lr x 1] double final line id (0 when invalid/unmapped)
    numRaw = size(rawLines, 1);
    numFinal = size(finalLines, 1);
    rawToFinal = zeros(numRaw, 1);
    if numRaw == 0 || numFinal == 0
        return;
    end

    [rhoRaw, thetaRaw, validRaw] = segmentsToHoughParams(rawLines);
    [rhoFinal, thetaFinal, validFinal] = segmentsToHoughParams(finalLines);
    validFinalIdx = find(validFinal);
    if isempty(validFinalIdx)
        return;
    end

    for k = 1:numRaw
        if ~validRaw(k)
            continue;
        end
        angDiff = angleDifferenceDeg(thetaRaw(k), thetaFinal(validFinalIdx));
        rhoDiff = abs(rhoRaw(k) - rhoFinal(validFinalIdx));
        score = angDiff + 0.2 * rhoDiff;
        [~, bestPos] = min(score);
        rawToFinal(k) = validFinalIdx(bestPos);
    end
end

function [facadePillarLineMap, facadeMaskAssigned] = assignPillarsByLineDistance(facadeMask, detectedLines, xMap, yMap, dx, dy, cfg)
% assignPillarsByLineDistance: Fallback assignment that maps facade
% pillars to the nearest detected line under a metric threshold.
%
% Input:
%   facadeMask: [Ny x Nx] logical facade mask
%   detectedLines: [L x 4] double line segments in meters
%   xMap: [Ny x Nx] double pillar x coordinates in meters
%   yMap: [Ny x Nx] double pillar y coordinates in meters
%   dx: scalar pillar size in x (meters)
%   dy: scalar pillar size in y (meters)
%   cfg: struct with lineDistanceThresholdVoxels
%
% Output:
%   facadePillarLineMap: [Ny x Nx] uint16 line-id map
%   facadeMaskAssigned: [Ny x Nx] logical assigned facade mask
    facadePillarLineMap = zeros(size(facadeMask), "uint16");
    facadeMaskAssigned = false(size(facadeMask));
    if ~any(facadeMask(:)) || isempty(detectedLines)
        return;
    end

    [rho, theta, valid] = segmentsToHoughParams(detectedLines);
    if ~any(valid)
        return;
    end
    validLineIds = find(valid);
    rho = rho(valid);
    theta = theta(valid);

    lin = find(facadeMask);
    xVals = xMap(lin);
    yVals = yMap(lin);
    validPts = isfinite(xVals) & isfinite(yVals);
    if ~any(validPts)
        return;
    end

    xVals = xVals(validPts);
    yVals = yVals(validPts);
    distances = abs(xVals .* cosd(theta(:).') + yVals .* sind(theta(:).') - rho(:).');
    [minDist, bestPos] = min(distances, [], 2);

    tolMeters = max(double(dx), double(dy)) * max(1, double(cfg.lineDistanceThresholdVoxels));
    if ~isfinite(tolMeters) || tolMeters <= 0
        tolMeters = max(double(dx), double(dy));
    end
    keep = minDist <= tolMeters;

    assigned = zeros(numel(lin), 1, "uint16");
    pos = find(validPts);
    assigned(pos(keep)) = uint16(validLineIds(bestPos(keep)));
    facadePillarLineMap(lin) = assigned;
    facadeMaskAssigned = facadePillarLineMap > 0;
end

function d = angleDifferenceDeg(aDeg, bDeg)
% angleDifferenceDeg: Compute smallest absolute angular difference between
% undirected angles using 180-degree periodicity.
%
% Input:
%   aDeg: scalar or vector numeric angles in degrees
%   bDeg: scalar or vector numeric angles in degrees
%
% Output:
%   d: numeric array of absolute differences in degrees in [0, 90]
    aDeg = double(aDeg);
    bDeg = double(bDeg);
    d = abs(mod(aDeg - bDeg + 90, 180) - 90);
end
