function [detectedLines, debug] = detectFacadeLines(lineScore, normalOrientation, occupiedMask, voteWeightImage, ~, origin, dx, dy, detParams)
% detectFacadeLines: Detect facade lines by Hough voting + pixel
% assignment and direct peak-segment projection extent.
%
% Input:
%   lineScore: [M x N] numeric in [0, 1] facade line likelihood
%   normalOrientation: [M x N] numeric normal angles in degrees
%   occupiedMask: [M x N] logical occupancy mask for valid pillars
%   voteWeightImage: [M x N] numeric raw Hough vote evidence, such as
%       run-layer counts
%   origin: [1 x 2] numeric map origin in meters
%   dx: scalar voxel width in meters
%   dy: scalar voxel height in meters
%   detParams: struct with Hough and output filtering parameters
%
% Output:
%   detectedLines: [K x 4] double matrix of [x1 y1 x2 y2] in meters
%   debug: struct with fields scoreMap, supportMask, facadeMask,
%       assignmentMap, lineSegments, and peakStats
    [detectedLines, debug] = detectFacadeLinesDirect(lineScore, normalOrientation, occupiedMask, voteWeightImage, origin, dx, dy, detParams);
end

function [detectedLines, debug] = detectFacadeLinesDirect(lineScore, normalOrientation, occupiedMask, voteWeightImage, origin, dx, dy, detParams)
% detectFacadeLinesDirect: Detect facade lines using oriented weighted
% Hough peaks, assign support pixels to peaks, and convert each retained
% peak to a world-frame segment by projection extent.
%
% Input:
%   lineScore: [M x N] numeric in [0, 1] facade line likelihood
%   normalOrientation: [M x N] numeric normal angles in degrees
%   occupiedMask: [M x N] logical occupancy mask for valid pillars
%   voteWeightImage: [M x N] numeric raw Hough vote evidence, such as
%       run-layer counts
%   origin: [1 x 2] numeric map origin in meters
%   dx: scalar voxel width in meters
%   dy: scalar voxel height in meters
%   detParams: struct with fields for score gating, Hough peaks, and
%       line output filtering
%
% Output:
%   detectedLines: [K x 4] double matrix of [x1 y1 x2 y2] in meters
%   debug: struct with score/support/assignment maps and retained peak stats
    lineScore = single(lineScore);
    normalOrientation = single(normalOrientation);
    occupiedMask = logical(occupiedMask);
    voteWeightImage = single(voteWeightImage);
    origin = double(origin(:).');
    dx = double(dx);
    dy = double(dy);

    scoreBase = min(max(lineScore, 0), 1);
    scoreMap = scoreBase;
    scoreMap(~occupiedMask) = 0;
    scoreMap(~isfinite(scoreMap)) = 0;

    if isempty(voteWeightImage) || ~isequal(size(voteWeightImage), size(scoreMap))
        voteWeightImage = scoreMap;
    end
    baseSupportMask = occupiedMask & isfinite(normalOrientation) & (scoreMap > 0);
    scoreVals = scoreMap(baseSupportMask);
    scoreVals = scoreVals(isfinite(scoreVals) & (scoreVals > 0));

    supportAbs = 0;
    if isfield(detParams, "supportAbs") && ~isempty(detParams.supportAbs) && isfinite(detParams.supportAbs)
        supportAbs = max(0, double(detParams.supportAbs));
    end
    supportQuantile = 0;
    if isfield(detParams, "supportQuantile") && ~isempty(detParams.supportQuantile) && isfinite(detParams.supportQuantile)
        supportQuantile = min(max(double(detParams.supportQuantile), 0), 1);
    end
    supportGammaMin = 0;
    if isfield(detParams, "supportGammaMin") && ~isempty(detParams.supportGammaMin) && isfinite(detParams.supportGammaMin)
        supportGammaMin = max(0, double(detParams.supportGammaMin));
    end
    supportThreshold = supportAbs;
    if ~isempty(scoreVals)
        if supportQuantile > 0
            qVal = computeQuantile(double(scoreVals), supportQuantile);
            if isfinite(qVal)
                supportThreshold = max(supportThreshold, qVal);
            end
        end
        if supportGammaMin > 0
            supportThreshold = max(supportThreshold, supportGammaMin * max(double(scoreVals)));
        end
    end
    supportMask = baseSupportMask & (scoreMap >= single(supportThreshold));
    voteWeightMap = normalizeHoughVoteWeightMap(voteWeightImage, supportMask, detParams.houghVoteWeightQuantile);
    scoreVals = scoreMap(supportMask);
    scoreVals = scoreVals(isfinite(scoreVals) & (scoreVals > 0));

    emptyPeakStats = struct("peakId", zeros(0, 1), "numPixels", zeros(0, 1), "weightSum", zeros(0, 1), "lengthMeters", zeros(0, 1), ...
        "linearity", zeros(0, 1), "residualMedianMeters", zeros(0, 1));
    if isempty(scoreVals)
        detectedLines = zeros(0, 4);
        debug = struct("scoreMap", scoreMap, "supportMask", false(size(scoreMap)), "facadeMask", false(size(scoreMap)), ...
            "assignmentMap", zeros(size(scoreMap), "uint16"), "lineSegments", struct("point1", {}, "point2", {}, "theta", {}, "rho", {}), ...
            "fittedLines", detectedLines, "peakStats", emptyPeakStats, "fittedLinesRaw", detectedLines, "peakStatsRaw", emptyPeakStats, ...
            "voteWeightMap", voteWeightMap, ...
            "selectedIdx", zeros(0, 1), "peaks", zeros(0, 2), "peakRho", zeros(0, 1), "peakTheta", zeros(0, 1), "peakScores", zeros(0, 1));
        return;
    end

    thetaVals = double(detParams.thetaVals);
    orientationToleranceDeg = double(detParams.orientationToleranceDeg);
    energyWeightExponent = 1.0;
    if isfield(detParams, "houghEnergyWeightExponent") && ~isempty(detParams.houghEnergyWeightExponent) && isfinite(detParams.houghEnergyWeightExponent)
        energyWeightExponent = max(0, double(detParams.houghEnergyWeightExponent));
    end
    scoreEvidence = clamp01(double(scoreMap));
    energyEvidence = clamp01(double(voteWeightMap));
    weightVotes = scoreEvidence .* (energyEvidence .^ energyWeightExponent);
    weightVotes = single(clamp01(weightVotes));
    weightVotes(~supportMask) = 0;
    weightVotes(~isfinite(normalOrientation)) = 0;

    peaks = zeros(0, 2);
    peakRho = zeros(0, 1);
    peakTheta = zeros(0, 1);
    peakScores = zeros(0, 1);
    assignmentMap = zeros(size(supportMask), "uint16");
    facadeMask = false(size(supportMask));

    if any(supportMask(:))
        theta = thetaVals(isfinite(thetaVals));
        theta = theta(:).';
        if isempty(theta)
            theta = -90:1:89;
        end

        [supportRows, supportCols] = find(supportMask);
        [rhoMinBound, rhoMaxBound] = computeRhoBoundsFromSupport(supportRows, supportCols, theta, size(supportMask));
        rho = rhoMinBound:rhoMaxBound;
        H = weightedHoughAccumulator(weightVotes, normalOrientation, theta, rho, orientationToleranceDeg);
        Hpos = max(H, 0);

        if logical(detParams.useLogAccumulator)
            HpeakBase = log1p(Hpos);
        else
            HpeakBase = Hpos;
        end
        HpeakBase = HpeakBase .^ 2.0;

        HForPeaks = double(round(HpeakBase * double(detParams.houghPeakQuantizationScale)));
        HForPeaks(~isfinite(HForPeaks)) = 0;
        Hmax = double(max(HForPeaks(:)));
        if isfinite(Hmax) && Hmax > 0
            houghNHoodSize = double(detParams.houghSuppressionSize);
            houghNHoodSize = min(houghNHoodSize, size(HForPeaks));
            houghNHoodSize = max(1, round(houghNHoodSize));
            houghNHoodSize = houghNHoodSize - mod(houghNHoodSize + 1, 2);
            peakThresholdRatio = max(0, double(detParams.houghPeakThresholdRatio));
            peakThreshold = max(0, peakThresholdRatio * Hmax);
            peaks = selectHoughPeaksMorphological(HForPeaks, double(detParams.maxHoughPeaks), houghNHoodSize, peakThreshold);

            if ~isempty(peaks)
                peakRho = double(rho(peaks(:, 1)));
                peakTheta = double(theta(peaks(:, 2)));
                numRho = size(H, 1);
                peakLin = peaks(:, 1) + (peaks(:, 2) - 1) * numRho;
                peakScores = double(Hpos(peakLin));
                assignmentMap = assignPixelsToHoughPeaks(peakRho, peakTheta, peakScores, supportMask, double(detParams.lineDistanceThresholdVoxels));
                facadeMask = assignmentMap > 0;
            end
        end
    end

    minAssignedPixels = 10;
    if isfield(detParams, "minAssignedPixels") && ~isempty(detParams.minAssignedPixels) && isfinite(detParams.minAssignedPixels)
        minAssignedPixels = max(1, round(double(detParams.minAssignedPixels)));
    end

    minPeakLengthMeters = 0;
    if isfield(detParams, "minPeakLengthMeters") && ~isempty(detParams.minPeakLengthMeters) && isfinite(detParams.minPeakLengthMeters)
        minPeakLengthMeters = max(0, double(detParams.minPeakLengthMeters));
    end

    maxOutputLines = Inf;
    if isfield(detParams, "maxOutputLines") && ~isempty(detParams.maxOutputLines) && isfinite(detParams.maxOutputLines)
        maxOutputLines = double(detParams.maxOutputLines);
        if maxOutputLines <= 0
            maxOutputLines = Inf;
        else
            maxOutputLines = max(1, round(maxOutputLines));
        end
    end

    facadeMaskUseKeptPeaks = false;
    if isfield(detParams, "facadeMaskUseKeptPeaks") && ~isempty(detParams.facadeMaskUseKeptPeaks)
        facadeMaskUseKeptPeaks = logical(detParams.facadeMaskUseKeptPeaks);
    end

    [detectedLines, peakStats] = buildPeakExtentLinesFromAssignments(assignmentMap, peakRho, peakTheta, peakScores, origin, dx, dy, ...
        minAssignedPixels, minPeakLengthMeters, maxOutputLines);
    selectedIdx = (1:size(detectedLines, 1)).';

    if facadeMaskUseKeptPeaks && any(assignmentMap(:) > 0)
        keptPeakIds = unique(double(peakStats.peakId(:)));
        keptPeakIds = keptPeakIds(isfinite(keptPeakIds) & (keptPeakIds > 0));
        if isempty(keptPeakIds)
            facadeMask = false(size(assignmentMap));
        else
            facadeMask = ismember(double(assignmentMap), keptPeakIds);
        end
    end

    debug = struct("scoreMap", scoreMap, "supportMask", supportMask, "facadeMask", facadeMask, ...
        "assignmentMap", assignmentMap, "lineSegments", struct("point1", {}, "point2", {}, "theta", {}, "rho", {}), ...
        "voteWeightMap", voteWeightMap, ...
        "fittedLines", detectedLines, "peakStats", peakStats, ...
        "fittedLinesRaw", detectedLines, "peakStatsRaw", peakStats, ...
        "selectedIdx", selectedIdx, "peaks", peaks, "peakRho", peakRho, "peakTheta", peakTheta, "peakScores", peakScores);
end

function H = weightedHoughAccumulator(weightImage, normalOrientation, theta, rho, orientationToleranceDeg)
% weightedHoughAccumulator: Build an oriented, weighted Hough accumulator
% by restricting each pixel's votes to theta bins near the pixel's local
% normal orientation, using 180-degree periodic angular distance and a
% hard angular support window only.
%
% Input:
%   weightImage: [M x N] numeric image of nonnegative weights
%   normalOrientation: [M x N] numeric map of normal angles in degrees, expected in [-90, 90]
%   theta: [1 x Nt] double vector of angles in degrees
%   rho: [1 x Nr] double vector of rho bin centers
%   orientationToleranceDeg: scalar nonnegative tolerance window in degrees
%
% Output:
%   H: [Nr x Nt] single weighted Hough accumulator
    weightImage = single(weightImage);
    normalOrientation = single(normalOrientation);
    theta = single(theta(:).');
    rho = single(rho(:));
    orientationToleranceDeg = single(orientationToleranceDeg);

    numRho = numel(rho);
    numTheta = numel(theta);
    H = zeros(numRho, numTheta, "single");
    if numRho == 0 || numTheta == 0
        return;
    end

    if ~isequal(size(weightImage), size(normalOrientation))
        return;
    end

    if ~isfinite(orientationToleranceDeg) || orientationToleranceDeg < 0
        orientationToleranceDeg = 0;
    end

    [rows, cols, weights] = find(weightImage);
    if isempty(weights)
        return;
    end
    phi = normalOrientation(sub2ind(size(normalOrientation), rows, cols));
    valid = isfinite(weights) & (weights > 0) & isfinite(phi);
    if ~any(valid)
        return;
    end
    rows = single(rows(valid));
    cols = single(cols(valid));
    weights = single(weights(valid));
    phi = single(phi(valid));
    phi = mod(phi + 90, 180) - 90;

    rhoMin = rho(1);
    if numRho > 1
        rhoStep = rho(2) - rho(1);
    else
        rhoStep = single(1);
    end

    cosTheta = cosd(theta);
    sinTheta = sind(theta);
    thetaStart = theta(1);
    if numTheta > 1
        thetaStep = abs(theta(2) - theta(1));
    else
        thetaStep = single(1);
    end
    if ~isfinite(thetaStep) || thetaStep <= 0
        thetaStep = single(1);
    end

    orientationToleranceDeg = min(orientationToleranceDeg, single(90));
    kMax = ceil(double(orientationToleranceDeg ./ thetaStep));
    kMax = min(kMax, floor((numTheta - 1) ./ 2));
    thetaOffsets = double(-kMax:kMax);

    centerThetaIdx = round((phi - thetaStart) ./ thetaStep) + 1;
    centerThetaIdx = mod(centerThetaIdx - 1, numTheta) + 1;
    centerThetaIdx = double(centerThetaIdx(:));

    thetaIdxMat = mod(centerThetaIdx + thetaOffsets - 1, numTheta) + 1;
    thetaCandMat = theta(thetaIdxMat);
    angleDiffMat = mod(thetaCandMat - phi + 90, 180) - 90;
    inWindowMat = abs(angleDiffMat) <= orientationToleranceDeg;
    if ~any(inWindowMat(:))
        return;
    end

    cosMat = cosTheta(thetaIdxMat);
    sinMat = sinTheta(thetaIdxMat);
    rhoValsMat = cols .* cosMat + rows .* sinMat;
    rhoIdxMat = round((rhoValsMat - rhoMin) ./ rhoStep) + 1;

    voteValid = inWindowMat & (rhoIdxMat >= 1) & (rhoIdxMat <= numRho) & isfinite(rhoIdxMat);
    validLin = find(voteValid);
    if isempty(validLin)
        return;
    end

    numSupport = size(thetaIdxMat, 1);
    rowIdx = mod(validLin - 1, numSupport) + 1;
    thetaIdxValid = double(thetaIdxMat(validLin));
    rhoIdxValid = double(rhoIdxMat(validLin));
    voteWeights = double(weights(rowIdx));
    linIdx = rhoIdxValid + (thetaIdxValid - 1) .* numRho;
    Hvec = accumarray(linIdx(:), voteWeights(:), [numRho * numTheta, 1], @sum, 0);
    H = reshape(single(Hvec), numRho, numTheta);
end

function voteWeightMap = normalizeHoughVoteWeightMap(voteWeightImage, supportMask, upperQuantile)
% normalizeHoughVoteWeightMap: Normalize nonnegative Hough vote
% evidence to [0, 1] over the active support mask by dividing by a robust
% upper quantile so raw run-layer counts retain relative voting strength.
%
% Input:
%   voteWeightImage: [M x N] numeric raw Hough vote evidence
%   supportMask: [M x N] logical active facade-vote support
%   upperQuantile: scalar quantile in (0, 1] used as the normalization
%       denominator
%
% Output:
%   voteWeightMap: [M x N] single normalized Hough vote evidence
    voteWeightMap = zeros(size(voteWeightImage), "single");
    if isempty(voteWeightImage) || isempty(supportMask) || ~isequal(size(voteWeightImage), size(supportMask))
        return;
    end

    supportMask = logical(supportMask);
    values = double(voteWeightImage);
    values(~isfinite(values)) = 0;
    values = max(values, 0);
    supportValues = values(supportMask);
    supportValues = supportValues(isfinite(supportValues) & (supportValues > 0));
    if isempty(supportValues)
        return;
    end

    if nargin < 3 || ~isscalar(upperQuantile) || ~isfinite(upperQuantile)
        upperQuantile = 0.95;
    end
    upperQuantile = min(max(double(upperQuantile), eps), 1);
    scaleValue = computeQuantile(supportValues, upperQuantile);
    if ~(isfinite(scaleValue) && scaleValue > 0)
        scaleValue = max(supportValues);
    end
    if ~(isfinite(scaleValue) && scaleValue > 0)
        return;
    end

    normalizedValues = values ./ scaleValue;
    normalizedValues = clamp01(normalizedValues);
    normalizedValues(~supportMask) = 0;
    normalizedValues(~isfinite(normalizedValues)) = 0;
    voteWeightMap = single(normalizedValues);
end

function [rhoMinBound, rhoMaxBound] = computeRhoBoundsFromSupport(supportRows, supportCols, theta, supportMaskSize)
% computeRhoBoundsFromSupport: Compute a cropped rho search range for
% Hough voting from the support-mask bounding box projected onto all theta
% bins, with one-bin safety padding.
%
% Input:
%   supportRows: [K x 1] support pixel row indices
%   supportCols: [K x 1] support pixel column indices
%   theta: [1 x Nt] numeric theta bins in degrees
%   supportMaskSize: [1 x 2] size vector of support mask [rows cols]
%
% Output:
%   rhoMinBound: scalar integer lower rho bound in pixel units
%   rhoMaxBound: scalar integer upper rho bound in pixel units
    defaultRhoMax = ceil(hypot(double(supportMaskSize(1)), double(supportMaskSize(2))));
    rhoMinBound = -defaultRhoMax;
    rhoMaxBound = defaultRhoMax;
    if isempty(supportRows) || isempty(supportCols) || isempty(theta)
        return;
    end

    rMin = min(double(supportRows));
    rMax = max(double(supportRows));
    cMin = min(double(supportCols));
    cMax = max(double(supportCols));
    cornerX = [cMin; cMin; cMax; cMax];
    cornerY = [rMin; rMax; rMin; rMax];
    thetaVals = double(theta(:).');
    rhoCorners = cornerX .* cosd(thetaVals) + cornerY .* sind(thetaVals);
    rhoMinCandidate = floor(min(rhoCorners(:))) - 1;
    rhoMaxCandidate = ceil(max(rhoCorners(:))) + 1;
    if isfinite(rhoMinCandidate) && isfinite(rhoMaxCandidate) && (rhoMaxCandidate >= rhoMinCandidate)
        rhoMinBound = rhoMinCandidate;
        rhoMaxBound = rhoMaxCandidate;
    end
end

function peaks = selectHoughPeaksMorphological(HForPeaks, maxHoughPeaks, houghNHoodSize, peakThreshold)
% selectHoughPeaksMorphological: Extract top Hough peaks by applying
% morphological non-maximum suppression and thresholding, then sorting
% candidates by peak value and keeping at most maxHoughPeaks.
%
% Input:
%   HForPeaks: [Nr x Nt] numeric quantized Hough accumulator
%   maxHoughPeaks: scalar positive integer maximum output peak count
%   houghNHoodSize: [1 x 2] odd neighborhood size [rho theta]
%   peakThreshold: scalar threshold in accumulator units
%
% Output:
%   peaks: [K x 2] double peak indices [rhoIdx thetaIdx]
    peaks = zeros(0, 2);
    if isempty(HForPeaks)
        return;
    end

    maxHoughPeaks = max(1, round(double(maxHoughPeaks)));
    houghNHoodSize = double(houghNHoodSize(:).');
    if isscalar(houghNHoodSize)
        houghNHoodSize = [houghNHoodSize, houghNHoodSize];
    end
    houghNHoodSize = max(1, round(houghNHoodSize));
    houghNHoodSize = houghNHoodSize - mod(houghNHoodSize + 1, 2);
    houghNHoodSize = min(houghNHoodSize, size(HForPeaks));
    peakThreshold = max(0, double(peakThreshold));

    nhoodKernel = true(houghNHoodSize(1), houghNHoodSize(2));
    localMaxMask = HForPeaks == imdilate(HForPeaks, nhoodKernel);
    candidateMask = localMaxMask & isfinite(HForPeaks) & (HForPeaks >= peakThreshold);
    if ~any(candidateMask(:))
        return;
    end

    cc = bwconncomp(candidateMask, 8);
    if cc.NumObjects <= 0
        return;
    end

    candRows = zeros(cc.NumObjects, 1);
    candCols = zeros(cc.NumObjects, 1);
    candVals = zeros(cc.NumObjects, 1);
    for iComp = 1:cc.NumObjects
        idx = cc.PixelIdxList{iComp};
        [bestVal, bestLocalIdx] = max(double(HForPeaks(idx)));
        bestIdx = idx(bestLocalIdx(1));
        [r, c] = ind2sub(size(HForPeaks), bestIdx);
        candRows(iComp) = r;
        candCols(iComp) = c;
        candVals(iComp) = bestVal;
    end

    [~, order] = sort(candVals, "descend");
    keepCount = min(maxHoughPeaks, numel(order));
    if keepCount <= 0
        return;
    end
    keepOrder = order(1:keepCount);
    peaks = [candRows(keepOrder), candCols(keepOrder)];
end

function [linesMeters, peakStats] = buildPeakExtentLinesFromAssignments(assignmentMap, rhoPeaks, thetaPeaks, peakScores, origin, dx, dy, ...
        minAssignedPixels, minPeakLengthMeters, maxOutputLines)
% buildPeakExtentLinesFromAssignments: Convert Hough peak assignments to
% segments by fixed-peak tangent projection extent.
%
% Input:
%   assignmentMap: [M x N] uint16 peak assignment image
%   rhoPeaks: [P x 1] numeric Hough rho values in pixels
%   thetaPeaks: [P x 1] numeric Hough theta values in degrees
%   peakScores: [P x 1] numeric Hough peak strengths
%   origin: [1 x 2] map origin in meters
%   dx: scalar voxel width in meters
%   dy: scalar voxel height in meters
%   minAssignedPixels: scalar minimum assigned pixels per retained peak
%   minPeakLengthMeters: scalar minimum retained segment length
%   maxOutputLines: scalar max number of returned lines
%
% Output:
%   linesMeters: [K x 4] double [x1 y1 x2 y2] in meters
%   peakStats: struct aligned with linesMeters rows
    linesMeters = zeros(0, 4);
    peakStats = struct("peakId", zeros(0, 1), "numPixels", zeros(0, 1), "weightSum", zeros(0, 1), "lengthMeters", zeros(0, 1), ...
        "linearity", zeros(0, 1), "residualMedianMeters", zeros(0, 1));

    if isempty(assignmentMap) || isempty(rhoPeaks) || isempty(thetaPeaks)
        return;
    end

    rhoPeaks = double(rhoPeaks(:));
    thetaPeaks = double(thetaPeaks(:));
    peakScores = double(peakScores(:));
    numPeaks = numel(thetaPeaks);
    if numPeaks == 0 || numel(rhoPeaks) ~= numPeaks
        return;
    end
    if numel(peakScores) ~= numPeaks
        peakScores = ones(numPeaks, 1);
    end

    minAssignedPixels = max(1, round(double(minAssignedPixels)));
    minPeakLengthMeters = max(0, double(minPeakLengthMeters));
    if ~isfinite(maxOutputLines) || maxOutputLines <= 0
        maxOutputLines = Inf;
    else
        maxOutputLines = max(1, round(double(maxOutputLines)));
    end

    assignedLin = find(assignmentMap > 0);
    if isempty(assignedLin)
        return;
    end
    assignedPeak = double(assignmentMap(assignedLin));
    validAssigned = isfinite(assignedPeak) & (assignedPeak >= 1) & (assignedPeak <= numPeaks);
    if ~any(validAssigned)
        return;
    end

    assignedLin = assignedLin(validAssigned);
    assignedPeak = assignedPeak(validAssigned);
    [rows, cols] = ind2sub(size(assignmentMap), assignedLin);
    rows = double(rows);
    cols = double(cols);

    thetaAssigned = thetaPeaks(assignedPeak);
    rhoAssigned = rhoPeaks(assignedPeak);
    normalX = cosd(thetaAssigned);
    normalY = sind(thetaAssigned);
    tangentX = -normalY;
    tangentY = normalX;
    x0 = rhoAssigned .* normalX;
    y0 = rhoAssigned .* normalY;
    tVals = (cols - x0) .* tangentX + (rows - y0) .* tangentY;
    validT = isfinite(tVals) & isfinite(cols) & isfinite(rows);
    if ~any(validT)
        return;
    end

    assignedPeak = assignedPeak(validT);
    tVals = tVals(validT);
    numPixelsAll = accumarray(assignedPeak, 1, [numPeaks, 1], @sum, 0);
    tMinAll = accumarray(assignedPeak, tVals, [numPeaks, 1], @min, NaN);
    tMaxAll = accumarray(assignedPeak, tVals, [numPeaks, 1], @max, NaN);

    keepPeaks = (numPixelsAll >= minAssignedPixels) & isfinite(tMinAll) & isfinite(tMaxAll);
    peakId = find(keepPeaks);
    if isempty(peakId)
        return;
    end

    thetaKeep = thetaPeaks(peakId);
    rhoKeep = rhoPeaks(peakId);
    normalX = cosd(thetaKeep);
    normalY = sind(thetaKeep);
    tangentX = -normalY;
    tangentY = normalX;
    x0 = rhoKeep .* normalX;
    y0 = rhoKeep .* normalY;

    tMin = tMinAll(peakId);
    tMax = tMaxAll(peakId);
    x1Pix = x0 + tMin .* tangentX;
    y1Pix = y0 + tMin .* tangentY;
    x2Pix = x0 + tMax .* tangentX;
    y2Pix = y0 + tMax .* tangentY;

    x1 = origin(1) + (x1Pix - 0.5) * dx;
    y1 = origin(2) + (y1Pix - 0.5) * dy;
    x2 = origin(1) + (x2Pix - 0.5) * dx;
    y2 = origin(2) + (y2Pix - 0.5) * dy;
    lengthMeters = hypot(x2 - x1, y2 - y1);
    keepLength = isfinite(lengthMeters) & (lengthMeters >= minPeakLengthMeters);
    if ~any(keepLength)
        return;
    end

    peakId = peakId(keepLength);
    x1 = x1(keepLength);
    y1 = y1(keepLength);
    x2 = x2(keepLength);
    y2 = y2(keepLength);
    lengthMeters = lengthMeters(keepLength);
    numPixelsKeep = numPixelsAll(peakId);
    weightSum = peakScores(peakId);
    badWeight = ~isfinite(weightSum) | (weightSum <= 0);
    weightSum(badWeight) = numPixelsKeep(badWeight);

    [~, order] = sort(weightSum, "descend");
    if isfinite(maxOutputLines) && numel(order) > maxOutputLines
        order = order(1:maxOutputLines);
    end

    peakId = peakId(order);
    x1 = x1(order);
    y1 = y1(order);
    x2 = x2(order);
    y2 = y2(order);
    lengthMeters = lengthMeters(order);
    numPixelsKeep = numPixelsKeep(order);
    weightSum = weightSum(order);

    linesMeters = [x1, y1, x2, y2];
    peakStats = struct("peakId", peakId(:), "numPixels", double(numPixelsKeep(:)), "weightSum", double(weightSum(:)), "lengthMeters", double(lengthMeters(:)), ...
        "linearity", ones(numel(peakId), 1), "residualMedianMeters", zeros(numel(peakId), 1));
end

function qValue = computeQuantile(values, q)
% computeQuantile: Approximate a quantile using sorting and linear
% interpolation without requiring the Statistics Toolbox.
%
% Input:
%   values: numeric vector
%   q: scalar in [0, 1]
%
% Output:
%   qValue: scalar quantile estimate
    values = double(values(:));
    values = values(isfinite(values));
    if isempty(values)
        qValue = NaN;
        return;
    end

    q = double(q);
    if ~isfinite(q)
        q = 0.5;
    end
    q = min(max(q, 0), 1);

    values = sort(values);
    n = numel(values);
    if n == 1
        qValue = values(1);
        return;
    end

    pos = 1 + (n - 1) * q;
    lo = floor(pos);
    hi = ceil(pos);
    if lo == hi
        qValue = values(lo);
        return;
    end

    w = pos - lo;
    qValue = (1 - w) * values(lo) + w * values(hi);
end

function assignmentMap = assignPixelsToHoughPeaks(rhoPeaks, thetaPeaks, peakScores, supportMask, distanceThresholdVoxels)
% assignPixelsToHoughPeaks: Assign each supported image pixel to at most
% one Hough peak line by choosing the nearest qualifying peak under a
% perpendicular distance threshold, using peak score only as a tie-break.
%
% Input:
%   rhoPeaks: [P x 1] or [1 x P] numeric rho values from houghpeaks bins
%   thetaPeaks: [P x 1] or [1 x P] numeric theta values in degrees
%   peakScores: [P x 1] or [1 x P] numeric peak scores from the Hough accumulator
%   supportMask: [M x N] logical support mask in image coordinates
%   distanceThresholdVoxels: scalar maximum line distance in pixel/voxel units
%
% Output:
%   assignmentMap: [M x N] uint16 map with 0 for unassigned pixels or the
%       peak index (into rhoPeaks/thetaPeaks) that owns the pixel
    imageSize = size(supportMask);
    assignmentMap = zeros(imageSize, "uint16");
    if ~any(supportMask(:)) || isempty(rhoPeaks) || isempty(thetaPeaks) || ~isfinite(distanceThresholdVoxels)
        return;
    end

    supportLin = find(supportMask);
    [rows, cols] = ind2sub(imageSize, supportLin);
    x = double(cols);
    y = double(rows);

    rhoPeaks = double(rhoPeaks(:));
    thetaPeaks = double(thetaPeaks(:));
    peakScores = double(peakScores(:));
    numPeaks = numel(thetaPeaks);
    if numel(rhoPeaks) ~= numPeaks || numel(peakScores) ~= numPeaks
        return;
    end

    distanceThresholdVoxels = double(distanceThresholdVoxels);

    cosTheta = cosd(thetaPeaks.');
    sinTheta = sind(thetaPeaks.');
    rhoRow = rhoPeaks.';
    distances = abs(x .* cosTheta + y .* sinTheta - rhoRow);

    validDistances = distances;
    validDistances(validDistances > distanceThresholdVoxels) = inf;
    [minDistance, nearestIdx] = min(validDistances, [], 2);
    anyHit = isfinite(minDistance);

    assigned = zeros(numel(supportLin), 1, "uint16");
    if any(anyHit)
        tieMask = abs(validDistances(anyHit, :) - minDistance(anyHit)) <= eps(max(distanceThresholdVoxels, 1));
        tieScores = repmat(peakScores(:).', nnz(anyHit), 1);
        tieScores(~tieMask) = -inf;
        [~, bestTieIdx] = max(tieScores, [], 2);
        nearestIdx(anyHit) = bestTieIdx;
        assigned(anyHit) = uint16(nearestIdx(anyHit));
    end

    assignmentMap(supportLin) = assigned;
end
