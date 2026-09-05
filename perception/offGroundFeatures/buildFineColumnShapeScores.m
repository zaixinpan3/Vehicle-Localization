function [pointScore, lineScore, normalOrientation, blobness] = buildFineColumnShapeScores(runLayerMap, occupiedMask, cfg, dx, dy)
% buildFineColumnShapeScores: Compute fine-column point-vs-line shape
% scores from directional point response and local weighted second-moment
% line response inside a configurable square neighborhood.
%
% Input:
%   runLayerMap: [Ny x Nx] single maximum contiguous z-layer count map
%   occupiedMask: [Ny x Nx] logical occupied fine-column mask
%   cfg: configuration struct with fine shape-score parameters
%   dx: scalar fine-cell x spacing in meters
%   dy: scalar fine-cell y spacing in meters
%
% Output:
%   pointScore, lineScore, normalOrientation, blobness: fine-column shape
%       maps consumed by facade detection and pole candidate scoring
    centerRun = single(runLayerMap);
    centerRun(~isfinite(centerRun)) = 0;
    if isfield(cfg, "useNativeKernels") && cfg.useNativeKernels
        parameters = [resolveFineShapeScoreNeighborhoodRadius(cfg), ...
            resolvePositiveScalar(dx, 1), resolvePositiveScalar(dy, 1), ...
            resolveFineShapeScoreWeightPower(cfg), resolveFineShapeScoreLinearityPower(cfg), ...
            resolveFineShapeScoreMinSeedCount(cfg), resolveFineShapeScoreSaturatedSeedCount(cfg)];
        [pointScore, lineScore, normalOrientation] = perceptionKernelsMex( ...
            'pillarShape', centerRun, logical(occupiedMask), parameters);
        blobness = pointScore;
        return;
    end
    [numRowsRun, numColsRun] = size(centerRun);
    rowIdx = 1:numRowsRun;
    colIdx = 1:numColsRun;
    shapeRadius = resolveFineShapeScoreNeighborhoodRadius(cfg);
    drop0 = zeros(size(centerRun), "single");
    drop90 = zeros(size(centerRun), "single");
    drop45 = zeros(size(centerRun), "single");
    drop135 = zeros(size(centerRun), "single");
    for radius = 1:shapeRadius
        leftIdx = max(colIdx - radius, 1);
        rightIdx = min(colIdx + radius, numColsRun);
        upIdx = max(rowIdx - radius, 1);
        downIdx = min(rowIdx + radius, numRowsRun);
        upRun = centerRun(upIdx, :);
        downRun = centerRun(downIdx, :);
        drop0 = max(drop0, max((single(2) .* centerRun) - (centerRun(:, leftIdx) + centerRun(:, rightIdx)), 0));
        drop90 = max(drop90, max((single(2) .* centerRun) - (upRun + downRun), 0));
        drop45 = max(drop45, max((single(2) .* centerRun) - (upRun(:, rightIdx) + downRun(:, leftIdx)), 0));
        drop135 = max(drop135, max((single(2) .* centerRun) - (upRun(:, leftIdx) + downRun(:, rightIdx)), 0));
    end

    dropStack = cat(3, drop0, drop90, drop45, drop135);
    [lambdaA, ~] = max(dropStack, [], 3);
    lambdaB = min(dropStack, [], 3);
    lambdaA(~occupiedMask) = 0;
    lambdaB(~occupiedMask) = 0;
    weakEnergyMask = occupiedMask & (lambdaA <= eps("single"));

    pointShape = lambdaB ./ (lambdaA + eps("single"));
    pointShape(~isfinite(pointShape)) = 0;
    pointScore = min(max(pointShape, 0), 1);
    pointScore(~occupiedMask) = 0;
    pointScore(weakEnergyMask) = 0;
    [lineScore, normalOrientation] = computeFineSecondMomentLineScore(centerRun, occupiedMask, cfg, dx, dy);
    blobness = pointScore;
    blobness(~occupiedMask) = 0;
end

function [lineScore, normalOrientation] = computeFineSecondMomentLineScore(runLayerMap, occupiedMask, cfg, dx, dy)
% computeFineSecondMomentLineScore: Estimate local line-likeness from
% weighted second moments of occupied fine-column run-layer evidence inside
% the configured square neighborhood.
%
% Input:
%   runLayerMap: [Ny x Nx] single maximum contiguous z-layer count map
%   occupiedMask: [Ny x Nx] logical occupied fine-column mask
%   cfg: off-ground processing configuration struct
%   dx: scalar fine-cell x spacing in meters
%   dy: scalar fine-cell y spacing in meters
%
% Output:
%   lineScore: [Ny x Nx] single weighted second-moment linearity score
%   normalOrientation: [Ny x Nx] single local line normal angle in degrees
    shapeRadius = resolveFineShapeScoreNeighborhoodRadius(cfg);
    dx = resolvePositiveScalar(dx, 1);
    dy = resolvePositiveScalar(dy, 1);
    weightMap = double(max(single(runLayerMap), 0)) .* double(logical(occupiedMask));
    weightPower = resolveFineShapeScoreWeightPower(cfg);
    if weightPower ~= 1
        weightMap = weightMap .^ weightPower;
    end
    weightMap(~isfinite(weightMap)) = 0;

    offsetRange = -shapeRadius:shapeRadius;
    [offsetCols, offsetRows] = meshgrid(offsetRange .* dx, offsetRange .* dy);
    supportKernel = ones(size(offsetCols));
    m00 = conv2(weightMap, supportKernel, "same");
    m10 = conv2(weightMap, rot90(offsetCols, 2), "same");
    m01 = conv2(weightMap, rot90(offsetRows, 2), "same");
    m20 = conv2(weightMap, rot90(offsetCols .* offsetCols, 2), "same");
    m02 = conv2(weightMap, rot90(offsetRows .* offsetRows, 2), "same");
    m11 = conv2(weightMap, rot90(offsetCols .* offsetRows, 2), "same");
    seedCount = conv2(double(weightMap > 0), supportKernel, "same");

    invM00 = 1 ./ max(m00, eps);
    muX = m10 .* invM00;
    muY = m01 .* invM00;
    cxx = max((m20 .* invM00) - (muX .* muX), 0);
    cyy = max((m02 .* invM00) - (muY .* muY), 0);
    cxy = (m11 .* invM00) - (muX .* muY);
    traceC = max(cxx + cyy, 0);
    deltaC = sqrt(max(((cxx - cyy) .* (cxx - cyy)) + (4 .* cxy .* cxy), 0));
    lambda1 = max(0.5 .* (traceC + deltaC), 0);
    lambda2 = max(min(0.5 .* (traceC - deltaC), lambda1), 0);
    anisotropy = (lambda1 - lambda2) ./ max(lambda1 + lambda2, eps);
    anisotropy = clamp01(anisotropy);

    seedGate = smoothStepMap(seedCount, resolveFineShapeScoreMinSeedCount(cfg), ...
        resolveFineShapeScoreSaturatedSeedCount(cfg));
    linearityPower = resolveFineShapeScoreLinearityPower(cfg);
    lineScore = clamp01((anisotropy .^ linearityPower) .* seedGate);
    momentValid = (m00 > eps) & (seedCount >= resolveFineShapeScoreMinSeedCount(cfg));
    lineScore(~occupiedMask | ~momentValid) = 0;

    thetaLine = 0.5 .* atan2(2 .* cxy, cxx - cyy);
    normalOrientation = single(mod((thetaLine .* (180 ./ pi)) + 180, 180) - 90);
    normalOrientation(~occupiedMask | ~momentValid | (lineScore <= eps("single"))) = NaN;
    normalOrientation(~isfinite(normalOrientation)) = NaN;
    lineScore = single(lineScore);
end

function shapeRadius = resolveFineShapeScoreNeighborhoodRadius(cfg)
% resolveFineShapeScoreNeighborhoodRadius: Read the configured local
% radius used for fine-column point-vs-line local shape scoring.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   shapeRadius: scalar positive integer radius in fine XY cells
    shapeRadius = 1;
    if isstruct(cfg) && isfield(cfg, "fineShapeScoreNeighborhoodRadiusCells") && ...
            isscalar(cfg.fineShapeScoreNeighborhoodRadiusCells) && ...
            isfinite(cfg.fineShapeScoreNeighborhoodRadiusCells)
        shapeRadius = max(1, round(double(cfg.fineShapeScoreNeighborhoodRadiusCells)));
    end
end

function value = resolvePositiveScalar(value, defaultValue)
% resolvePositiveScalar: Return a finite positive scalar parameter or
% a supplied positive fallback value when the input is invalid.
%
% Input:
%   value: numeric candidate scalar
%   defaultValue: numeric fallback scalar
%
% Output:
%   value: finite positive scalar
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        value = defaultValue;
    end
    value = double(value);
end

function weightPower = resolveFineShapeScoreWeightPower(cfg)
% resolveFineShapeScoreWeightPower: Read the configured exponent used
% to sharpen run-layer weights in local second-moment line scoring.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   weightPower: scalar positive exponent
    weightPower = 1.0;
    if isstruct(cfg) && isfield(cfg, "fineShapeScoreWeightPower") && ...
            isscalar(cfg.fineShapeScoreWeightPower) && isfinite(cfg.fineShapeScoreWeightPower)
        weightPower = max(double(cfg.fineShapeScoreWeightPower), eps);
    end
end

function linearityPower = resolveFineShapeScoreLinearityPower(cfg)
% resolveFineShapeScoreLinearityPower: Read the configured exponent
% applied to local second-moment anisotropy before seed gating.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   linearityPower: scalar positive exponent
    linearityPower = 1.0;
    if isstruct(cfg) && isfield(cfg, "fineShapeScoreLinearityPower") && ...
            isscalar(cfg.fineShapeScoreLinearityPower) && isfinite(cfg.fineShapeScoreLinearityPower)
        linearityPower = max(double(cfg.fineShapeScoreLinearityPower), eps);
    end
end

function minSeedCount = resolveFineShapeScoreMinSeedCount(cfg)
% resolveFineShapeScoreMinSeedCount: Read the minimum occupied
% fine-column count required inside the line-score moment window.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   minSeedCount: scalar nonnegative seed-count threshold
    minSeedCount = 3;
    if isstruct(cfg) && isfield(cfg, "fineShapeScoreMinSeedCount") && ...
            isscalar(cfg.fineShapeScoreMinSeedCount) && isfinite(cfg.fineShapeScoreMinSeedCount)
        minSeedCount = max(0, double(cfg.fineShapeScoreMinSeedCount));
    end
end

function saturatedSeedCount = resolveFineShapeScoreSaturatedSeedCount(cfg)
% resolveFineShapeScoreSaturatedSeedCount: Read the occupied
% fine-column count at which local line-score support reaches full weight.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   saturatedSeedCount: scalar nonnegative seed-count threshold
    minSeedCount = resolveFineShapeScoreMinSeedCount(cfg);
    saturatedSeedCount = 5;
    if isstruct(cfg) && isfield(cfg, "fineShapeScoreSaturatedSeedCount") && ...
            isscalar(cfg.fineShapeScoreSaturatedSeedCount) && isfinite(cfg.fineShapeScoreSaturatedSeedCount)
        saturatedSeedCount = max(minSeedCount, double(cfg.fineShapeScoreSaturatedSeedCount));
    end
end

function smoothMap = smoothStepMap(valueMap, lowerValue, upperValue)
% smoothStepMap: Apply a cubic smooth-step transfer function
% that is zero at and below lowerValue and one at and above upperValue.
%
% Input:
%   valueMap: numeric map or scalar values to transform
%   lowerValue: scalar lower transition boundary
%   upperValue: scalar upper transition boundary
%
% Output:
%   smoothMap: double map with values clipped to [0, 1]
    valueMap = double(valueMap);
    lowerValue = double(lowerValue);
    upperValue = double(upperValue);
    if upperValue <= lowerValue
        smoothMap = double(valueMap >= upperValue);
        smoothMap(~isfinite(valueMap)) = 0;
        return;
    end
    t = (valueMap - lowerValue) ./ (upperValue - lowerValue);
    t = min(max(t, 0), 1);
    smoothMap = (3 .* t .* t) - (2 .* t .* t .* t);
    smoothMap(~isfinite(smoothMap)) = 0;
end
