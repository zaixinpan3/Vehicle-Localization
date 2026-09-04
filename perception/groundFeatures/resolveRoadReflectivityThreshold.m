function threshold = resolveRoadReflectivityThreshold(groundReflectivity, groundRoadPointMask, cfg)
% resolveRoadReflectivityThreshold: Estimate a dynamic reflectivity cutoff
% from the current frame's road-surface returns. The threshold combines a
% high quantile, a median absolute-deviation rule, and an absolute floor.
% Keeping threshold estimation separate lets the coarse branch classify
% only raster cells without first constructing a point-semantic mask.
%
% Input:
%   groundReflectivity: [N x 1] numeric reflectivity values aligned with
%       ground returns
%   groundRoadPointMask: [N x 1] logical mask selecting returns assigned
%       to road cells
%   cfg: road-marking configuration with relative and absolute thresholds
%
% Output:
%   threshold: scalar dynamic reflectivity cutoff; inf when no finite road
%       reflectivity samples are available
    reflectivityValues = double(groundReflectivity(:));
    roadMask = logical(groundRoadPointMask(:));
    assert(numel(reflectivityValues) == numel(roadMask), ...
        "groundReflectivity and groundRoadPointMask must have the same number of elements.");

    roadValues = reflectivityValues(roadMask & isfinite(reflectivityValues));
    if isempty(roadValues)
        threshold = inf;
        return;
    end

    quantileLevel = 0.90;
    if isfield(cfg, "roadReflectivityHighlightQuantile")
        quantileLevel = double(cfg.roadReflectivityHighlightQuantile);
        quantileLevel = quantileLevel(1);
    end
    if ~isfinite(quantileLevel)
        quantileLevel = 0.90;
    end
    quantileLevel = min(max(quantileLevel, 0), 1);

    madScale = 2.5;
    if isfield(cfg, "roadReflectivityHighlightMadScale")
        madScale = double(cfg.roadReflectivityHighlightMadScale);
        madScale = madScale(1);
    end
    if ~isfinite(madScale)
        madScale = 2.5;
    end
    madScale = max(madScale, 0);

    quantileThreshold = computeQuantile(roadValues, quantileLevel);
    medianValue = median(roadValues);
    medianAbsDeviation = median(abs(roadValues - medianValue));
    robustSpread = double(1.4826 .* medianAbsDeviation);
    robustSpread = robustSpread(1);
    if isfinite(robustSpread) && robustSpread > 0
        threshold = max(quantileThreshold, medianValue + madScale .* robustSpread);
    else
        threshold = quantileThreshold;
    end
    minimumThreshold = 0;
    if isfield(cfg, "roadReflectivityHighlightMinimum")
        minimumThreshold = double(cfg.roadReflectivityHighlightMinimum);
        minimumThreshold = minimumThreshold(1);
    end
    if ~isfinite(minimumThreshold)
        minimumThreshold = 0;
    end
    threshold = max(threshold, max(0, minimumThreshold));
    if ~isfinite(threshold)
        threshold = inf;
    end
end

function qValue = computeQuantile(values, q)
% computeQuantile: Interpolate a scalar quantile without an optional
% toolbox dependency.
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

    pos = 1 + (n - 1) .* q;
    lo = floor(pos);
    hi = ceil(pos);
    if lo == hi
        qValue = values(lo);
        return;
    end

    weight = pos - lo;
    qValue = ((1 - weight) .* values(lo)) + (weight .* values(hi));
end
