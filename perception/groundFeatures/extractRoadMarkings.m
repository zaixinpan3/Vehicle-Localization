function [roadMarkingPointMask, reflectivityThreshold] = extractRoadMarkings(groundReflectivity, roadPointMask, cfg)
% extractRoadMarkings: Select high-reflectivity road-marking points on the
% detected road surface. The threshold is estimated relative to the
% current frame's road-surface reflectivity distribution and floored by a
% configured absolute minimum, so markings are points that are both
% relatively bright within the road and absolutely bright.
%
% Input:
%   groundReflectivity: [N x 1] reflectivity aligned with ground points
%   roadPointMask: [N x 1] logical road-surface point mask
%   cfg: struct from groundFeatureConfig().roadMarking
%
% Output:
%   roadMarkingPointMask: [N x 1] logical road-marking point mask
%   reflectivityThreshold: scalar reflectivity threshold used
    reflectivityThreshold = resolveRoadReflectivityThreshold(groundReflectivity, roadPointMask, cfg);
    roadMarkingPointMask = roadPointMask & isfinite(groundReflectivity) & groundReflectivity > reflectivityThreshold;
end

function threshold = resolveRoadReflectivityThreshold(groundReflectivity, groundRoadPointMask, cfg)
% resolveRoadReflectivityThreshold: Estimate a dynamic
% reflectivity threshold from only the current frame's road-surface points
% and apply a configured absolute floor so the pink overlay follows points
% that are both relatively high within the detected road distribution and
% above a minimum physically meaningful reflectivity value.
%
% Input:
%   groundReflectivity: [N x 1] numeric reflectivity values aligned with
%       ground points
%   groundRoadPointMask: [N x 1] logical mask selecting ground points that
%       belong to road cells
%   cfg: visualization config struct with relative reflectivity fields and
%       an optional absolute minimum reflectivity threshold
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
% computeQuantile: Estimate a scalar quantile by sorting finite
% samples and linearly interpolating between adjacent order statistics so
% relative reflectivity thresholding does not depend on an optional
% toolbox.
%
% Input:
%   values: numeric vector
%   q: scalar quantile in [0, 1]
%
% Output:
%   qValue: scalar quantile estimate, or NaN when no finite samples exist
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

    w = pos - lo;
    qValue = ((1 - w) .* values(lo)) + (w .* values(hi));
end
