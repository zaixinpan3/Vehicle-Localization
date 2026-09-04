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
