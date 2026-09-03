function saturatedFrameSupport = saturateFrameSupport(rawFrameSupportEvidence, params)
% saturateFrameSupport: Convert raw per-frame foreground evidence into
% bounded frame support so one dense frame cannot dominate repeated-frame map
% support purely through many feature anchors.
%
% Input:
%   rawFrameSupportEvidence: [B x 1] nonnegative raw foreground evidence
%   params: class parameter struct with frameCountSaturation
%
% Output:
%   saturatedFrameSupport: [B x 1] saturated per-frame support values
    if any(~isfinite(rawFrameSupportEvidence)) || any(rawFrameSupportEvidence < 0)
        error("buildTemporalStabilityGmmMap:InvalidFrameSupport", ...
            "Raw integrated frame support evidence must be finite and nonnegative.");
    end
    if isempty(params.frameCountSaturation)
        saturatedFrameSupport = rawFrameSupportEvidence;
    else
        saturatedFrameSupport = 1 - exp(-rawFrameSupportEvidence ./ params.frameCountSaturation);
    end
    if any(~isfinite(saturatedFrameSupport)) || any(saturatedFrameSupport < 0)
        error("buildTemporalStabilityGmmMap:InvalidFrameSupport", ...
            "Saturated integrated frame support must be finite and nonnegative.");
    end
end
