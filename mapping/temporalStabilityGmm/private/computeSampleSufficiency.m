function sampleSufficiency = computeSampleSufficiency(totalSaturatedFrameSupport, params)
% computeSampleSufficiency: Compute the repeated-frame sample sufficiency
% multiplier from total saturated foreground frame support using the configured
% prior scale. This term measures enough repeated support evidence and is
% distinct from concentration-aware temporal diversity.
%
% Input:
%   totalSaturatedFrameSupport: scalar sum of saturated per-frame support
%   params: class parameter struct with effectiveFramePrior
%
% Output:
%   sampleSufficiency: scalar support amplitude multiplier in [0, 1]
    if ~isscalar(totalSaturatedFrameSupport) || ~isfinite(totalSaturatedFrameSupport) || totalSaturatedFrameSupport < 0
        error("buildTemporalStabilityGmmMap:InvalidSampleSufficiencyInput", ...
            "Sample sufficiency requires finite nonnegative total saturated frame support.");
    end
    sampleSufficiency = clipUnit(1 - exp(-totalSaturatedFrameSupport ./ params.effectiveFramePrior));
end
