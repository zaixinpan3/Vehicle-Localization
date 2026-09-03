function seeds = initializeGaussianMixtureSeeds(points, patchLocalIndices, patchComponents, stability, resampleCounts, params)
% initializeGaussianMixtureSeeds: Select and weight patch-initialized Gaussian
% components before standard EM. Patch geometry and temporal reliability may
% influence initialization and model order, but these seeds do not constrain
% later ordinary EM updates.
%
% Input:
%   points: [N x 2] class-local BEV feature point coordinates
%   patchLocalIndices: cell array of buildable patch point indices
%   patchComponents: struct array of patch-initialized Gaussian components
%   stability: scalar struct with patch reliability scores
%   resampleCounts: [N x 1] Bernoulli sample count diagnostics
%   params: validated class parameter struct with seed settings
%
% Output:
%   seeds: struct array of Gaussian mixture initialization components
    patchCount = numel(patchComponents);
    if patchCount < 1 || numel(patchLocalIndices) ~= patchCount || numel(stability.patchScores) ~= patchCount
        error("buildTemporalStabilityGmmMap:InvalidGmmSeedInput", ...
            "GMM seed construction requires matching nonempty patches, patch components, and patch scores.");
    end
    patchResampledMass = zeros(patchCount, 1);
    for patchIdx = 1:patchCount
        localIndices = patchLocalIndices{patchIdx}(:);
        patchResampledMass(patchIdx) = sum(resampleCounts(localIndices));
    end
    keepMask = stability.patchScores(:) >= params.minPatchSupportForGmmSeed & patchResampledMass > 0;
    if ~any(keepMask)
        sampledPatchMask = patchResampledMass > 0;
        if ~any(sampledPatchMask)
            error("buildTemporalStabilityGmmMap:NoResampledSeedPatch", ...
                "GMM seed construction requires at least one retained patch with resampled training mass.");
        end
        [~, bestPatchOffset] = max(stability.patchScores(sampledPatchMask));
        sampledPatchIdx = find(sampledPatchMask);
        keepMask(sampledPatchIdx(bestPatchOffset)) = true;
    end
    keptPatchIdx = find(keepMask);
    patchMass = patchResampledMass + eps;
    [~, orderIdx] = sort(patchMass(keptPatchIdx), "descend");
    keptPatchIdx = keptPatchIdx(orderIdx);
    maxSeedCount = min(numel(keptPatchIdx), max(1, nnz(resampleCounts)));
    seeds = patchComponents(keptPatchIdx(1:maxSeedCount));
    seedMass = patchMass(keptPatchIdx(1:maxSeedCount));
    if any(~isfinite(seedMass)) || any(seedMass <= 0) || sum(seedMass) <= 0
        error("buildTemporalStabilityGmmMap:InvalidGmmSeedMass", ...
            "Patch-based GMM seed masses must be finite positive values.");
    end
    mixtureWeights = seedMass ./ sum(seedMass);
    for componentIdx = 1:numel(seeds)
        seeds(componentIdx).mixtureWeight = mixtureWeights(componentIdx);
        seeds(componentIdx).initialMixtureWeight = mixtureWeights(componentIdx);
        seeds(componentIdx).emMixtureWeightBeforePruning = mixtureWeights(componentIdx);
        seeds(componentIdx).pointCount = size(points, 1) .* mixtureWeights(componentIdx);
    end
end
