function seeds = reseedFromRetainedComponents(components)
% reseedFromRetainedComponents: Convert post-hoc retained components
% into a normalized initialization for the final ordinary Gaussian EM refit.
%
% Input:
%   components: struct array of retained Gaussian support components
%
% Output:
%   seeds: struct array with positive mixture weights summing to one
    if isempty(components)
        error("buildTemporalStabilityGmmMap:InvalidRefitSeeds", ...
            "Final standard GMM refit requires at least one retained component.");
    end
    seeds = components;
    mixtureWeights = [seeds.mixtureWeight].';
    if any(~isfinite(mixtureWeights)) || any(mixtureWeights <= 0) || sum(mixtureWeights) <= 0
        mixtureWeights = max([seeds.supportAmplitude].', eps);
    end
    mixtureWeights = mixtureWeights ./ sum(mixtureWeights);
    for componentIdx = 1:numel(seeds)
        seeds(componentIdx).mixtureWeight = mixtureWeights(componentIdx);
        seeds(componentIdx).initialMixtureWeight = mixtureWeights(componentIdx);
    end
end
