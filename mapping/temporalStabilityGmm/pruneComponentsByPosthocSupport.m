function [components, pruningDiagnostics] = pruneComponentsByPosthocSupport(components, params)
% pruneComponentsByPosthocSupport: Remove structured GMM components whose
% post-hoc support amplitude, frame-bin count, point count, or geometric
% stability is below configured class thresholds after ordinary EM has
% completed.
%
% Input:
%   components: struct array of structured Gaussian support components
%   params: validated class parameter struct with pruning thresholds
%
% Output:
%   components: pruned struct array of components
%   pruningDiagnostics: scalar struct describing post-hoc support pruning order
    pruningDiagnostics = struct();
    pruningDiagnostics.appliedAfterIntegratedSupport = true;
    pruningDiagnostics.appliedAfterPosthocSupport = true;
    pruningDiagnostics.inputComponentCount = numel(components);
    pruningDiagnostics.outputComponentCount = numel(components);
    pruningDiagnostics.keepMask = false(numel(components), 1);
    pruningDiagnostics.removedCount = 0;
    pruningDiagnostics.inputSupportAmplitudes = zeros(numel(components), 1);
    pruningDiagnostics.inputTimestampBinCounts = zeros(numel(components), 1);
    pruningDiagnostics.inputEffectiveSupportPointCounts = zeros(numel(components), 1);
    pruningDiagnostics.inputGeometricStability = zeros(numel(components), 1);

    if isempty(components)
        error("buildTemporalStabilityGmmMap:NoComponentsBeforePruning", ...
            "No post-hoc temporal support components are available before pruning; empty component layers are invalid.");
    end

    keepMask = false(numel(components), 1);
    for componentIdx = 1:numel(components)
        pruningDiagnostics.inputSupportAmplitudes(componentIdx) = components(componentIdx).supportAmplitude;
        pruningDiagnostics.inputTimestampBinCounts(componentIdx) = components(componentIdx).timestampBinCount;
        pruningDiagnostics.inputEffectiveSupportPointCounts(componentIdx) = components(componentIdx).effectiveSupportPointCount;
        pruningDiagnostics.inputGeometricStability(componentIdx) = components(componentIdx).geometricStability;
        keepMask(componentIdx) = components(componentIdx).supportAmplitude >= params.minComponentSupport && ...
            components(componentIdx).timestampBinCount >= params.minTimestampBins && ...
            components(componentIdx).effectiveSupportPointCount >= params.minComponentPoints && ...
            components(componentIdx).normalStability >= params.minNormalStability;
    end
    pruningDiagnostics.keepMask = keepMask;
    components = components(keepMask);
    pruningDiagnostics.outputComponentCount = numel(components);
    pruningDiagnostics.removedCount = pruningDiagnostics.inputComponentCount - pruningDiagnostics.outputComponentCount;
    if isempty(components)
        error("buildTemporalStabilityGmmMap:NoRetainedComponents", ...
            "Post-hoc temporal support pruning removed every component; empty retained component layers are invalid.");
    end
end
