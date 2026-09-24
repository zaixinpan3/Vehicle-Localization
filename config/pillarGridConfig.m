function cfg = pillarGridConfig(executionMode)
% pillarGridConfig: Fixed whole XY pillar lattice and retained-return limits.
% Each execution mode owns one lattice. The online coarse product runs on
% 100 x 100 pillars of 0.6 m covering [-29.9, 30.1) m; the offline fine
% product keeps 334 x 334 pillars of 0.3 m covering [-50, 50.2) m. Both use
% the same 0.1 m lattice offset, so cell boundaries stay at -50 + k*0.3 m and
% every coarse pillar is the union of four offline pillars. Bounds and cell
% phase are shared by every perception stage of one execution mode.
    if nargin < 1, executionMode = "coarseProbabilityCloud"; end
    executionMode = string(executionMode);
    assert(isscalar(executionMode) && any(executionMode == ["coarseProbabilityCloud", "offline"]), ...
        "perception:InvalidExecutionMode", "Use coarseProbabilityCloud or offline.");
    if executionMode == "coarseProbabilityCloud"
        voxelSize = [0.6 0.6]; gridDims = [100 100];
    else
        voxelSize = [0.3 0.3]; gridDims = [334 334];
    end
    cfg = struct('executionMode', executionMode, 'voxelSize', voxelSize, 'gridDims', gridDims, ...
        'latticeOffset', [0.1 0.1], 'exclusionHalfSize', 3.0, 'minRange', 0, 'maxRange', Inf);
end
