function minPoints = resolvePoleOccupiedLayerMinPoints(cfg)
% resolvePoleOccupiedLayerMinPoints: Read and sanitize the minimum
% per-fine-voxel point count used before a z layer contributes to pole
% vertical occupancy, run-length, and candidate gating maps.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   minPoints: scalar positive integer minimum point count per fine z voxel
    minPoints = 1;
    if isfield(cfg, "poleOccupiedLayerMinPoints") && isscalar(cfg.poleOccupiedLayerMinPoints) && isfinite(cfg.poleOccupiedLayerMinPoints)
        minPoints = max(1, round(double(cfg.poleOccupiedLayerMinPoints)));
    end
end
