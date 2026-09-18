function cfg = pillarGridConfig()
% pillarGridConfig: Fixed whole XY pillar lattice and retained-return limits.
% Keep the tuned 0.3 m cell boundaries at -50 + k*0.3 m. The fixed 334-cell
% lattice covers [-50, 50.2) m; latticeOffset sets its center relative to the
% sensor. Bounds and cell phase are shared by every perception stage.
    cfg = struct('voxelSize',[0.3 0.3],'gridDims',[334 334], ...
        'latticeOffset',[0.1 0.1], ...
        'exclusionHalfSize',3.0,'minRange',0,'maxRange',Inf);
end
