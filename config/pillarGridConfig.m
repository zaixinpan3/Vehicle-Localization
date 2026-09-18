function cfg = pillarGridConfig()
% pillarGridConfig: Whole XY pillar lattice and retained-return limits.
% The lattice is fixed: gridDims pillars of voxelSize meters, centered at the
% sensor origin. No ROI is stated separately; returns outside the lattice are
% ignored. 200 x 200 pillars of 0.3 m cover [-30, 30) m in x and y.
    cfg = struct('voxelSize',[0.3 0.3],'gridDims',[200 200], ...
        'exclusionHalfSize',3.0,'minRange',0,'maxRange',Inf);
end
