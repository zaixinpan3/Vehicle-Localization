function cfg = pillarGridConfig()
% pillarGridConfig: Whole XY pillar geometry and retained-return limits.
    cfg = struct('voxelSize',[0.3 0.3],'roiLimits',[-50 50 -50 50], ...
        'exclusionHalfSize',3.0,'minRange',0,'maxRange',Inf);
end
