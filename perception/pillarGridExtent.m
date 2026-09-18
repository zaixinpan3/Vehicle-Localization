function extent = pillarGridExtent(cfg)
% pillarGridExtent: XY bounds [xMin xMax yMin yMax] of the fixed pillar lattice.
% The lattice is cfg.gridDims pillars of cfg.voxelSize meters centered at the
% sensor origin plus latticeOffset. There is no separate region of interest: returns outside the
% lattice are ignored by pillarizePointCloud. An optional cfg.latticeOffset
% [dx dy] sets its center without introducing a second geometry path.
    assert(~isfield(cfg, "roiLimits"), "perception:ObsoleteRoiLimits", ...
        "The pillar lattice is fixed by gridDims and voxelSize; remove roiLimits.");
    spacing = double(cfg.voxelSize(:).');
    dims = double(cfg.gridDims(:).');
    assert(numel(spacing) == 2, "perception:InvalidPillarSpacing", ...
        "Whole pillars require exactly two XY spacings.");
    assert(all(isfinite(spacing) & spacing > 0), "perception:InvalidPillarSpacing", ...
        "XY pillar spacings must be positive and finite.");
    assert(numel(dims) == 2 && all(isfinite(dims) & dims >= 1 & dims == round(dims)), ...
        "perception:InvalidPillarGridDims", "gridDims must contain two positive integer XY counts.");
    lower = -dims.*spacing./2;
    if isfield(cfg, "latticeOffset") && ~isempty(cfg.latticeOffset)
        offset = double(cfg.latticeOffset(:).');
        assert(numel(offset) == 2 && all(isfinite(offset)), "perception:InvalidLatticeOffset", ...
            "latticeOffset must contain two finite XY shifts.");
        lower = lower + offset;
    end
    upper = lower + dims.*spacing;
    extent = [lower(1), upper(1), lower(2), upper(2)];
end
