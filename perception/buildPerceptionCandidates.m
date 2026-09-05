function candidates = buildPerceptionCandidates(pillars, ground, offGround)
% buildPerceptionCandidates: Publish semantic XY pillar IDs and geometry.
% Ground and structural returns share the XY lattice. Sparse height bins
% contribute statistics, but no height bin or point receives a feature label.
    geometry = pillars.pillarGeometry;
    [row, col] = find(offGround.poleCellMask);
    row = row(:); col = col(:);
    maps = offGround.columnMaps;
    centers = maps.origin + ([col row]-0.5).*[maps.dx maps.dy];
    bins = floor((centers-geometry.origin)./geometry.cellSize)+1;
    valid = bins(:,1)>=1 & bins(:,1)<=geometry.mapSize(2) & ...
        bins(:,2)>=1 & bins(:,2)<=geometry.mapSize(1);
    poles = sub2ind(geometry.mapSize,bins(valid,2),bins(valid,1));
    ids = {int32(find(ground.curbCellMask)); ...
        int32(find(ground.roadMarkingCellMask)); int32(unique(poles))};
    candidates = struct("productType", "sparseSemanticPillarCandidates", ...
        "geometry", geometry, "semanticNames", ["curb"; "roadMarking"; "pole"], ...
        "pillarIndices", {ids}, "framePointCount", pillars.numInputPoints, ...
        "groundReflectivityThreshold", ground.roadMarkingReflectivityThreshold);
end
