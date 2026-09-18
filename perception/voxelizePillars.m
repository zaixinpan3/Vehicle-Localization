function voxelGrid = voxelizePillars(pillars, dz)
% voxelizePillars: Split whole XY pillars into fixed height layers for offline fine detection.
% Layer boundaries lie on the vehicle-frame height lattice k*dz, so a return's
% layer never depends on which other returns the frame contains; layer 1 is the
% lowest occupied lattice layer. XY membership is taken from the pillars as is.
% No dense statistics or inverse voxel lookup is allocated.
    assert(isstruct(pillars) && all(isfield(pillars, ["gridConfig", "points", "pointIndices", ...
        "pointPillarSub", "pointAttributes"])), "voxelizePillars requires pillarizePointCloud output.");
    dz = double(dz);
    assert(isscalar(dz) && isfinite(dz) && dz > 0, "perception:InvalidLayerHeight", ...
        "Height layers require one positive finite spacing.");
    xy = pillars.gridConfig;
    z = double(pillars.points(:, 3));
    zFloor = 0;
    if ~isempty(z)
        zFloor = floor(min(z)./dz).*dz;
    end
    zBin = floor((z - zFloor)./dz) + 1;
    dims = [double(xy.dims(1:2)), max([zBin; 1])];
    voxelSize = [double(xy.voxelSize(1:2)), dz];
    minCorner = [double(xy.minCorner(1:2)), zFloor];
    pointVoxelSub = [double(pillars.pointPillarSub), zBin];
    voxelGrid = struct("spatialIndexType", "voxelGrid", ...
        "gridConfig", struct("dims", dims, "voxelSize", voxelSize, "minCorner", minCorner, ...
        "maxCorner", minCorner + dims.*voxelSize, "origin", minCorner + voxelSize./2), ...
        "points", pillars.points, "pointIndices", pillars.pointIndices, ...
        "pointVoxelSub", int32(pointVoxelSub), ...
        "pointVoxelLinIdx", int32(sub2ind(dims, pointVoxelSub(:, 1), pointVoxelSub(:, 2), pointVoxelSub(:, 3))), ...
        "pointAttributes", pillars.pointAttributes, "numFilteredPoints", size(pillars.points, 1));
end
