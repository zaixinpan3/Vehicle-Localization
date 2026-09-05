function pillars = pillarizePointCloud(frame, cfg)
% pillarizePointCloud: Filter returns and index XY pillars without a dense
% 3D volume. The shared binning utility preserves the historical ROI and
% range conventions. Sparse Z-bin membership is retained only for vertical
% distribution statistics; semantic decisions use XY pillars exclusively.
% Public pillar IDs index a [Ny Nx] raster in MATLAB column-major order.
    cfg.statisticsMode = "sparse";
    pillars = voxelizePointCloud(frame, cfg);
    geometry = pillars.gridConfig;
    mapSize = max(geometry.dims([2 1]), 1);
    sub = double(pillars.pointVoxelSub);
    pillars.pointPillarLinIdx = int32(sub2ind(mapSize, sub(:,2), sub(:,1)));
    pillars.pillarGeometry = struct("origin", geometry.minCorner(1:2), ...
        "cellSize", geometry.voxelSize(1:2), "mapSize", mapSize, "layout", "NyNx");
    pillars.spatialIndexType = "xyPillarsWithSparseHeightBins";
end
