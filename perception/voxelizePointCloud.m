function voxelGrid = voxelizePointCloud(pointCloud, cfg)
% voxelizePointCloud: Build a reusable canonical 3D voxel grid from an
% organized frame or an unordered [N x 3] point cloud, preserving point-
% to-voxel and voxel-to-point mappings together with dense voxel counts
% and first/second-order moment tensors for downstream modules that need
% shared spatial indexing, PCA-ready voxel statistics, and recovery of
% original point membership per voxel.
%
% Input:
%   pointCloud: either
%       1) struct with fields x, y, z and optional range, intensity, or
%          reflectivity, or
%       2) [N x 3] numeric matrix with columns [x, y, z]
%   cfg: struct with optional fields:
%       voxelSize: scalar, [1 x 2], or [1 x 3] voxel size
%           in meters
%       roiLimits: [] or [1 x 4]/[1 x 6] limits
%           [xMin xMax yMin yMax (zMin zMax)]
%       origin or minCorner: [1 x 3] lower voxel-grid edge
%       voxelOffset: [1 x 3] additive offset applied to
%           the chosen lower voxel-grid edge
%       exclusionHalfSize: scalar XY exclusion half-width in meters
%       minRange: scalar minimum range in meters
%       maxRange: scalar maximum range in meters
%       statisticsMode: "full", "countOnly", or "sparse". Sparse mode
%           builds membership only; every dense 3D statistic is empty.
%       buildPointLookup: true by default. False omits sorted inverse lookup
%           arrays; numOccupiedVoxels is NaN because that count is not computed.
%
% Output:
%   voxelGrid: struct with fields:
%       spatialIndexType: string scalar "voxelGrid"
%       gridConfig: struct with dims [1 x 3], voxelSize [1 x 3], origin
%           [1 x 3] at the first voxel center, minCorner [1 x 3],
%           maxCorner [1 x 3], roiLimits [1 x 6], and countLayout
%       count,sumX,sumY,sumZ,sumXX,sumYY,sumZZ,sumXY,sumXZ,sumYZ:
%           [Nx x Ny x Nz] single dense voxel statistics, with empty moment
%           placeholders in countOnly mode
%       points: [K x 3] double retained point coordinates
%       pointIndices: [K x 1] int32 original point indices
%       pointVoxelSub: [K x 3] int32 voxel subscripts [x y z]
%       pointVoxelLinIdx: [K x 1] int32 voxel linear indices
%       occupiedVoxelLinIdx: [M x 1] int32 occupied voxel indices
%       occupiedVoxelSub: [M x 3] int32 occupied voxel subscripts [x y z]
%       voxelPointOffsets: [(M+1) x 1] int32 CSR-style offsets into
%           voxelPointLocalIdx and voxelPointIndices for each occupied voxel
%       voxelPointLocalIdx: [K x 1] int32 indices into points/pointIndices
%       voxelPointIndices: [K x 1] int32 original point indices sorted by
%           occupied voxel
%       pointAttributes: struct of retained per-point attributes aligned
%           with points, including range and optional intensity/reflectivity
%       inputType: string scalar "organized" or "unorganized"
%       inputSize: [1 x 2] original organized size or [N 1] for matrix
%       numInputPoints, numFilteredPoints, numOccupiedVoxels: scalars
    if nargin < 2 || ~isstruct(cfg)
        cfg = struct();
    end

    [xyz, pointIndices, pointAttributes, inputMeta] = readPerceptionPoints(pointCloud);
    voxelCfg = resolveVoxelConfig(cfg);

    keepMask = buildPointKeepMask(xyz, pointAttributes, voxelCfg);
    xyz = xyz(keepMask, :);
    pointIndices = pointIndices(keepMask);
    pointAttributes = filterPointAttributes(pointAttributes, keepMask);

    [minCorner, maxCorner, dims] = resolveGridGeometry(xyz, voxelCfg);
    [xyz, pointIndices, pointAttributes, pointVoxelSub, pointVoxelLinIdx] = ...
        binPointsIntoGrid(xyz, pointIndices, pointAttributes, minCorner, dims, voxelCfg.voxelSize);

    [count, sumX, sumY, sumZ, sumXX, sumYY, sumZZ, sumXY, sumXZ, sumYZ] = ...
        accumulateVoxelStatistics(xyz, pointVoxelSub, dims, voxelCfg.statisticsMode);
    buildPointLookup = ~isfield(cfg,"buildPointLookup") || logical(cfg.buildPointLookup);
    if buildPointLookup
        [occupiedVoxelLinIdx, occupiedVoxelSub, voxelPointOffsets, voxelPointLocalIdx] = ...
            buildVoxelPointMapping(pointVoxelLinIdx, dims);
    else
        occupiedVoxelLinIdx = zeros(0,1,"int32");
        occupiedVoxelSub = zeros(0,3,"int32");
        voxelPointOffsets = int32(1);
        voxelPointLocalIdx = zeros(0,1,"int32");
    end
    voxelPointIndices = zeros(0, 1, "int32");
    if ~isempty(voxelPointLocalIdx)
        voxelPointIndices = int32(pointIndices(double(voxelPointLocalIdx(:))));
    end

    voxelGrid = struct();
    voxelGrid.spatialIndexType = "voxelGrid";
    voxelGrid.gridConfig = struct();
    voxelGrid.gridConfig.dims = double(dims(:).');
    voxelGrid.gridConfig.voxelSize = double(voxelCfg.voxelSize(:).');
    voxelGrid.gridConfig.origin = double(minCorner(:).' + (0.5 .* voxelCfg.voxelSize(:).'));
    voxelGrid.gridConfig.minCorner = double(minCorner(:).');
    voxelGrid.gridConfig.maxCorner = double(maxCorner(:).');
    voxelGrid.gridConfig.roiLimits = [double(minCorner(1)), double(maxCorner(1)), ...
        double(minCorner(2)), double(maxCorner(2)), double(minCorner(3)), double(maxCorner(3))];
    voxelGrid.gridConfig.countLayout = "NxNyNz";
    voxelGrid.count = count;
    voxelGrid.sumX = sumX;
    voxelGrid.sumY = sumY;
    voxelGrid.sumZ = sumZ;
    voxelGrid.sumXX = sumXX;
    voxelGrid.sumYY = sumYY;
    voxelGrid.sumZZ = sumZZ;
    voxelGrid.sumXY = sumXY;
    voxelGrid.sumXZ = sumXZ;
    voxelGrid.sumYZ = sumYZ;
    voxelGrid.points = xyz;
    voxelGrid.pointIndices = int32(pointIndices(:));
    voxelGrid.pointVoxelSub = int32(pointVoxelSub);
    voxelGrid.pointVoxelLinIdx = int32(pointVoxelLinIdx(:));
    voxelGrid.occupiedVoxelLinIdx = int32(occupiedVoxelLinIdx(:));
    voxelGrid.occupiedVoxelSub = int32(occupiedVoxelSub);
    voxelGrid.voxelPointOffsets = int32(voxelPointOffsets(:));
    voxelGrid.voxelPointLocalIdx = int32(voxelPointLocalIdx(:));
    voxelGrid.voxelPointIndices = int32(voxelPointIndices(:));
    voxelGrid.pointAttributes = pointAttributes;
    voxelGrid.inputType = inputMeta.inputType;
    voxelGrid.inputSize = double(inputMeta.inputSize(:).');
    voxelGrid.numInputPoints = double(inputMeta.numInputPoints);
    voxelGrid.numFilteredPoints = double(size(xyz, 1));
    voxelGrid.numOccupiedVoxels = double(numel(occupiedVoxelLinIdx));
    voxelGrid.hasPointLookup = buildPointLookup;
    if ~buildPointLookup && ~isempty(xyz), voxelGrid.numOccupiedVoxels = NaN; end
end

function voxelCfg = resolveVoxelConfig(cfg)
% resolveVoxelConfig: Normalize voxelization configuration
% and defaults into one internal struct with explicit voxel geometry,
% range limits, and XY exclusion settings.
%
% Input:
%   cfg: user configuration struct
%
% Output:
%   voxelCfg: struct with voxelSize, roiLimits, origin, voxelOffset,
%       exclusionHalfSize, minRange, maxRange, and statisticsMode fields
    voxelCfg = struct();
    voxelSize = [];
    if isfield(cfg, "voxelSize") && ~isempty(cfg.voxelSize)
        voxelSize = double(cfg.voxelSize(:).');
    end
    if isempty(voxelSize)
        voxelSize = [0.2, 0.2, 0.2];
    end
    if isscalar(voxelSize)
        voxelSize = repmat(voxelSize, 1, 3);
    elseif numel(voxelSize) == 2
        voxelSize = [voxelSize(1:2), max(voxelSize(1:2))];
    else
        voxelSize = voxelSize(1:3);
    end
    assert(all(isfinite(voxelSize)) && all(voxelSize > 0), ...
        "voxelSize must contain positive finite values.");
    voxelCfg.voxelSize = voxelSize;

    roiLimits = [];
    if isfield(cfg, "roiLimits") && ~isempty(cfg.roiLimits)
        roiLimits = double(cfg.roiLimits(:).');
    elseif all(isfield(cfg, ["xMin", "xMax", "yMin", "yMax"]))
        roiLimits = [double(cfg.xMin), double(cfg.xMax), double(cfg.yMin), double(cfg.yMax)];
        if all(isfield(cfg, ["zMin", "zMax"]))
            roiLimits = [roiLimits, double(cfg.zMin), double(cfg.zMax)];
        end
    end
    if ~isempty(roiLimits)
        assert((numel(roiLimits) == 4 || numel(roiLimits) == 6) && all(isfinite(roiLimits)), ...
            "roiLimits must contain 4 or 6 finite values.");
    end
    voxelCfg.roiLimits = roiLimits;

    origin = [];
    if isfield(cfg, "minCorner") && ~isempty(cfg.minCorner)
        origin = double(cfg.minCorner(:).');
    elseif isfield(cfg, "origin") && ~isempty(cfg.origin)
        origin = double(cfg.origin(:).');
    end
    if ~isempty(origin)
        if isscalar(origin)
            origin = repmat(origin, 1, 3);
        elseif numel(origin) == 2
            origin = [origin(1:2), 0];
        else
            origin = origin(1:3);
        end
        assert(all(isfinite(origin)), "origin must contain finite values.");
    end
    voxelCfg.origin = origin;

    voxelOffset = [0, 0, 0];
    if isfield(cfg, "voxelOffset") && ~isempty(cfg.voxelOffset)
        voxelOffset = double(cfg.voxelOffset(:).');
    end
    if isscalar(voxelOffset)
        voxelOffset = repmat(voxelOffset, 1, 3);
    elseif numel(voxelOffset) == 2
        voxelOffset = [voxelOffset(1:2), 0];
    else
        voxelOffset = voxelOffset(1:3);
    end
    assert(all(isfinite(voxelOffset)), "voxelOffset must contain finite values.");
    voxelCfg.voxelOffset = voxelOffset;

    voxelCfg.exclusionHalfSize = readScalarConfig(cfg, "exclusionHalfSize", 0);
    voxelCfg.minRange = readScalarConfig(cfg, "minRange", 0);
    voxelCfg.maxRange = readFiniteOrInfiniteScalarConfig(cfg, "maxRange", inf);
    assert(voxelCfg.exclusionHalfSize >= 0, "exclusionHalfSize must be >= 0.");
    assert(voxelCfg.minRange >= 0, "minRange must be >= 0.");
    assert(voxelCfg.maxRange > voxelCfg.minRange, "maxRange must be greater than minRange.");

    statisticsMode = "full";
    if isfield(cfg, "statisticsMode") && ~isempty(cfg.statisticsMode)
        statisticsMode = lower(strtrim(string(cfg.statisticsMode)));
    end
    if ~any(statisticsMode == ["countonly", "sparse"])
        statisticsMode = "full";
    end
    voxelCfg.statisticsMode = statisticsMode;
end

function keepMask = buildPointKeepMask(xyz, pointAttributes, voxelCfg)
% buildPointKeepMask: Combine finite-value checks, optional ROI
% limits, range gating, and XY exclusion filtering into the retained-point
% mask used before grid geometry and voxel indexing are finalized.
%
% Input:
%   xyz: [N x 3] double point coordinates
%   pointAttributes: struct with range field aligned with xyz
%   voxelCfg: normalized voxelizer configuration
%
% Output:
%   keepMask: [N x 1] logical retained-point mask
    if isempty(xyz)
        keepMask = false(0, 1);
        return;
    end

    keepMask = all(isfinite(xyz), 2);
    rangeVals = sqrt(sum(xyz.^2, 2));
    if isfield(pointAttributes, "range") && numel(pointAttributes.range) == size(xyz, 1)
        rangeVals = double(pointAttributes.range(:));
    end
    keepMask = keepMask & isfinite(rangeVals);
    keepMask = keepMask & (rangeVals >= voxelCfg.minRange) & (rangeVals <= voxelCfg.maxRange);
    if voxelCfg.exclusionHalfSize > 0
        keepMask = keepMask & ~((abs(xyz(:, 1)) < voxelCfg.exclusionHalfSize) & (abs(xyz(:, 2)) < voxelCfg.exclusionHalfSize));
    end
    if ~isempty(voxelCfg.roiLimits)
        roiLimits = voxelCfg.roiLimits;
        keepMask = keepMask & ...
            (xyz(:, 1) >= roiLimits(1)) & (xyz(:, 1) <= roiLimits(2)) & ...
            (xyz(:, 2) >= roiLimits(3)) & (xyz(:, 2) <= roiLimits(4));
        if numel(roiLimits) >= 6
            keepMask = keepMask & (xyz(:, 3) >= roiLimits(5)) & (xyz(:, 3) <= roiLimits(6));
        end
    end
end

function pointAttributes = filterPointAttributes(pointAttributes, keepMask)
% filterPointAttributes: Apply one retained-point mask to every
% vector-valued per-point attribute stored in the attribute struct so all
% downstream arrays remain aligned after filtering and bin clipping.
%
% Input:
%   pointAttributes: struct of per-point attribute vectors
%   keepMask: [N x 1] logical retained-point mask
%
% Output:
%   pointAttributes: struct with all aligned attributes filtered
    fieldNames = string(fieldnames(pointAttributes));
    for iField = 1:numel(fieldNames)
        fieldName = char(fieldNames(iField));
        values = pointAttributes.(fieldName);
        if isempty(values)
            continue;
        end
        if numel(values) == numel(keepMask)
            pointAttributes.(fieldName) = values(keepMask);
        end
    end
end

function [minCorner, maxCorner, dims] = resolveGridGeometry(xyz, voxelCfg)
% resolveGridGeometry: Resolve the lower edge, upper edge, and voxel
% counts of the canonical dense grid from filtered data bounds, optional
% ROI limits, and optional explicit origin or offset overrides.
%
% Input:
%   xyz: [N x 3] filtered point coordinates
%   voxelCfg: normalized voxelizer configuration
%
% Output:
%   minCorner: [1 x 3] lower grid edge in meters
%   maxCorner: [1 x 3] upper grid edge in meters
%   dims: [1 x 3] voxel counts [Nx Ny Nz]
    dims = [0, 0, 0];
    minCorner = [0, 0, 0];
    maxCorner = [0, 0, 0];

    if isempty(xyz) && isempty(voxelCfg.roiLimits)
        return;
    end

    dataMin = [0, 0, 0];
    dataMax = [0, 0, 0];
    if ~isempty(xyz)
        dataMin = min(xyz, [], 1);
        dataMax = max(xyz, [], 1);
    end

    if ~isempty(voxelCfg.roiLimits)
        roiLimits = voxelCfg.roiLimits;
        baseMin = [roiLimits(1), roiLimits(3), 0];
        baseMax = [roiLimits(2), roiLimits(4), 0];
        if numel(roiLimits) >= 6
            baseMin(3) = roiLimits(5);
            baseMax(3) = roiLimits(6);
        else
            if isempty(xyz)
                baseMin(3) = 0;
                baseMax(3) = voxelCfg.voxelSize(3);
            else
                baseMin(3) = dataMin(3);
                baseMax(3) = dataMax(3);
            end
        end
    else
        baseMin = dataMin;
        baseMax = dataMax;
    end

    if ~isempty(voxelCfg.origin)
        minCorner = voxelCfg.origin + voxelCfg.voxelOffset;
    else
        minCorner = baseMin + voxelCfg.voxelOffset;
    end
    maxCorner = max(baseMax, minCorner + voxelCfg.voxelSize);
    extents = max(maxCorner - minCorner, 0);
    dims = max(ceil(extents ./ voxelCfg.voxelSize), 1);
    maxCorner = minCorner + (dims .* voxelCfg.voxelSize);
end

function [xyz, pointIndices, pointAttributes, pointVoxelSub, pointVoxelLinIdx] = ...
        binPointsIntoGrid(xyz, pointIndices, pointAttributes, minCorner, dims, voxelSize)
% binPointsIntoGrid: Convert filtered metric coordinates into voxel
% subscripts and linear indices for one dense [Nx x Ny x Nz] grid while
% discarding any samples that fall outside the resolved metric support.
%
% Input:
%   xyz: [N x 3] filtered point coordinates
%   pointIndices: [N x 1] original point indices
%   pointAttributes: struct of per-point attributes aligned with xyz
%   minCorner: [1 x 3] lower voxel-grid edge
%   dims: [1 x 3] voxel counts [Nx Ny Nz]
%   voxelSize: [1 x 3] voxel spacing [dx dy dz]
%
% Output:
%   xyz: [K x 3] retained in-grid points
%   pointIndices: [K x 1] retained original indices
%   pointAttributes: filtered retained-point attributes
%   pointVoxelSub: [K x 3] int32 voxel subscripts [x y z]
%   pointVoxelLinIdx: [K x 1] int32 voxel linear indices
    pointVoxelSub = zeros(0, 3, "int32");
    pointVoxelLinIdx = zeros(0, 1, "int32");
    if isempty(xyz) || any(dims <= 0)
        xyz = zeros(0, 3);
        pointIndices = zeros(0, 1, "int32");
        pointAttributes = filterPointAttributes(pointAttributes, false(0, 1));
        return;
    end

    xBin = floor((xyz(:, 1) - minCorner(1)) ./ voxelSize(1)) + 1;
    yBin = floor((xyz(:, 2) - minCorner(2)) ./ voxelSize(2)) + 1;
    zBin = floor((xyz(:, 3) - minCorner(3)) ./ voxelSize(3)) + 1;
    valid = isfinite(xBin) & isfinite(yBin) & isfinite(zBin);
    valid = valid & (xBin >= 1) & (xBin <= dims(1)) & ...
        (yBin >= 1) & (yBin <= dims(2)) & (zBin >= 1) & (zBin <= dims(3));

    xyz = xyz(valid, :);
    pointIndices = pointIndices(valid);
    pointAttributes = filterPointAttributes(pointAttributes, valid);
    if isempty(xyz)
        pointVoxelSub = zeros(0, 3, "int32");
        pointVoxelLinIdx = zeros(0, 1, "int32");
        return;
    end

    xBin = xBin(valid);
    yBin = yBin(valid);
    zBin = zBin(valid);
    pointVoxelSub = int32([xBin(:), yBin(:), zBin(:)]);
    pointVoxelLinIdx = int32(sub2ind(double(dims), double(xBin(:)), double(yBin(:)), double(zBin(:))));
end

function [count, sumX, sumY, sumZ, sumXX, sumYY, sumZZ, sumXY, sumXZ, sumYZ] = ...
        accumulateVoxelStatistics(xyz, pointVoxelSub, dims, statisticsMode)
% accumulateVoxelStatistics: Accumulate point counts and first/second
% moments into dense [Nx x Ny x Nz] tensors so downstream modules can
% perform shared statistics-driven processing directly on the voxel grid,
% or skip the dense moment accumulations when only count statistics are
% requested for speed-oriented perception paths.
%
% Input:
%   xyz: [K x 3] retained point coordinates
%   pointVoxelSub: [K x 3] voxel subscripts [x y z]
%   dims: [1 x 3] voxel counts [Nx Ny Nz]
%   statisticsMode: string scalar "full" or "countOnly"
%
% Output:
%   count,sumX,sumY,sumZ,sumXX,sumYY,sumZZ,sumXY,sumXZ,sumYZ:
%       [Nx x Ny x Nz] single dense voxel statistics
    outSize = double(max(dims, 0));
    countOnlyMode = lower(strtrim(string(statisticsMode))) == "countonly";
    emptyMoment = zeros(0, 0, 0, "single");
    if lower(strtrim(string(statisticsMode))) == "sparse"
        [count, sumX, sumY, sumZ, sumXX, sumYY, sumZZ, sumXY, sumXZ, sumYZ] = deal(emptyMoment);
        return;
    end
    count = zeros(outSize(1), outSize(2), outSize(3), "single");
    if countOnlyMode
        sumX = emptyMoment;
        sumY = emptyMoment;
        sumZ = emptyMoment;
        sumXX = emptyMoment;
        sumYY = emptyMoment;
        sumZZ = emptyMoment;
        sumXY = emptyMoment;
        sumXZ = emptyMoment;
        sumYZ = emptyMoment;
    else
        sumX = zeros(outSize(1), outSize(2), outSize(3), "single");
        sumY = zeros(outSize(1), outSize(2), outSize(3), "single");
        sumZ = zeros(outSize(1), outSize(2), outSize(3), "single");
        sumXX = zeros(outSize(1), outSize(2), outSize(3), "single");
        sumYY = zeros(outSize(1), outSize(2), outSize(3), "single");
        sumZZ = zeros(outSize(1), outSize(2), outSize(3), "single");
        sumXY = zeros(outSize(1), outSize(2), outSize(3), "single");
        sumXZ = zeros(outSize(1), outSize(2), outSize(3), "single");
        sumYZ = zeros(outSize(1), outSize(2), outSize(3), "single");
    end

    if isempty(xyz) || isempty(pointVoxelSub) || any(dims <= 0)
        return;
    end

    subscriptArray = double(pointVoxelSub);
    count = single(accumarray(subscriptArray, 1, outSize, @sum, 0));
    if countOnlyMode
        return;
    end

    sumX = single(accumarray(subscriptArray, xyz(:, 1), outSize, @sum, 0));
    sumY = single(accumarray(subscriptArray, xyz(:, 2), outSize, @sum, 0));
    sumZ = single(accumarray(subscriptArray, xyz(:, 3), outSize, @sum, 0));
    sumXX = single(accumarray(subscriptArray, xyz(:, 1) .* xyz(:, 1), outSize, @sum, 0));
    sumYY = single(accumarray(subscriptArray, xyz(:, 2) .* xyz(:, 2), outSize, @sum, 0));
    sumZZ = single(accumarray(subscriptArray, xyz(:, 3) .* xyz(:, 3), outSize, @sum, 0));
    sumXY = single(accumarray(subscriptArray, xyz(:, 1) .* xyz(:, 2), outSize, @sum, 0));
    sumXZ = single(accumarray(subscriptArray, xyz(:, 1) .* xyz(:, 3), outSize, @sum, 0));
    sumYZ = single(accumarray(subscriptArray, xyz(:, 2) .* xyz(:, 3), outSize, @sum, 0));
end

function [occupiedVoxelLinIdx, occupiedVoxelSub, voxelPointOffsets, voxelPointLocalIdx] = ...
        buildVoxelPointMapping(pointVoxelLinIdx, dims)
% buildVoxelPointMapping: Build CSR-style voxel-to-point lookup
% arrays from point-wise voxel indices so downstream code can recover the
% original points contained in any occupied fine voxel efficiently.
%
% Input:
%   pointVoxelLinIdx: [K x 1] int32 voxel linear indices
%   dims: [1 x 3] voxel counts [Nx Ny Nz]
%
% Output:
%   occupiedVoxelLinIdx: [M x 1] int32 occupied voxel indices
%   occupiedVoxelSub: [M x 3] int32 occupied voxel subscripts [x y z]
%   voxelPointOffsets: [(M+1) x 1] int32 offsets into voxelPointLocalIdx
%   voxelPointLocalIdx: [K x 1] int32 indices into the retained points
    occupiedVoxelLinIdx = zeros(0, 1, "int32");
    occupiedVoxelSub = zeros(0, 3, "int32");
    voxelPointOffsets = int32(1);
    voxelPointLocalIdx = zeros(0, 1, "int32");

    if isempty(pointVoxelLinIdx)
        return;
    end

    [sortedLinIdx, sortOrder] = sort(double(pointVoxelLinIdx(:)));
    voxelPointLocalIdx = int32(sortOrder(:));
    occupiedMask = [true; diff(sortedLinIdx) ~= 0];
    occupiedStartIdx = find(occupiedMask);
    occupiedVoxelLinIdx = int32(sortedLinIdx(occupiedStartIdx));
    voxelPointOffsets = int32([occupiedStartIdx; numel(sortedLinIdx) + 1]);
    [xSub, ySub, zSub] = ind2sub(double(dims), double(occupiedVoxelLinIdx));
    occupiedVoxelSub = int32([xSub(:), ySub(:), zSub(:)]);
end

function value = readScalarConfig(cfg, fieldName, defaultValue)
% readScalarConfig: Read one finite scalar configuration value with
% default fallback and scalar validation.
%
% Input:
%   cfg: configuration struct
%   fieldName: target field name
%   defaultValue: fallback scalar value
%
% Output:
%   value: validated finite scalar double
    if isfield(cfg, fieldName) && ~isempty(cfg.(fieldName))
        value = double(cfg.(fieldName));
    else
        value = double(defaultValue);
    end
    assert(isscalar(value) && isfinite(value), ...
        "Config field %s must be a finite scalar.", fieldName);
end

function value = readFiniteOrInfiniteScalarConfig(cfg, fieldName, defaultValue)
% readFiniteOrInfiniteScalarConfig: Read one scalar configuration
% value while allowing finite values and +/-Inf.
%
% Input:
%   cfg: configuration struct
%   fieldName: target field name
%   defaultValue: fallback scalar value
%
% Output:
%   value: validated scalar double
    if isfield(cfg, fieldName) && ~isempty(cfg.(fieldName))
        value = double(cfg.(fieldName));
    else
        value = double(defaultValue);
    end
    assert(isscalar(value) && ~isnan(value), ...
        "Config field %s must be a scalar numeric value.", fieldName);
end
