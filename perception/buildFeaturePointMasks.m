function featureMasks = buildFeaturePointMasks(frame, ground, offGround, offGroundVoxelGrid, groundPointIdx)
% buildFeaturePointMasks: Express every perception channel as a full-frame
% logical point mask over the organized frame. Curb and road-marking points
% come from the ground branch as original indices; poles prefer the
% 3D-refined fine-voxel mask and fall back to the validated column mask;
% traffic signs prefer direct point indices and fall back to fine-voxel and
% column masks; facades use the assigned column mask.
%
% Input:
%   frame: organized point-cloud frame
%   ground: struct returned by extractGroundFeatures
%   offGround: struct returned by extractOffGroundFeatures
%   offGroundVoxelGrid: canonical off-ground voxel grid
%   groundPointIdx: original frame indices labeled as ground
%
% Output:
%   featureMasks: struct with groundPoint, curb, roadMarking, pole,
%       trafficSign, and facade [numFramePoints x 1] logical masks
    numFramePoints = numel(frame.x);
    featureMasks = struct();
    featureMasks.groundPoint = buildMaskFromIndices(groundPointIdx, numFramePoints);
    featureMasks.curb = buildMaskFromIndices(ground.channels.curb, numFramePoints);
    featureMasks.roadMarking = buildMaskFromIndices(ground.channels.roadMarking, numFramePoints);
    featureMasks.pole = mapPolePoints(offGroundVoxelGrid, offGround.pole, numFramePoints);
    featureMasks.trafficSign = mapTrafficSignPoints(offGroundVoxelGrid, offGround.trafficSign, numFramePoints);
    featureMasks.facade = mapColumnMaskToFrame(offGroundVoxelGrid, offGround.facade.mask, numFramePoints);
end

function featureMask = mapPolePoints(offGroundVoxelGrid, pole, numFramePoints)
% mapPolePoints: Convert the pole channel into a full-frame point mask,
% preferring the 3D-refined fine-voxel pole mask and falling back to the
% validated pole column mask when the fine mask selects no point.
%
% Input:
%   offGroundVoxelGrid: canonical off-ground voxel grid
%   pole: pole result struct from extractOffGroundFeatures
%   numFramePoints: total number of original frame points
%
% Output:
%   featureMask: [numFramePoints x 1] logical pole point mask
    if any(pole.fineVoxelMask(:))
        featureMask = mapFineVoxelMaskToFrame(offGroundVoxelGrid, pole.fineVoxelMask, pole.fineOrigin, pole.fineVoxelSize, numFramePoints);
        if any(featureMask)
            return;
        end
    end
    featureMask = mapColumnMaskToFrame(offGroundVoxelGrid, pole.mask, numFramePoints);
end

function featureMask = mapTrafficSignPoints(offGroundVoxelGrid, trafficSign, numFramePoints)
% mapTrafficSignPoints: Convert the traffic-sign channel into a full-frame
% point mask, using the direct high-intensity point indices when present
% and falling back to the fine-voxel and column masks otherwise.
%
% Input:
%   offGroundVoxelGrid: canonical off-ground voxel grid
%   trafficSign: traffic-sign result struct from extractOffGroundFeatures
%   numFramePoints: total number of original frame points
%
% Output:
%   featureMask: [numFramePoints x 1] logical traffic-sign point mask
    featureMask = buildMaskFromIndices(trafficSign.pointIndices, numFramePoints);
    if any(featureMask)
        return;
    end
    if any(trafficSign.fineVoxelMask(:))
        featureMask = mapFineVoxelMaskToFrame(offGroundVoxelGrid, trafficSign.fineVoxelMask, trafficSign.fineOrigin, trafficSign.fineVoxelSize, numFramePoints);
        if any(featureMask)
            return;
        end
    end
    featureMask = mapColumnMaskToFrame(offGroundVoxelGrid, trafficSign.mask, numFramePoints);
end

function featureMask = mapFineVoxelMaskToFrame(offGroundVoxelGrid, voxelMask, origin, voxelSize, numFramePoints)
% mapFineVoxelMaskToFrame: Project a [Ny x Nx x Nz] fine-voxel feature
% mask back to original organized-frame point indices using retained
% off-ground point coordinates.
%
% Input:
%   offGroundVoxelGrid: canonical off-ground fine voxel grid
%   voxelMask: [Ny x Nx x Nz] logical feature mask
%   origin: [1 x 3] lower metric edge for voxelMask
%   voxelSize: [1 x 3] metric voxel size for voxelMask
%   numFramePoints: total number of original frame points
%
% Output:
%   featureMask: [numFramePoints x 1] logical point mask
    featureMask = false(numFramePoints, 1);
    if isempty(voxelMask) || ndims(voxelMask) ~= 3 || ~any(voxelMask(:))
        return;
    end
    if numel(origin) < 3 || numel(voxelSize) < 3
        return;
    end
    origin = double(origin(1:3));
    voxelSize = double(voxelSize(1:3));
    if ~all(isfinite(origin)) || ~all(isfinite(voxelSize)) || ~all(voxelSize > 0)
        return;
    end
    points = double(offGroundVoxelGrid.points);
    pointIdx = double(offGroundVoxelGrid.pointIndices(:));
    if isempty(points) || isempty(pointIdx)
        return;
    end
    xBin = floor((points(:, 1) - origin(1)) ./ voxelSize(1)) + 1;
    yBin = floor((points(:, 2) - origin(2)) ./ voxelSize(2)) + 1;
    zBin = floor((points(:, 3) - origin(3)) ./ voxelSize(3)) + 1;
    validBin = isfinite(xBin) & isfinite(yBin) & isfinite(zBin);
    validBin = validBin & xBin >= 1 & xBin <= size(voxelMask, 2) & yBin >= 1 & yBin <= size(voxelMask, 1) & zBin >= 1 & zBin <= size(voxelMask, 3);
    selectedLocal = false(numel(pointIdx), 1);
    if any(validBin)
        voxelLinIdx = sub2ind(size(voxelMask), yBin(validBin), xBin(validBin), zBin(validBin));
        selectedLocal(validBin) = logical(voxelMask(voxelLinIdx));
    end
    selectedPointIdx = pointIdx(selectedLocal);
    validPointIdx = selectedPointIdx >= 1 & selectedPointIdx <= numFramePoints & selectedPointIdx == floor(selectedPointIdx);
    featureMask(selectedPointIdx(validPointIdx)) = true;
end

function featureMask = mapColumnMaskToFrame(offGroundVoxelGrid, columnMask, numFramePoints)
% mapColumnMaskToFrame: Project a [Ny x Nx] fine-column feature
% mask back to original organized-frame point indices using point voxel
% subscripts from the off-ground fine voxel grid.
%
% Input:
%   offGroundVoxelGrid: canonical off-ground fine voxel grid
%   columnMask: [Ny x Nx] logical fine-column feature mask
%   numFramePoints: total number of original frame points
%
% Output:
%   featureMask: [numFramePoints x 1] logical point mask
    featureMask = false(numFramePoints, 1);
    if isempty(columnMask) || ~ismatrix(columnMask) || ~any(columnMask(:))
        return;
    end
    if ~isfield(offGroundVoxelGrid, "pointVoxelSub") || size(offGroundVoxelGrid.pointVoxelSub, 2) < 2
        return;
    end
    pointSub = double(offGroundVoxelGrid.pointVoxelSub(:, 1:2));
    pointIdx = double(offGroundVoxelGrid.pointIndices(:));
    if isempty(pointSub) || isempty(pointIdx)
        return;
    end
    xBin = pointSub(:, 1);
    yBin = pointSub(:, 2);
    validBin = isfinite(xBin) & isfinite(yBin);
    validBin = validBin & xBin >= 1 & xBin <= size(columnMask, 2) & yBin >= 1 & yBin <= size(columnMask, 1);
    selectedLocal = false(numel(pointIdx), 1);
    if any(validBin)
        columnLinIdx = sub2ind(size(columnMask), yBin(validBin), xBin(validBin));
        selectedLocal(validBin) = logical(columnMask(columnLinIdx));
    end
    selectedPointIdx = pointIdx(selectedLocal);
    validPointIdx = selectedPointIdx >= 1 & selectedPointIdx <= numFramePoints & selectedPointIdx == floor(selectedPointIdx);
    featureMask(selectedPointIdx(validPointIdx)) = true;
end
