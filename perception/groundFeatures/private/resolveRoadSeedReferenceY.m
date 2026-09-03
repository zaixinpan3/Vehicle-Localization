function referenceY = resolveRoadSeedReferenceY(xyView, roadSeedMask)
% resolveRoadSeedReferenceY: Compute the lateral road reference from
% the occupied road-seed cells used by road extraction. The median y value
% is used so sparse seed outliers do not move the side split.
%
% Input:
%   xyView: XY view struct with yMap
%   roadSeedMask: [Ny x Nx] logical road seed raster
%
% Output:
%   referenceY: scalar median road-seed y coordinate
    yMap = double(xyView.yMap);
    referenceY = 0;
    if isempty(roadSeedMask)
        return;
    end

    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end
end
