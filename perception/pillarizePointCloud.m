function pillars = pillarizePointCloud(frame, cfg)
% pillarizePointCloud: Index whole XY pillars and retain their XYZ statistics.
% Z is never quantized. voxelSize contains exactly two XY spacings.
% Optional Z ROI limits crop returns but never divide a pillar.
    [xyz, indices, attributes, meta] = readPerceptionPoints(frame);
    spacing = double(cfg.voxelSize(:).');
    assert(numel(spacing)==2,'perception:InvalidPillarSpacing', ...
        'Whole pillars require exactly two XY spacings.');
    assert(all(isfinite(spacing) & spacing>0), 'Invalid XY pillar spacing.');
    roi = double(cfg.roiLimits(:).');
    assert(isempty(roi) || (any(numel(roi)==[4 6]) && all(isfinite(roi))), 'Invalid ROI.');
    keep = all(isfinite(xyz),2) & isfinite(attributes.range);
    keep = keep & attributes.range>=cfg.minRange & attributes.range<=cfg.maxRange;
    keep = keep & ~(abs(xyz(:,1))<cfg.exclusionHalfSize & abs(xyz(:,2))<cfg.exclusionHalfSize);
    if ~isempty(roi)
        assert(roi(1)<=roi(2) && roi(3)<=roi(4), 'Unordered XY ROI.');
        keep = keep & xyz(:,1)>=roi(1) & xyz(:,1)<=roi(2) & xyz(:,2)>=roi(3) & xyz(:,2)<=roi(4);
        if numel(roi)==6
            assert(roi(5)<=roi(6), 'Unordered Z ROI.');
            keep = keep & xyz(:,3)>=roi(5) & xyz(:,3)<=roi(6);
        end
    end
    xyz = xyz(keep,:); indices = indices(keep);
    names = fieldnames(attributes);
    for k=1:numel(names), attributes.(names{k})=attributes.(names{k})(keep); end
    if ~isempty(roi)
        lower = roi([1 3]); upper = roi([2 4]);
    elseif isempty(xyz)
        lower = [0 0]; upper = spacing;
    else
        lower = min(xyz(:,1:2),[],1); upper = max(xyz(:,1:2),[],1);
    end
    if isfield(cfg,'minCorner') && ~isempty(cfg.minCorner)
        lower = xyValue(cfg.minCorner);
    elseif isfield(cfg,'origin') && ~isempty(cfg.origin)
        lower = xyValue(cfg.origin);
    end
    if isfield(cfg,'voxelOffset') && ~isempty(cfg.voxelOffset)
        lower = lower+xyValue(cfg.voxelOffset);
    end
    dims = max(ceil((upper-lower)./spacing),1);
    upper = lower+dims.*spacing;
    bins = floor((xyz(:,1:2)-lower)./spacing)+1;
    valid = all(bins>=1 & bins<=dims,2);
    xyz=xyz(valid,:); indices=indices(valid); bins=bins(valid,:);
    for k=1:numel(names), attributes.(names{k})=attributes.(names{k})(valid); end
    mapSize=dims([2 1]);
    ids=int32(sub2ind(mapSize,bins(:,2),bins(:,1)));
    pillars=struct('spatialIndexType',"xyPillars",'points',xyz, ...
        'pointIndices',indices,'pointAttributes',attributes, ...
        'pointPillarSub',int32(bins),'pointPillarLinIdx',ids, ...
        'inputType',meta.inputType,'inputSize',meta.inputSize, ...
        'numInputPoints',meta.numInputPoints,'numFilteredPoints',size(xyz,1));
    pillars.pillarGeometry=struct('origin',lower,'cellSize',spacing,'mapSize',mapSize,'layout',"NyNx");
    % Terrain preprocessing consumes only XY geometry through this shared name.
    pillars.gridConfig=struct('dims',dims,'voxelSize',spacing, ...
        'minCorner',lower,'maxCorner',upper,'origin',lower+spacing/2);
    useNative=isfield(cfg,'useNativeKernels') && cfg.useNativeKernels;
    pillars.statistics=aggregatePillarStatistics(xyz,ids,attributes,useNative);
end

function value=xyValue(value)
% xyValue: Normalize an optional XY geometry override.
    value=double(value(:).');
    if isscalar(value), value=[value value]; end
    value=value(1:2);
    assert(all(isfinite(value)), 'Invalid XY geometry override.');
end
