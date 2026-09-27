function evidence=continuousPillarHeightSupport(pillars,mapSize,minimumPoints)
% continuousPillarHeightSupport: Longest dense vertical run in each whole cell.
% Sliding triples/groups use actual sorted heights, not height bins or voxels.
    ids=double(pillars.pointPillarLinIdx);z=double(pillars.points(:,3));keep=true(size(ids));
    if isfield(pillars.pointAttributes,'intensity')
        intensity=double(pillars.pointAttributes.intensity);keep=~(isfinite(intensity)&intensity>1800);
    end
    pairs=sortrows([ids(keep),z(keep)],[1 2]);evidence=zeros(mapSize,'single');if isempty(pairs),return;end
    starts=[1;find(diff(pairs(:,1))~=0)+1];ends=[starts(2:end)-1;size(pairs,1)];
    for k=1:numel(starts)
        height=pairs(starts(k):ends(k),2);n=numel(height);if n<minimumPoints,continue;end
        dense=height(minimumPoints:end)-height(1:end-minimumPoints+1)<=.5;
        qualified=false(n,1);begins=find(dense);
        for j=0:minimumPoints-1,qualified(begins+j)=true;end
        height=height(qualified);if numel(height)<minimumPoints,continue;end
        cut=[0;find(diff(height)>.5);numel(height)];span=max(height(cut(2:end))-height(cut(1:end-1)+1));
        if span>=1,evidence(pairs(starts(k),1))=single(span/.5);end
    end
end
