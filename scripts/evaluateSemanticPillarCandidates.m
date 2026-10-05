function [report,labels]=evaluateSemanticPillarCandidates(frame,gridCfg,candidates,pointMasks)
% evaluateSemanticPillarCandidates: Offline any-target-point pillar agreement.
% A pillar is positive for a class if any original reference point in that
% XY cell has that class. Mixed points and ground/off-ground membership do
% not make a positive pillar false. Count each pillar once, not by point mass.
% Reference-only cells outside the configured lattice remain false negatives;
% range/exclusion filtering does not silently remove reference positives.
% This evaluation helper must never participate in runtime perception.
    xy=[double(frame.x(:)),double(frame.y(:))];extent=pillarGridExtent(gridCfg);
    dims=double(gridCfg.gridDims(:).');spacing=double(gridCfg.voxelSize(:).');
    origin=extent([1 3]);names=string(candidates.semanticNames(:));
    assert(isequal(double(candidates.geometry.mapSize),dims([2 1])) && ...
        max(abs(double(candidates.geometry.origin)-origin))<1e-10 && ...
        max(abs(double(candidates.geometry.cellSize)-spacing))<1e-10, ...
        'evaluation:PillarGeometryMismatch','Candidate and reference lattices differ.');
    bins=floor((xy-origin)./spacing)+1;finite=all(isfinite(xy),2);
    indices=(1:size(xy,1)).';
    if isfield(frame,'pointIndices')
        indices=double(frame.pointIndices(:));
        assert(numel(indices)==size(xy,1) && isequal(sort(indices),(1:size(xy,1)).'), ...
            'evaluation:OriginalPointIndexMapping','Explicit point indices must permute the original frame.');
    end
    labels=struct('semanticNames',names,'positivePillarIndices',{cell(numel(names),1)}, ...
        'outsidePillarCount',zeros(numel(names),1));rows=cell(numel(names),1);
    for k=1:numel(names)
        target=logical(pointMasks.(names(k))(:));
        assert(numel(target)==size(xy,1),'evaluation:PointMaskSize','Reference mask size differs.');
        target=target(indices);
        assert(~any(target&~finite),'evaluation:InvalidReferenceXY','A target point has invalid XY.');
        positive=unique(bins(target,:),'rows');inside=all(positive>=1 & positive<=dims,2);
        ids=unique(sub2ind(dims([2 1]),positive(inside,2),positive(inside,1)));
        selected=unique(double(candidates.pillarIndices{k}(:)));
        assert(all(selected>=1 & selected<=prod(dims) & selected==floor(selected)), ...
            'evaluation:InvalidCandidateId','Candidate IDs must address the shared lattice.');
        labels.positivePillarIndices{k}=ids;labels.outsidePillarCount(k)=nnz(~inside);
        tp=numel(intersect(selected,ids));fp=numel(setdiff(selected,ids));fn=size(positive,1)-tp;
        rows{k}=table(names(k),tp,fp,fn,tp/max(1,tp+fp),tp/max(1,tp+fn), ...
            tp/max(1,tp+fp+fn),numel(selected),size(positive,1),nnz(~inside), ...
            'VariableNames',{'feature','tp','fp','fn','precision','recall','iou', ...
            'candidatePillars','targetPillars','outsideTargetPillars'});
    end
    if isempty(rows),report=table();else,report=vertcat(rows{:});end
end
