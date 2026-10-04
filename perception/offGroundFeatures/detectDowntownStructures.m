function result = detectDowntownStructures(pillars,fullPillars,groundMask,zReference,facade,cfg,featureNames,candidateMasks)
% detectDowntownStructures: OFFLINE ONLY pole and reflective-sign validation.
% Use sparse height counts on the supplied XY lattice for pole proposals,
% then validate original-point shape, structural attachments and local
% ground clearance. No additional XY grid or dense 3D volume is created.
%
% Input: pillars: nonground pillar context; fullPillars: retained frame;
%   groundMask: logical aligned with fullPillars.points; zReference: shared
%   height-bin origin; facade: independent detectFacadeSurfaces result;
%   cfg: downtownStructuralConfig; featureNames: requested channels;
%   candidateMasks: coarse pole/trafficSign masks on this same lattice.
%   Surrounding points supply validation context but cannot receive labels.
% Output: pole/trafficSign decisions with evaluated original pointIndices,
%   candidateMask/acceptedMask, pillar mask, supportFraction and diagnostics.
    if nargin<7, featureNames=["pole","trafficSign"]; end
    xyz=double(pillars.points); ids=double(pillars.pointPillarLinIdx(:));
    original=double(pillars.pointIndices(:)); dims=double(pillars.pillarGeometry.mapSize);
    if nargin<8
        candidateMasks=struct('pole',true(dims),'trafficSign',true(dims));
    end
    intensity=nan(size(ids));
    if isfield(pillars.pointAttributes,'intensity')
        intensity=double(pillars.pointAttributes.intensity(:));
    end
    reflective=isfinite(intensity) & intensity>cfg.intensityThreshold;
    result=struct();
    if any(string(featureNames)=="trafficSign")
        assert(isequal(size(candidateMasks.trafficSign),dims));
        signCandidate=reflective & candidateMasks.trafficSign(ids);
        supportCandidate=signCandidate;
        if isfield(candidateMasks,'trafficSignContext')
            supportCandidate=reflective & candidateMasks.trafficSignContext(ids);
        end
        signMask=false(dims); signMask(ids(supportCandidate))=true;
        components=double(labelmatrix(bwconncomp(signMask,8)));
        % Evaluate connected neighborhood support only for components that
        % touch a candidate point. Context points cannot receive labels.
        touching=unique(components(ids(signCandidate)));
        supportCandidate=supportCandidate & ismember(components(ids),touching);
        [keep,metrics]=validateDowntownSignSupport(xyz(supportCandidate,:),components(ids(supportCandidate)), ...
            double(fullPillars.points(groundMask,:)),cfg.signSupport);
        selected=false(size(ids)); selected(supportCandidate)=keep;
        selected=selected & signCandidate;
        result.trafficSign=makeDecision(original,ids,signCandidate,selected,dims);
        result.trafficSign.supportValidation=metrics;
    end
    if ~any(string(featureNames)=="pole"), return; end
    assert(isequal(size(candidateMasks.pole),dims));
    eligible=~reflective;
    [maps,samples]=buildHeightSupport(xyz,ids,eligible,dims,zReference,cfg);
    geometry=pillars.pillarGeometry;
    [maps.pointScore,maps.lineScore]=buildPillarShapeScores(maps.runLayerMap, ...
        maps.occupiedMask,cfg.candidate,geometry.cellSize(1),geometry.cellSize(2));
    seedFacade=facade.seedLineMap>0;
    if isfield(facade,'fullContextSeedLineMap')
        seedFacade=facade.fullContextSeedLineMap>0;
    end
    [rawMask,supportMask]=proposePoles(maps,samples,seedFacade,cfg,candidateMasks.pole);
    nativeMask=validateSliceSupport(rawMask,maps,samples,zReference,cfg);
    components=double(labelmatrix(bwconncomp(nativeMask,8)));
    selected=nativeMask(ids) & eligible;
    selectedIds=find(selected);
    componentIds=components(ids(selected));
    [keep,shapeMetrics]=validateDowntownPoleShape(xyz(selected,:),componentIds,cfg.pointShape);
    rejected=shapeMetrics.componentId(~shapeMetrics.accepted);
    nativeMask(ismember(components,rejected))=false;
    selected(selectedIds(~keep))=false;
    componentIds=componentIds(keep);
    labels=zeros(size(ids)); labels(selected)=componentIds;
    detail=struct('poleMaskRaw',rawMask,'poleSupportMask',supportMask);
    recovery=recoverDowntownDensePoles(xyz,ids,eligible,facade.pointLineIds, ...
        nativeMask,detail,cfg.denseRecovery);
    labels(recovery.pointMask)=max(components,[],'all')+recovery.pointComponentIds(recovery.pointMask);
    selected=selected | recovery.pointMask;
    [keep,contextMetrics]=validateDowntownPoleContext(xyz(selected,:),original(selected), ...
        labels(selected),double(fullPillars.points),double(fullPillars.pointIndices),cfg.context);
    selectedIds=find(selected); selected(selectedIds(~keep))=false;
    assert(all(candidateMasks.pole(ids(selected))), ...
        'perception:FineOutsideCoarseCandidate','Fine pole labels must stay inside coarse candidates.');
    evaluationMask=rawMask | (supportMask & imdilate(rawMask,ones(2*cfg.denseRecovery.supportRadiusCells+1)));
    evaluated=(evaluationMask(ids) & eligible) | recovery.pointMask;
    result.pole=makeDecision(original,ids,evaluated,selected,dims);
    result.pole.pointShapeValidation=shapeMetrics;
    result.pole.contextValidation=contextMetrics;
    result.pole.denseRecoveryMetrics=recovery.metrics;
    result.pole.rawMask=rawMask;
    result.pole.supportMask=supportMask;
    result.pole.nativeMask=nativeMask;
end

function decision=makeDecision(original,ids,evaluated,accepted,dims)
    mask=false(dims); mask(ids(accepted))=true;
    count=accumarray(ids,1,[prod(dims),1]);
    support=accumarray(ids(accepted),1,[prod(dims),1]);
    pointIndices=original(evaluated);
    decision=struct('pointIndices',pointIndices,'candidateMask',true(size(pointIndices)), ...
        'acceptedMask',accepted(evaluated), ...
        'mask',mask,'supportFraction',reshape(support./max(count,1),dims));
end

function [maps,samples]=buildHeightSupport(xyz,ids,eligible,dims,zReference,cfg)
    number=prod(dims);
    z=floor((xyz(eligible,3)-double(zReference))/cfg.heightResolution)+1;
    [keys,~,membership]=unique(ids(eligible)+(z-1)*number);
    counts=accumarray(membership,1,[numel(keys),1]);
    columns=mod(keys-1,number)+1; heights=floor((keys-1)/number)+1;
    qualified=counts>=cfg.candidate.poleOccupiedLayerMinPoints;
    pairs=sortrows([columns(qualified),heights(qualified)],[1 2]);
    run=zeros(number,1); layers=zeros(number,1);
    if ~isempty(pairs)
        starts=find([true;diff(pairs(:,1))~=0 | diff(pairs(:,2))~=1]);
        run=accumarray(pairs(starts,1),diff([starts;size(pairs,1)+1]),[number,1],@max,0);
        layers=accumarray(pairs(:,1),1,[number,1]);
    end
    [row,col]=ind2sub(dims,columns);
    samples=struct('columns',columns,'heights',heights,'counts',counts, ...
        'rows',row,'cols',col,'numberOfLayers',max([1;heights]));
    maps=struct('runLayerMap',reshape(single(run),dims), ...
        'occupiedLayerCount',reshape(single(layers),dims),'occupiedMask',reshape(layers>0,dims));
end

function [raw,support]=proposePoles(maps,samples,facadeMask,cfg,candidateMask)
    params=resolvePoleDetectionParams(cfg.candidate);
    eligible=maps.occupiedMask & ~facadeMask & candidateMask;
    core=eligible & maps.runLayerMap>=params.coreMinRunLayerThreshold & maps.lineScore<params.coreMaxLineScore;
    support=eligible & maps.occupiedLayerCount>=params.componentSupportMinOccupiedLayers;
    split=splitLayerCandidates(eligible,support,maps,params);
    footprintCfg=struct('minimumContextFraction',params.componentContextMinRunLayerRatio, ...
        'maximumFootprintSpanCells',params.maxFootprintSpanCells);
    [~,~,compact]=selectPillarFootprints(core,support,maps.occupiedLayerCount, ...
        maps.runLayerMap,footprintCfg,split);
    raw=false(size(compact));
    components=bwconncomp(compact,8);
    for k=1:components.NumObjects
        cells=components.PixelIdxList{k}; selected=ismember(samples.columns,cells);
        counts=accumarray(samples.heights(selected),samples.counts(selected),[samples.numberOfLayers,1]);
        if nnz(counts>=cfg.candidate.poleOccupiedLayerMinPoints)>=3
            raw(cells)=true;
        end
    end
end

function candidate=splitLayerCandidates(eligible,support,maps,params)
    layers=double(maps.occupiedLayerCount); run=double(maps.runLayerMap);
    member=eligible & support & layers>=params.splitLayerMinMemberOccupiedLayers & ...
        maps.pointScore>=params.splitLayerMinPointScore & maps.lineScore<=params.splitLayerMaxLineScore;
    horizontal=member(:,1:end-1) & member(:,2:end) & ...
        layers(:,1:end-1)+layers(:,2:end)>=params.splitLayerMinCombinedOccupiedLayers & ...
        run(:,1:end-1)+run(:,2:end)>=params.splitLayerMinCombinedRunLayers & ...
        max(layers(:,1:end-1),layers(:,2:end))>=params.coreMinRunLayerThreshold;
    vertical=member(1:end-1,:) & member(2:end,:) & ...
        layers(1:end-1,:)+layers(2:end,:)>=params.splitLayerMinCombinedOccupiedLayers & ...
        run(1:end-1,:)+run(2:end,:)>=params.splitLayerMinCombinedRunLayers & ...
        max(layers(1:end-1,:),layers(2:end,:))>=params.coreMinRunLayerThreshold;
    candidate=false(size(eligible));
    candidate(:,1:end-1)=horizontal; candidate(:,2:end)=candidate(:,2:end)|horizontal;
    candidate(1:end-1,:)=candidate(1:end-1,:)|vertical;
    candidate(2:end,:)=candidate(2:end,:)|vertical;
end

function accepted=validateSliceSupport(raw,maps,samples,zReference,cfg)
% Reduce each footprint to one height profile; keep whole support columns.
    accepted=false(size(raw)); components=bwconncomp(raw,8); p=cfg.slice;
    for k=1:components.NumObjects
        cells=components.PixelIdxList{k};
        [row,col]=ind2sub(size(raw),cells);
        object=ismember(samples.columns,cells);
        neighborhood=samples.rows>=min(row)-1 & samples.rows<=max(row)+1 & ...
            samples.cols>=min(col)-1 & samples.cols<=max(col)+1;
        n=samples.numberOfLayers;
        objectCount=accumarray(samples.heights(object),samples.counts(object),[n,1]);
        contextCount=accumarray(samples.heights(neighborhood),samples.counts(neighborhood),[n,1]);
        ratio=objectCount./max(contextCount,1);
        qualified=objectCount>=max(p.minimumLayerPoints,cfg.candidate.poleOccupiedLayerMinPoints) & ratio>p.minimumSliceRatio;
        qualifiedCount=nnz(qualified);
        if qualifiedCount==0, continue; end
        meanRatio=mean(ratio(qualified)); totalRatio=sum(objectCount(qualified))/sum(contextCount(qualified));
        valid=qualifiedCount>=p.minimumCandidateLayers;
        if ~valid && p.allowRelaxed && qualifiedCount>=p.minimumCandidateLayers-1
            base=double(zReference)+(find(qualified,1)-1)*cfg.heightResolution;
            valid=base<=p.relaxedMaximumBaseHeight && meanRatio>=p.relaxedMinimumMeanRatio && ...
                totalRatio>=p.relaxedMinimumTotalRatio;
        end
        if ~valid || meanRatio<p.minimumMeanRatio || totalRatio<p.minimumTotalRatio, continue; end
        kept=unique(samples.columns(object & qualified(samples.heights)));
        if isempty(kept) || mean(double(maps.pointScore(kept)))<p.minimumPointScore, continue; end
        accepted(kept)=true;
    end
end
