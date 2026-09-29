function [evidence,diagnostic]=classifyPillarPoleSupport(points,pillarIds,geometry,modes,validation,structural,pointScore,lineScore,cfg)
% classifyPillarPoleSupport: Joint context validation of continuous shaft subsets.
% The initial shaft distribution still needs concentrated, continuous support.
% A learned context rule replaces independent post-fit width/tilt/count vetoes;
% every selected owner supplies its own measured returns and vertical extent.
    assert(all(abs(geometry.cellSize-[.6 .6])<1e-12), ...
        'perception:PoleModelLattice','The distribution model requires the shared 0.6 m lattice.');
    ids=double(modes.pillarIndices(:));n=numel(ids);
    evidence=struct('pillarIndices',ids,'found',false(n,1),'score',zeros(n,1), ...
        'ownCount',zeros(n,1),'height',zeros(n,1),'radialRms',zeros(n,1),'isolation',zeros(n,1));
    hypotheses=measurePillarPoleSupport(points,pillarIds,geometry,modes,validation,structural);
    model=loadPillarPoleModel(cfg.modelFile);
    [features,owners,which]=measurePillarPoleFeatures(points,pillarIds,geometry,hypotheses, ...
        structural,pointScore,lineScore,cfg,model.featureNames);
    score=scorePillarPoleModel(features,model);
    if isfield(cfg,'minorityShaftProtection')
        protected=protectMinorityPoleShaft(features,model.featureNames,cfg.minorityShaftProtection);
        score(protected)=max(score(protected),cfg.minimumScore);
    end
    % Training excludes the central vehicle square with a 3 m half-width.
    % Preserve geometric behavior for near-field inputs when callers widen
    % that ROI, instead of extrapolating an untrained learned decision there.
    near=arrayfun(@(h)norm(h.axisXY)<cfg.minimumLearnedRange,hypotheses);
    if any(near)
        score(ismember(which,find(near)))=-Inf;
        old=validatePillarPoleSupport(points,pillarIds,geometry,modes,validation,structural,double(pointScore(ids)));
        nearOwners=unique(vertcat(hypotheses(near).ownerIds));keep=old.found & ismember(ids,nearOwners);
        for field={'found','score','ownCount','height','radialRms','isolation'}
            evidence.(field{1})(keep)=old.(field{1})(keep);
        end
    end
    for k=find(score>=cfg.minimumScore).'
        h=hypotheses(which(k));row=find(ids==owners(k),1);
        if score(k)<=evidence.score(row),continue;end
        at=find(h.ownerIds==owners(k),1);
        evidence.found(row)=true;evidence.score(row)=score(k);evidence.ownCount(row)=h.ownerCount(at);
        evidence.height(row)=h.supportHeight;evidence.radialRms(row)=h.radialRms;evidence.isolation(row)=h.isolation;
    end
    if nargout>1
        diagnostic=struct('features',features,'owners',owners,'hypothesisIds',which,'scores',score,'hypotheses',hypotheses);
    end
end
