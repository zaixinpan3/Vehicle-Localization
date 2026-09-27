function T=exportPoleContext(frames,label)
% exportPoleContext: Labels join only after current candidate features exist.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'),'cases');
    baseline=load(fullfile(root,'output','pillar_fine_alignment_20260926','final_full.mat'),'records');
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    cfg=structuralPillarConfig();cfg.useNativeKernels=true;
    cfg.pole.validation.sparseSupportPointThreshold=0;
    cfg.pole.validation.sparseOwnerPointThreshold=0;
    cloud=coarseSemanticProbabilityCloudConfig();cloud.semanticNames="pole";
    rows={};denominators=cell(numel(frames),1);cache=denominators;
    for k=1:numel(frames)
        j=find(cellfun(@(c)c.frame==frames(k),data.cases),1);c=data.cases{j};
        raw=source.pointClouds(1,frames(k));intensity=double(raw.intensity(c.originalIndices));
        structural=~(isfinite(intensity) & intensity>cfg.trafficSignIntensityThreshold);
        pillars=struct('points',c.points,'pointPillarLinIdx',c.pillarIds, ...
            'pointAttributes',struct('intensity',intensity),'pillarGeometry',c.geometry);
        result=analyzeStructuralPillars(pillars,cfg,cloud);maps=result.columnMaps;
        candidates=find(result.poleCellMask);assert(isequal(candidates,baseline.records{frames(k)}.poleCells));
        [e,h]=validatePillarPoleSupport(c.points,c.pillarIds,c.geometry,maps.poleValidationProposals, ...
            cfg.pole.validation,structural,double(maps.pointScore(double(maps.statistics.pillarIndices))));
        current={};
        for a=1:numel(h)
            if ~h(a).geometryAccepted,continue;end
            feature=measurePoleContext(c.points,structural,h(a));
            for b=1:numel(h(a).ownerIds)
                owner=h(a).ownerIds(b);if ~ismember(owner,candidates) || h(a).ownerCount(b)<cfg.pole.validation.minimumOwnerPoints || h(a).ownerHeight(b)<cfg.pole.validation.minimumOwnerHeight,continue;end
                r=feature;r.ownerCount=h(a).ownerCount(b);r.ownerHeight=h(a).ownerHeight(b);
                r.ownerFraction=r.ownerCount/h(a).acceptedCount;
                r.pointScore=double(maps.pointScore(owner));r.lineScore=double(maps.lineScore(owner));
                r.frame=frames(k);r.pillar=owner;r.hypothesis=a;r.finePointCount=nnz(c.fineIds==owner);
                rows{end+1,1}=r;current{end+1,1}=r; %#ok<AGROW>
            end
        end
        denominators{k}=struct('frame',frames(k),'finePoints',numel(c.fineIds),'finePillars',numel(unique(c.fineIds)), ...
            'baselineSelected',numel(candidates),'baselineMatched',nnz(ismember(candidates,c.fineIds)));
        cache{k}=struct('frame',frames(k),'hypotheses',h,'evidence',e,'rows',{current});
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,[label '_features.csv']));
    D=struct2table(vertcat(denominators{:}));writetable(D,fullfile(folder,[label '_frames.csv']));
    save(fullfile(root,'output','pole_context_20260927',[label '.mat']),'cache','T','D','-v7.3');
    fprintf('%s: %d hypotheses/owners, %d frames\n',label,height(T),height(D));
end
