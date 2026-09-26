function captureShaftPipeline(dataset,frames,label,withFine)
% captureShaftPipeline: Frozen-configuration replay and independent-sequence checks.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    if nargin<4,withFine=false;end
    cfg=perceptionConfig(dataset);cfg.offGroundFeatures.pole.detector="shaft";
    cfg.offGroundFeatures.pole.probabilityEvidence="shaft";cfg.coarseProbabilityCloud.storeDiagnostics=true;
    old=cfg;old.offGroundFeatures.pole.detector="pillar";old.offGroundFeatures.pole.probabilityEvidence="distribution";
    sub=cfg;sub.offGroundFeatures.pole.detector="subset";sub.offGroundFeatures.pole.probabilityEvidence="subset";
    if dataset=="Mississippi",filename='MissisipiPointClouds.mat';else,filename='downTownPointClouds.mat';end
    source=matfile(fullfile(root,'data','raw',filename));
    fineCfg=perceptionConfig(dataset,"offline");
    if dataset=="Mississippi"
        frozen=load(fullfile(root,'output','coarse_lattice_20260924','distributionEvidenceFull_sequence.mat'));
        coarse=load(fullfile(root,'output','coarse_lattice_20260924','baseline_sequence.mat'));
        fineCache=load(fullfile(root,'output','coarse_lattice_20260924','fine_pole_pillars.mat'));
    end
    rows=cell(numel(frames),1);records=rows;
    out=fullfile(root,'output','pillar_shaft_20260926');if ~isfolder(out),mkdir(out);end
    log=fopen(fullfile(out,[label '_progress.log']),'w');clean=onCleanup(@()fclose(log));
    for first=1:25:numel(frames)
        indices=first:min(first+24,numel(frames));requested=frames(indices);
        if numel(requested)<3 || all(diff(requested)==requested(2)-requested(1))
            block=source.pointClouds(1,requested);
        else
            block=source.pointClouds(1,requested(1));
            for b=2:numel(requested),block(b)=source.pointClouds(1,requested(b));end
        end
        for k=indices
            frame=block(k-first+1);timer=tic;p=perceiveFrame(frame,cfg);elapsed=toc(timer);
            ids=poleIds(p);g=p.candidates.geometry;legacyTime=NaN;subsetCount=NaN;subsetMatch=NaN;
            if withFine
                timer=tic;b=perceiveFrame(frame,old);legacyTime=toc(timer);oldIds=poleIds(b);
                s=perceiveFrame(frame,sub);fine=perceiveFrame(frame,fineCfg);
                fineIds=projectPoints(frame,fine.featureMasks.pole,g);fineMatched=numel(intersect(ids,fineIds));
                oldFine=numel(intersect(oldIds,fineIds));subsetCount=numel(poleIds(s));subsetMatch=numel(intersect(poleIds(s),fineIds));
                assert(isequaln(p.diagnostics.offGround.columnMaps.statistics,b.diagnostics.offGround.columnMaps.statistics));
                assert(isequaln(p.diagnostics.offGround.columnMaps.moments,b.diagnostics.offGround.columnMaps.moments));
                compareNonPole(p,b);
            else
                b=frozen.records{frames(k)};oldIds=double(b.poleCells);fineIds=[];fineMatched=NaN;oldFine=NaN;
                assert(isequal(p.sourceSummary,b.sourceSummary));
                names=p.candidates.semanticNames;
                for channel=["curb","trafficSign"]
                    assert(isequal(p.candidates.pillarIndices{names==channel},b.pillarIndices{b.semanticNames==channel}));
                end
                assert(isequal(p.diagnostics.ground.energyMaps.total(p.diagnostics.ground.curbCellMask),double(b.curbEnergy)) || ...
                    isequal(single(p.diagnostics.ground.energyMaps.total(p.diagnostics.ground.curbCellMask)),b.curbEnergy));
                assert(isequal(double(p.diagnostics.offGround.trafficSignProbability(p.diagnostics.offGround.trafficSignCellMask)),double(b.signProbability)));
                compareStoredComponents(p.probabilityCloud,b);
                index=find([fineCache.fine.frame]==frames(k),1);
                if ~isempty(index)
                    mask=false(size(frame.x));mask(fineCache.fine(index).polePoints)=true;
                    fineIds=projectPoints(frame,mask,g);fineMatched=numel(intersect(ids,fineIds));oldFine=numel(intersect(oldIds,fineIds));
                end
            end
            assert(all(ismember(oldIds,ids)),'A legacy candidate was lost.');
            reference=[];
            if dataset=="Mississippi",reference=projectCells(coarse.records{frames(k)}.poleCells,coarse.geometry,g);end
            rows{k}=struct('frame',frames(k),'milliseconds',1000*elapsed,'legacyMilliseconds',1000*legacyTime, ...
                'legacyCount',numel(oldIds),'candidateCount',numel(ids),'retainedLegacy',numel(intersect(ids,oldIds)), ...
                'fineCount',numel(fineIds),'fineMatched',fineMatched,'legacyFineMatched',oldFine, ...
                'subsetCount',subsetCount,'subsetFineMatched',subsetMatch, ...
                'coarseCount',numel(reference),'coarseShared',numel(intersect(ids,reference)), ...
                'coarseMatchedNewTolerant',nnz(ismember(ids,dilate(reference,g.mapSize))), ...
                'coarseMatchedOldTolerant',nnz(ismember(reference,dilate(ids,g.mapSize))));
            records{k}=struct('frame',frames(k),'poleCells',ids,'legacyCells',oldIds,'fineCells',fineIds, ...
                'poleProbability',p.diagnostics.offGround.poleProbability(p.diagnostics.offGround.poleCellMask), ...
                'components',p.probabilityCloud.components,'sourceSummary',p.sourceSummary);
        end
        fprintf(log,'%d/%d\n',indices(end),numel(frames));fprintf('%s %d/%d\n',label,indices(end),numel(frames));
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,[label '_pipeline.csv']));
    save(fullfile(out,[label '_pipeline.mat']),'records','cfg','frames','dataset','g','-v7.3');
    fprintf('%s: %d legacy, %d candidates, fine %d/%d, median %.1f ms\n',label,sum(T.legacyCount),sum(T.candidateCount),sum(T.fineMatched,'omitnan'),sum(T.fineCount),median(T.milliseconds));
end

function ids=poleIds(p)
    ids=double(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"});
end
function ids=projectPoints(frame,mask,g)
    xy=double([frame.x(mask),frame.y(mask)]);bins=floor((xy-g.origin)./g.cellSize)+1;
    valid=all(bins>=1 & bins<=[g.mapSize(2),g.mapSize(1)],2);
    ids=unique(sub2ind(g.mapSize,bins(valid,2),bins(valid,1)));
end
function ids=projectCells(ids,source,target)
    [r,c]=ind2sub(source.mapSize,double(ids));xy=source.origin+([c r]-.5).*source.cellSize;
    bins=floor((xy-target.origin)./target.cellSize)+1;valid=all(bins>=1 & bins<=[target.mapSize(2),target.mapSize(1)],2);
    ids=unique(sub2ind(target.mapSize,bins(valid,2),bins(valid,1)));
end
function ids=dilate(ids,dims)
    if isempty(ids),return;end
    [r,c]=ind2sub(dims,ids);[dr,dc]=meshgrid(-1:1,-1:1);rr=r+dr(:).';cc=c+dc(:).';valid=rr>=1 & rr<=dims(1) & cc>=1 & cc<=dims(2);
    ids=unique(sub2ind(dims,rr(valid),cc(valid)));
end
function compareNonPole(a,b)
    names=a.candidates.semanticNames;
    for name=names(names~="pole").'
        assert(isequal(a.candidates.pillarIndices{names==name},b.candidates.pillarIndices{b.candidates.semanticNames==name}));
    end
    stored=struct('components',b.probabilityCloud.components,'cloudSemanticNames',b.probabilityCloud.semanticNames);
    compareStoredComponents(a.probabilityCloud,stored);
end
function compareStoredComponents(a,b)
    aa=a.components.semanticId~=find(a.semanticNames=="pole");bb=b.components.semanticId~=find(b.cloudSemanticNames=="pole");
    for field={'semanticId','count','mean','covariance','meanXYZ','semanticProbability','occupancyProbability'}
        x=a.components.(field{1});y=b.components.(field{1});
        if strcmp(field{1},'covariance'),assert(isequaln(x(:,:,aa),y(:,:,bb)));else,assert(isequaln(x(aa,:),y(bb,:)));end
    end
end
