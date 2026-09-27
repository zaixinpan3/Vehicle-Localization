function captureSemanticBaseline(dataset)
% captureSemanticBaseline: Freeze original references and baseline descriptors.
% Fine labels join only after production coarse features have been measured.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    out=fullfile(root,'output','semantic_precision_20260927');dataset=string(dataset);label=lower(dataset);
    cfg=perceptionConfig(dataset);cfg.coarseProbabilityCloud.storeDiagnostics=true;
    if isfield(cfg,'semanticPrecision'),cfg.semanticPrecision.enabled=false;end
    fineCfg=perceptionConfig(dataset,'offline');names=cfg.featureNames;
    frozenFile=fullfile(out,label+"_baseline.mat");reuse=isfile(frozenFile);
    if reuse,frozen=load(frozenFile,'frames','references','processed');reuse=frozen.processed==numel(frozen.frames);end
    if dataset=="Mississippi"
        frames=1:1170;source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
        references=cell(1170,numel(names));
        for worker=1:4
            part=load(fullfile(root,'output','fine_matching_20260919',sprintf('inputs_%d.mat',worker)), ...
                'frames','selectedIndices','cfg','processed');
            assert(part.processed==numel(part.frames));
            for j=1:numel(names)
                channel=find(string(part.cfg.featureNames)==names(j));assert(isscalar(channel));
                references(part.frames,j)=part.selectedIndices(:,channel);
            end
        end
    else
        frames=unique([1:5:539,round(linspace(1,539,24))]);source=matfile(fullfile(root,'data','raw','downTownPointClouds.mat'));
        references=cell(numel(frames),numel(names));
    end
    if reuse,assert(isequal(frames,frozen.frames));references=frozen.references;end
    files=struct();cleanups=cell(1,numel(names));records=cell(numel(frames),1);rows={};schemas=struct();
    for name=names
        if name=="pole",continue;end
        files.(name)=fopen(fullfile(out,label+"_"+name+"_features.csv"),'w');
        cleanups{names==name}=onCleanup(@()fclose(files.(name)));
    end
    started=tic;lastBlock=-1;
    for k=1:numel(frames)
        frameId=frames(k);blockId=floor((frameId-1)/50);
        if blockId~=lastBlock
            first=blockId*50+1;last=min(first+49,max(frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
        end
        frame=block(frameId-first+1);timer=tic;p=perceiveFrame(frame,cfg);coarseSeconds=toc(timer);
        descriptors=struct();
        for name=names
            if name=="pole",continue;end
            [baseX,baseNames,local]=measureSemanticPillarFeatures(p.diagnostics.ground,p.diagnostics.offGround,name);
            [support,supportNames]=measureSemanticPointDistributions(p.diagnostics.ground,p.diagnostics.offGround,name, ...
                p.diagnostics.groundPointContext,p.diagnostics.offGroundPointContext);
            x=[baseX,support];fields=[baseNames,supportNames];
            if name=="facade"
                [groupX,groupNames]=measureFacadeGroupEvidence(p.diagnostics.offGround,p.diagnostics.offGroundPointContext);
                x=[x,groupX];fields=[fields,groupNames]; %#ok<AGROW>
            end
            if name=="curb"
                b=p.diagnostics.ground;origin=b.cellOrigin;spacing=b.cellSize;dims=b.cellMapSize;
            else
                b=p.diagnostics.offGround.columnMaps;origin=b.origin;spacing=[b.dx b.dy];dims=b.mapSize;
            end
            [r,c]=ind2sub(dims,local);xy=origin+([c r]-.5).*spacing;g=p.candidates.geometry;
            bins=floor((xy-g.origin)./g.cellSize)+1;ids=sub2ind(g.mapSize,bins(:,2),bins(:,1));
            assert(isequal(sort(ids),sort(double(p.candidates.pillarIndices{p.candidates.semanticNames==name}))));
            descriptors.(name)=struct('x',x,'ids',ids);
            if k==1
                schemas.(name)=fields;fprintf(files.(name),'frame,pillar,finePointCount,%s\n',strjoin(fields,','));
            else
                assert(isequal(schemas.(name),fields));
            end
        end
        if dataset=="Downtown" && ~reuse
            fine=perceiveFrame(frame,fineCfg);
            for j=1:numel(names),references{k,j}=find(fine.featureMasks.(names(j)));end
        end
        for j=1:numel(names)
            name=names(j);ids=double(p.candidates.pillarIndices{p.candidates.semanticNames==name});
            [metric,detail]=measureFinePoleAlignment(frame,references{k,j},ids,p.candidates.geometry);
            metric.dataset=dataset;metric.frame=frameId;metric.feature=name;metric.coarseSeconds=coarseSeconds;
            rows{end+1,1}=metric; %#ok<AGROW>
            if name=="pole",continue;end
            d=descriptors.(name);targetIds=detail.finePointPillarIds(detail.finePointPillarIds>0);
            counts=accumarray(targetIds(:),ones(numel(targetIds),1),[prod(g.mapSize),1]);
            values=[repmat(frameId,numel(d.ids),1),d.ids,counts(d.ids),d.x];
            format=[repmat('%.12g,',1,size(values,2)-1),'%.12g\n'];
            if ~isempty(values),fprintf(files.(name),format,values.');end
        end
        records{k}=struct('frame',frameId,'candidates',p.candidates,'sourceSummary',p.sourceSummary);
        if mod(k,50)==0 || k==numel(frames)
            processed=k;T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,label+"_baseline.csv"));
            save(fullfile(out,label+"_baseline.mat"),'frames','references','names','cfg','fineCfg','records','schemas','processed','T','-v7.3');
            fprintf('%s %d/%d %.1fs\n',dataset,k,numel(frames),toc(started));
        end
    end
end
