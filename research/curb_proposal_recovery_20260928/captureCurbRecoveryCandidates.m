function captureCurbRecoveryCandidates()
% captureCurbRecoveryCandidates: Export omitted raw proposals before label join.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    out=fullfile(root,'output','curb_proposal_recovery_20260928');if ~isfolder(out),mkdir(out);end
    cache=load(fullfile(root,'output','semantic_precision_20260927','mississippi_baseline.mat'),'frames','references','names');
    cfg=perceptionConfig('Mississippi');cfg.featureNames="curb";cfg.semanticPrecision.enabled=false;cfg.coarseProbabilityCloud.storeDiagnostics=true;
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));lastBlock=-1;rows={};started=tic;
    file=fopen(fullfile(out,'extra_features.csv'),'w');cleanup=onCleanup(@()fclose(file));
    for k=1:numel(cache.frames)
        frameId=cache.frames(k);blockId=floor((frameId-1)/50);
        if blockId~=lastBlock
            first=blockId*50+1;last=min(first+49,max(cache.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
        end
        frame=block(frameId-first+1);p=perceiveFrame(frame,cfg);g=p.diagnostics.ground;
        proposals=g.energyMaps.rawExtractedMask & ~g.curbCellMask & g.energyMaps.totalBase>=.25;
        g.curbCellMask=proposals;
        [base,baseNames,local]=measureSemanticPillarFeatures(g,struct(),"curb");
        [raw,rawNames]=measureSemanticPointDistributions(g,struct(),"curb",p.diagnostics.groundPointContext,struct());
        x=[base,raw];names=[baseNames,rawNames];
        [r,c]=ind2sub(g.cellMapSize,local);xy=g.cellOrigin+([c r]-.5).*g.cellSize;
        geometry=p.candidates.geometry;bins=floor((xy-geometry.origin)./geometry.cellSize)+1;
        ids=sub2ind(geometry.mapSize,bins(:,2),bins(:,1));
        fine=cache.references{k,cache.names=="curb"};[metric,detail]=measureFinePoleAlignment(frame,fine,ids,geometry);
        counts=accumarray(detail.finePointPillarIds(detail.finePointInRoi),1,[prod(geometry.mapSize),1]);
        if k==1,fprintf(file,'frame,pillar,finePointCount,%s\n',strjoin(names,','));end
        values=[repmat(frameId,numel(ids),1),ids,counts(ids),x];format=[repmat('%.12g,',1,size(values,2)-1),'%.12g\n'];
        if ~isempty(values),fprintf(file,format,values.');end
        metric.frame=frameId;rows{end+1,1}=metric; %#ok<AGROW>
        if mod(k,100)==0 || k==numel(cache.frames)
            writetable(struct2table(vertcat(rows{:})),fullfile(folder,'extra_proposals.csv'));
            fprintf('Extra proposals %d/%d %.1fs\n',k,numel(cache.frames),toc(started));
        end
    end
end
