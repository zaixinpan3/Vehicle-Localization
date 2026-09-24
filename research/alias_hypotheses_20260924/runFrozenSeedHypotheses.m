function summary=runFrozenSeedHypotheses()
% runFrozenSeedHypotheses Per-frame hypothesis generation on the recorded recursive seeds.
% Variants: current single-seed solve; map-alias seeds; weak-direction grid.
% Selection among accepted hypotheses uses the registration similarity only.
% The reference-seeded solve is an evaluation-only truth-basin representative.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));out='output/alias_hypotheses_20260924';
    if ~isfolder(out),mkdir(out);end
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');c=C.calls;n=height(c);
    cfg=distributionRegistrationConfig();scale=[1;1;1/cfg.yawLeverArm];
    ref=[c.referenceX,c.referenceY,c.referencePsi];seed=[c.predictedX,c.predictedY,c.predictedPsi];
    err=@(p,k)norm(p(1:2)-ref(k,1:2));
    % Map alias statistics
    names=string(C.fixed.components.semanticName(:));
    for nm=["pole","trafficSign"]
        idx=find(names==nm);m=C.fixed.components.mean(idx,1:2);w=C.fixed.components.mixtureWeight(idx);pairs=0;heavy=0;
        for i=1:numel(idx)-1
            r=vecnorm(m(i+1:end,:)-m(i,:),2,2);sel=r>0.25&r<=1.5;pairs=pairs+nnz(sel);
            heavy=heavy+nnz(sel & w(i+1:end)>=0.05*max(w) & w(i)>=0.05*max(w));
        end
        fprintf('map %s: %d components, same-class pairs 0.25-1.5 m apart: %d (both weights >= 5%% of class max: %d)\n',nm,numel(idx),pairs,heavy);
    end
    rows=zeros(n,14);gridOffsets=[-1.5 -1.2 -0.9 -0.6 -0.3 0.3 0.6 0.9 1.2 1.5];
    for k=1:n
        timer=tic;primary=matchLocalProbabilityCloud(C.fixed,C.sources{k},seed(k,:),cfg,[]);tPrimary=toc(timer);
        truth=matchLocalProbabilityCloud(C.fixed,C.sources{k},ref(k,:),cfg,[]);
        % (c) map-alias hypotheses
        timer=tic;[aliasSeeds,aliases]=enumerateMapAliasSeeds(C.fixed,C.sources{k},seed(k,:));
        candidates={primary};
        for j=1:size(aliasSeeds,1),candidates{end+1}=matchLocalProbabilityCloud(C.fixed,C.sources{k},aliasSeeds(j,:),cfg,[]);end %#ok<AGROW>
        [aliasSel,aliasBest]=select(candidates,k,err);tAlias=toc(timer);
        % (d) weak-direction grid around the primary solution
        timer=tic;gridSel=primary;gridBest=primary;
        if primary.accepted
            I=(primary.information(1:2,1:2)+primary.information(1:2,1:2).')/2;[V,Dg]=eig(I,'vector');[~,i]=min(Dg);u=V(:,i).';
            [fixedLocal,~]=selectLocalProbabilityCloud(C.fixed,primary.poseXYTheta,cfg.localMapRadius);
            model=prepareSemanticRegistrationGeometry(fixedLocal,C.sources{k},primary.poseXYTheta,cfg);
            sims=zeros(size(gridOffsets));
            for j=1:numel(gridOffsets),s=model.linearize([gridOffsets(j)*u,primary.poseXYTheta(3)],scale);sims(j)=s.similarity;end
            [~,order]=sort(sims,'descend');candidates={primary};
            for j=order(1:2)
                if sims(j)>=0.5*primary.similarity
                    candidates{end+1}=matchLocalProbabilityCloud(C.fixed,C.sources{k},[primary.poseXYTheta(1:2)+gridOffsets(j)*u,primary.poseXYTheta(3)],cfg,[]); %#ok<AGROW>
                end
            end
            [gridSel,gridBest]=select(candidates,k,err);
        end
        tGrid=toc(timer);
        rows(k,:)=[k,err(primary.poseXYTheta,k),primary.accepted,primary.similarity,err(truth.poseXYTheta,k),truth.similarity, ...
            err(aliasSel.poseXYTheta,k),aliasSel.similarity,err(aliasBest.poseXYTheta,k),size(aliasSeeds,1), ...
            err(gridSel.poseXYTheta,k),err(gridBest.poseXYTheta,k),tAlias-0,tGrid];
        if mod(k,100)==0,fprintf('frozen %d/%d\n',k,n);end
    end
    T=array2table(rows,VariableNames={'frame','primaryM','primaryAccepted','primarySim','truthM','truthSim', ...
        'aliasSelectedM','aliasSelectedSim','aliasBestM','aliasSeeds','gridSelectedM','gridBestM','aliasSeconds','gridSeconds'});
    writetable(T,fullfile(dest,'frozen_seed_hypotheses.csv'));save(fullfile(out,'frozen.mat'),'T','-v7.3');
    a=logical(T.primaryAccepted);
    report=@(name,e)fprintf('%-22s RMSE %.2f cm, P95 %.2f, max %.2f, >30cm %d, >50cm %d\n',name,100*rms(e(a)),100*prctile(e(a),95),100*max(e(a)),nnz(e(a)>.3),nnz(e(a)>.5));
    report('primary (current)',T.primaryM);report('reference-seeded',T.truthM);
    report('alias selected',T.aliasSelectedM);report('alias best available',T.aliasBestM);
    report('grid selected',T.gridSelectedM);report('grid best available',T.gridBestM);
    fprintf('frames with alias seeds: %d (mean seeds %.1f); alias harmed (>10 cm worse than primary): %d; alias rescued (primary >30cm, selected <15cm): %d\n', ...
        nnz(T.aliasSeeds>0),mean(T.aliasSeeds(T.aliasSeeds>0)),nnz(T.aliasSelectedM>T.primaryM+.1),nnz(T.primaryM>.3&T.aliasSelectedM<.15));
    fprintf('grid harmed: %d; grid rescued: %d\n',nnz(T.gridSelectedM>T.primaryM+.1),nnz(T.primaryM>.3&T.gridSelectedM<.15));
    fprintf('selection reliability: frames where truth-basin solve differs from primary by >30 cm: %d; of these, truth similarity higher: %d\n', ...
        nnz(abs(T.truthM-T.primaryM)>.3),nnz(abs(T.truthM-T.primaryM)>.3 & T.truthSim>T.primarySim));
    fprintf('time per frame: alias %.1f ms mean (%.1f max), grid %.1f ms mean (%.1f max)\n',1e3*mean(T.aliasSeconds),1e3*max(T.aliasSeconds),1e3*mean(T.gridSeconds),1e3*max(T.gridSeconds));
    summary=T;
end

function [selected,best]=select(candidates,k,err)
    selected=candidates{1};best=candidates{1};bestSim=-inf;bestErr=inf;
    for j=1:numel(candidates)
        r=candidates{j};
        if r.accepted && r.similarity>bestSim,bestSim=r.similarity;selected=r;end
        if r.accepted && err(r.poseXYTheta,k)<bestErr,bestErr=err(r.poseXYTheta,k);best=r;end
    end
end
