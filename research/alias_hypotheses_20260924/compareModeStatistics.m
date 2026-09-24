function S=compareModeStatistics()
% compareModeStatistics Which statistic tells the true mode from an alias mode?
% Frames: those where the recorded seed and the reference seed converge to
% solutions more than 30 cm apart, plus frames the alias selection harmed in
% the closed loop. Both modes are re-solved with and without relative-height
% association; every statistic is compared between the two modes.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');c=C.calls;
    F=readtable(fullfile(dest,'frozen_seed_hypotheses.csv'));
    L=readtable(fullfile(dest,'closed_loop_frames.csv'));
    frames=unique([F.frame(abs(F.truthM-F.primaryM)>.3);L.frame(L.alias_selectErrorM>L.currentErrorM+.1)]);frames(frames==1)=[];
    cfg=distributionRegistrationConfig();hcfg=cfg;hcfg.relativeHeight.enabled=true;
    ref=[c.referenceX,c.referenceY,c.referencePsi];seed=[c.predictedX,c.predictedY,c.predictedPsi];
    rows=cell(0,25);
    for k=frames.'
        truth=matchLocalProbabilityCloud(C.fixed,C.sources{k},ref(k,:),cfg,[]);
        if ~truth.accepted,continue;end
        list={matchLocalProbabilityCloud(C.fixed,C.sources{k},seed(k,:),cfg,[])};
        aliasSeeds=enumerateMapAliasSeeds(C.fixed,C.sources{k},seed(k,:));
        for j=1:size(aliasSeeds,1),list{end+1}=matchLocalProbabilityCloud(C.fixed,C.sources{k},aliasSeeds(j,:),cfg,[]);end %#ok<AGROW>
        alias=[];bestSim=-inf;
        for j=1:numel(list)
            r=list{j};
            if r.accepted && norm(r.poseXYTheta(1:2)-truth.poseXYTheta(1:2))>.3 && r.similarity>bestSim,bestSim=r.similarity;alias=r;end
        end
        if isempty(alias),continue;end
        % Height-enabled re-solves from each converged pose.
        truthH=matchLocalProbabilityCloud(C.fixed,C.sources{k},truth.poseXYTheta,hcfg,[]);
        aliasH=matchLocalProbabilityCloud(C.fixed,C.sources{k},alias.poseXYTheta,hcfg,[]);
        rows(end+1,:)=[{k,norm(truth.poseXYTheta(1:2)-ref(k,1:2)),norm(alias.poseXYTheta(1:2)-ref(k,1:2))}, ...
            stats(truth),stats(alias),heightStats(truthH),heightStats(aliasH)]; %#ok<AGROW>
        fprintf('frame %4d: truth %.2f m sim %.3f cost %.2f hCost %.2f | alias %.2f m sim %.3f cost %.2f hCost %.2f\n',k,rows{end,2}, ...
            truth.similarity,rows{end,5},rows{end,21},rows{end,3},alias.similarity,rows{end,13},rows{end,24});
    end
    names={'frame','truthErrM','aliasErrM'};
    for side=["truth","alias"],names=[names,cellfun(@(x)char(side+x),{'Sim','Cost','Pairs','MatchedFraction','MeanQ','CurbQ','PoleQ','SignQ'},UniformOutput=false)];end
    for side=["truth","alias"],names=[names,cellfun(@(x)char(side+x),{'HSim','HCostSum','HCostMax'},UniformOutput=false)];end
    S=cell2table(rows,VariableNames=names);writetable(S,fullfile(dest,'mode_statistics.csv'));
    n=height(S);fprintf('\n%d frames with two accepted modes. Fraction where the statistic favours the TRUE mode:\n',n);
    fprintf('  similarity higher         %.2f\n',mean(S.truthSim>S.aliasSim));
    fprintf('  robust cost lower         %.2f\n',mean(S.truthCost<S.aliasCost));
    fprintf('  cost per pair lower       %.2f\n',mean(S.truthCost./S.truthPairs<S.aliasCost./S.aliasPairs));
    fprintf('  mean q lower              %.2f\n',mean(S.truthMeanQ<S.aliasMeanQ));
    fprintf('  curb q lower              %.2f\n',mean(S.truthCurbQ<S.aliasCurbQ));
    fprintf('  pole q lower              %.2f\n',mean(S.truthPoleQ<S.aliasPoleQ,'omitnan'));
    fprintf('  sign q lower              %.2f\n',mean(S.truthSignQ<S.aliasSignQ,'omitnan'));
    fprintf('  more pairs                %.2f\n',mean(S.truthPairs>S.aliasPairs));
    fprintf('  height-enabled similarity higher %.2f\n',mean(S.truthHSim>S.aliasHSim));
    fprintf('  height cost sum lower     %.2f (ties %.2f)\n',mean(S.truthHCostSum<S.aliasHCostSum),mean(S.truthHCostSum==S.aliasHCostSum));
    fprintf('  sim - 0.02*heightCost higher %.2f\n',mean(S.truthHSim-0.02*S.truthHCostSum>S.aliasHSim-0.02*S.aliasHCostSum));
    fprintf('  -2log(sim)+heightCost lower  %.2f\n',mean(-2*log(S.truthHSim)+S.truthHCostSum< -2*log(S.aliasHSim)+S.aliasHCostSum));
end

function s=stats(r)
    p=r.correspondences;q=@(nm)mean(p.squaredStandardizedResidual(p.semanticName==nm));
    s={r.similarity,sum(p.weight.*2.5^2.*log1p(p.squaredStandardizedResidual/2.5^2)),height(p),r.matchedFraction, ...
        mean(p.squaredStandardizedResidual),q("curb"),q("pole"),q("trafficSign")};
end

function s=heightStats(r)
    p=r.correspondences;h=p.heightAssociationCost;h(~isfinite(h))=0;
    s={r.similarity,sum(h),max([0;h])};
end
