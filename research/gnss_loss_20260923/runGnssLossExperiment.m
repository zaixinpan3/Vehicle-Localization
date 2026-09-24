function summary=runGnssLossExperiment()
% runGnssLossExperiment Closed-loop observer replays with 20% of GNSS frames withdrawn.
% Patterns: independent frame dropout, 1 s and 5 s bursts, and one contiguous
% block at five positions. Withdrawn frames provide neither a GNSS correction
% nor matching aid. Every run rematches LiDAR with its own fused seed.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    out='output/gnss_loss_20260923';if ~isfolder(out),mkdir(out);end
    E=load('output/localization_evaluation_20260923/observer/experiment.mat','data','lateral','lateralDesign','cfg','reference');
    C=load('output/localization_evaluation_20260923/sources.mat','fixed','sources');
    base=rmfield(E.data,'lidar');ref=E.reference;t=base.highRate.time;n=numel(t);
    % Startup is excluded from withdrawals so that the declared initialization
    % transient is common to every run; metrics are reported after 2 s.
    eligible=find(t>=t(1)+2);m=numel(eligible);lost=round(.2*m);
    runs=struct('pattern',{},'seed',{},'mask',{});
    runs(end+1)=struct('pattern',"none",'seed',0,'mask',false(n,1));
    for seed=1:10
        rng(seed,'twister');mask=false(n,1);mask(eligible(randperm(m,lost)))=true;
        runs(end+1)=struct('pattern',"iid_frames",'seed',seed,'mask',mask); %#ok<AGROW>
    end
    for len=[10 50]
        for seed=1:10
            rng(1000*len+seed,'twister');slots=floor(m/len);chosen=randperm(slots,round(lost/len));
            mask=false(n,1);
            for s=chosen,mask(eligible((s-1)*len+(1:len)))=true;end
            runs(end+1)=struct('pattern',"burst_"+len/10+"s",'seed',seed,'mask',mask); %#ok<AGROW>
        end
    end
    for startSecond=[5 25 45 65 85]
        first=find(t>=t(1)+startSecond,1);mask=false(n,1);mask(first:first+lost-1)=true;
        runs(end+1)=struct('pattern',"block_"+lost/10+"s",'seed',startSecond,'mask',mask); %#ok<AGROW>
    end
    runs(end+1)=struct('pattern',"all_lost",'seed',0,'mask',true(n,1));
    regCfg=distributionRegistrationConfig();post=t>=t(1)+2;
    rows=cell(numel(runs),17);traces=zeros(n,numel(runs));
    for r=1:numel(runs)
        data=base;data.gnss.valid=base.gnss.valid & ~runs(r).mask;
        data.lidarMatcher=@(k,seed,aid) matchLocalProbabilityCloud(C.fixed,C.sources{k},seed,regCfg,aid);
        timer=tic;est=runFullLocalizationObserver(data,E.lateralDesign,E.cfg,LateralInputs=E.lateral);seconds=toc(timer);
        e=vecnorm(est.position-ref(:,1:2),2,2);traces(:,r)=e;
        h=rad2deg(atan2(sin(est.pose(:,3)-ref(:,3)),cos(est.pose(:,3)-ref(:,3))));
        pose=cell2mat(cellfun(@(x)x.poseXYTheta,est.matchingResults,UniformOutput=false));
        accepted=cellfun(@(x)x.accepted,est.matchingResults);
        em=vecnorm(pose(:,1:2)-ref(:,1:2),2,2);
        lostFrames=runs(r).mask & post;keptFrames=~runs(r).mask & post;
        rows(r,:)={runs(r).pattern,runs(r).seed,nnz(~data.gnss.valid & post)/nnz(post), ...
            100*rms(e(post)),100*median(e(post)),100*prctile(e(post),95),100*max(e(post)), ...
            rms(h(post)),100*rmsOrNaN(e(lostFrames)),100*rmsOrNaN(e(keptFrames)), ...
            nnz(accepted & post),100*rms(em(accepted & post)),100*max(em(accepted & post)), ...
            nnz(em(accepted & post)>.3),longestRun(~data.gnss.valid & post)/10,nnz(e(post)>.3),seconds};
        fprintf('%-12s seed %3d: RMSE %.2f cm, max %.2f cm, lost-frame RMSE %.2f cm\n', ...
            runs(r).pattern,runs(r).seed,rows{r,4},rows{r,7},rows{r,9});
    end
    results=cell2table(rows,VariableNames={'pattern','seed','gnssUnavailableFraction', ...
        'rmseCm','medianCm','p95Cm','maxCm','headingRmseDeg','rmseOnLostFramesCm','rmseOnKeptFramesCm', ...
        'acceptedMatches','matchRmseCm','matchMaxCm','matchAbove30cm','longestGapSeconds','fusedAbove30cm','seconds'});
    groups=unique(results.pattern,'stable');aggregate=cell(numel(groups),9);
    for g=1:numel(groups)
        s=results(results.pattern==groups(g),:);
        aggregate(g,:)={groups(g),height(s),mean(s.rmseCm),min(s.rmseCm),max(s.rmseCm), ...
            mean(s.p95Cm),max(s.maxCm),mean(s.headingRmseDeg),mean(s.rmseOnLostFramesCm)};
    end
    aggregate=cell2table(aggregate,VariableNames={'pattern','runs','meanRmseCm','minRmseCm','maxRmseCm', ...
        'meanP95Cm','worstMaxCm','meanHeadingRmseDeg','meanRmseOnLostFramesCm'});
    disp(aggregate);
    writetable(results,fullfile(dest,'runs.csv'));writetable(aggregate,fullfile(dest,'summary.csv'));
    masks=[runs.mask];patterns=[runs.pattern];seeds=[runs.seed];
    save(fullfile(out,'traces.mat'),'traces','masks','patterns','seeds','t','ref','-v7.3');
    plotGnssLoss();
    summary=struct('results',results,'aggregate',aggregate);
end

function value=rmsOrNaN(x)
    value=NaN;if ~isempty(x),value=rms(x);end
end

function s=longestRun(mask)
    d=diff([0;mask(:);0]);s=max([0;find(d==-1)-find(d==1)]);
end
