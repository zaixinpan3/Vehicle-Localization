function results=runObserverWithPyramid(levels,label,trustRadius)
% runObserverWithPyramid Closed-loop observer replays with a canonicalized map pyramid.
% Each LiDAR call solves the coarse levels without position aid and the last
% level with the existing GNSS hypothesis selection. Scenarios: GNSS+LiDAR
% and LiDAR only. Reference poses are used for evaluation only.
    if nargin<3,trustRadius=Inf;end
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    E=load('output/mncav_coarse_localization_20260924/observer/experiment.mat','data','lateral','lateralDesign','cfg','reference');
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources');
    L=size(levels,1);maps=cell(L,1);sources=cell(L,1);
    for l=1:L
        maps{l}=canonicalizeSemanticCloud(C.fixed,levels(l,1));
        sources{l}=cellfun(@(s)canonicalizeSemanticCloud(s,levels(l,2)),C.sources,UniformOutput=false);
    end
    regCfg=distributionRegistrationConfig();ref=E.reference;t=E.data.highRate.time;post=t>=t(1)+2;
    base=rmfield(E.data,'lidar');rows=cell(2,11);results=struct();
    for scenario=["both","lidar_only"]
        data=base;if scenario=="lidar_only",data=rmfield(data,'gnss');end
        data.lidarMatcher=@(k,seed,aid) matchPyramid(maps,sources,k,seed,regCfg,aid,trustRadius);
        est=runFullLocalizationObserver(data,E.lateralDesign,E.cfg,LateralInputs=E.lateral);
        e=vecnorm(est.position-ref(:,1:2),2,2);yaw=rad2deg(atan2(sin(est.pose(:,3)-ref(:,3)),cos(est.pose(:,3)-ref(:,3))));
        p=cell2mat(cellfun(@(x)x.poseXYTheta,est.matchingResults,UniformOutput=false));a=cellfun(@(x)x.accepted,est.matchingResults);
        em=vecnorm(p(:,1:2)-ref(:,1:2),2,2);sec=cellfun(@(x)x.matchingSeconds,est.matchingResults);
        rows(find(scenario==["both","lidar_only"]),:)={label,scenario,100*rms(e),100*rms(e(post)),100*prctile(e(post),95),100*max(e(post)),nnz(e(post)>.3), ...
            rms(yaw(post)),nnz(a),100*rms(em(a)),1e3*median(sec)};
        fprintf('%s %-10s fused RMSE %.2f cm (after 2 s %.2f, P95 %.2f, max %.2f, >30cm %d), heading %.3f deg, matches %d RMSE %.2f cm, matching %.1f ms median\n',rows{find(scenario==["both","lidar_only"]),:});
        results.(scenario)=struct('errorM',e,'matchErrorM',em,'accepted',a,'seconds',sec);
    end
    T=cell2table(rows,VariableNames={'pyramid','scenario','rmseAllCm','rmseCm','p95Cm','maxCm','above30cm','headingRmseDeg','acceptedMatches','matchRmseCm','matchingMsMedian'});
    writetable(T,fullfile(dest,"observer_"+label+".csv"));
end

function r=matchPyramid(maps,sources,k,seed,cfg,aid,trustRadius)
% Coarse levels select the basin without aid; the last level refines with the
% existing position-aid hypothesis selection. Without aid, a refinement that
% leaves the coarse basin by more than trustRadius is replaced by the coarse
% result, since the fine level is a refinement rather than a new search.
    s=seed;L=numel(maps);coarse=[];
    for l=1:L-1
        coarse=matchLocalProbabilityCloud(maps{l},sources{l}{k},s,cfg,[]);
        if coarse.accepted || coarse.directionalAccepted,s=coarse.poseXYTheta;end
    end
    r=matchLocalProbabilityCloud(maps{L},sources{L}{k},s,cfg,aid);
    aided=~isempty(aid) && isfield(aid,'valid') && aid.valid;
    if ~aided && ~isempty(coarse) && coarse.accepted && r.accepted && norm(r.poseXYTheta(1:2)-coarse.poseXYTheta(1:2))>trustRadius
        r=coarse;r.reason="coarseRetainedByTrustRadius";
    end
end
