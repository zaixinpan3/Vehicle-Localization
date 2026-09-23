function summary=run_guided_experiment()
% run_guided_experiment Preserve the observer while testing graph proposals.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));out='output/robust_pose_graph_20260923';
    clouds=load(fullfile(out,'sources.mat'));
    input=load('output/temporal_perception_20260922/observer/experiment.mat','data','cfg','lateral');
    replay=load('output/temporal_perception_20260922/five_frame_matching/report.mat','report');
    baseline=load('output/gnss_aided_matching_20260922/closed_loop.mat','aided','observerCfg');
    motion=replay.report.deadReckoning{:,{'x','y','psi'}};cfg=robustPoseGraphConfig();
    data=rmfield(input.data,'lidar');
    data.lidarMatcher=createRobustGraphMatcher(clouds,clouds.fixed,motion,data.highRate.time,cfg);
    estimate=runSynchronousLocalizationObserver(data,baseline.observerCfg,input.lateral);
    n=numel(estimate.time);reference=clouds.calls{1:n,{'referenceX','referenceY','referencePsi'}};
    error=vecnorm(estimate.position-reference(:,1:2),2,2);r=estimate.matchingResults;
    pose=cell2mat(cellfun(@(a)a.poseXYTheta,r,UniformOutput=false));
    matchingError=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);valid=cellfun(@(a)a.accepted,r);
    old=baseline.aided.matchingResults;oldPose=cell2mat(cellfun(@(a)a.poseXYTheta,old,UniformOutput=false));
    oldError=vecnorm(oldPose(:,1:2)-reference(:,1:2),2,2);common=valid & cellfun(@(a)a.accepted,old);
    seconds=cellfun(@(a)a.graphAiding.totalSeconds,r);post=estimate.time>=2;
    summary=struct('fusionRmseM',rms(error),'fusionMaxM',max(error), ...
        'postStartupRmseM',rms(error(post)),'postStartupMaxM',max(error(post)), ...
        'commonMatchingFrames',nnz(common),'matchingRmseM',rms(matchingError(common)), ...
        'baselineMatchingRmseM',rms(oldError(common)),'matchingMaxM',max(matchingError(common)), ...
        'matchingAbove30cm',nnz(matchingError(common)>.3),'acceptedFrames',nnz(valid), ...
        'totalSeconds',sum(seconds),'medianSeconds',median(seconds),'p95Seconds',prctile(seconds,95));
    frames=table(clouds.calls.frame(1:n),error,matchingError,oldError,valid,seconds, ...
        VariableNames={'frame','fusionErrorM','matchingErrorM','baselineMatchingErrorM','accepted','seconds'});
    save(fullfile(out,'guided.mat'),'estimate','summary','frames','cfg','-v7.3');
    writetable(frames,fullfile(dest,'guided_frames.csv'));
    fid=fopen(fullfile(dest,'guided_summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));disp(summary);
end
