function summary=run_graph_experiment(loss)
% run_graph_experiment Compare graph output and conditional map associations.
    if nargin<1,loss="switchable";end
    loss=string(loss);
    setupVehicleLocalization;out='output/robust_pose_graph_20260923';dest=fileparts(mfilename('fullpath'));
    clouds=load(fullfile(out,'sources.mat'));
    input=load('output/temporal_perception_20260922/observer/experiment.mat','data','cfg','lateral','reference');
    previous=load('output/gnss_aided_matching_20260922/closed_loop.mat','aided');
    replay=load('output/temporal_perception_20260922/five_frame_matching/report.mat','report');
    motion=replay.report.deadReckoning{:,{'x','y','psi'}};
    cfg=robustPoseGraphConfig();cfg.featureLoss=loss;
    first=clouds.calls{1,{'predictedX','predictedY','predictedPsi'}};
    estimate=runRobustGraphLocalization(input.data,input.lateral,clouds,clouds.fixed,motion,first,cfg,input.cfg);
    n=numel(estimate.time);reference=clouds.calls{1:n,{'referenceX','referenceY','referencePsi'}};
    errors=vecnorm(estimate.pose(:,1:2)-reference(:,1:2),2,2);
    before=vecnorm(previous.aided.position-reference(:,1:2),2,2);
    matched=estimate.conditionalMatching;valid=cellfun(@(r)r.accepted,matched);
    pose=cell2mat(cellfun(@(r)r.poseXYTheta,matched,UniformOutput=false));
    matchingError=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);
    rows=cell(0,7);
    for range=["all","after_2_seconds"]
        use=true(n,1);if range=="after_2_seconds",use=estimate.time>=2;end
        rows(end+1,:)={range,nnz(use),rms(errors(use)),prctile(errors(use),95),max(errors(use)),nnz(errors(use)>.3),rms(before(use))}; %#ok<AGROW>
    end
    metrics=cell2table(rows,VariableNames={'population','frames','graphRmseM','graphP95M','graphMaxM','above30cm','baselineRmseM'});
    frame=clouds.calls.frame(1:n);seconds=estimate.seconds;
    frames=table(frame,before,errors,matchingError,valid,seconds);
    rows=cell(0,7);
    for k=1:n
        p=estimate.results{k}.correspondences;
        for j=find(p.semanticName=="trafficSign" | p.graphInfluence<.25).'
            rows(end+1,:)={k,p.source(j),p.globalTarget(j),p.semanticName(j),p.squaredStandardizedResidual(j),p.graphInfluence(j),p.weight(j)}; %#ok<AGROW>
        end
    end
    associations=cell2table(rows,VariableNames={'frame','source','globalTarget','class','squaredResidual','graphInfluence','sourceWeight'});
    summary=struct('loss',loss,'metrics',metrics,'matchingAccepted',nnz(valid), ...
        'matchingRmseM',rms(matchingError(valid)),'matchingMaxM',max(matchingError(valid)), ...
        'totalSeconds',sum(seconds),'medianSeconds',median(seconds),'p95Seconds',prctile(seconds,95), ...
        'uniqueObservations',estimate.uniqueObservationCount, ...
        'nonconvergedFrames',nnz(~cellfun(@(r)r.converged,estimate.results)),'containsGnss',true, ...
        'informationCalibrated',false,'marginalizedAcquisitions',estimate.marginalizedAcquisitions);
    disp(summary);disp(metrics);
    save(fullfile(out,loss+'.mat'),'estimate','summary','frames','associations','cfg','-v7.3');
    writetable(frames,fullfile(dest,loss+'_frames.csv'));writetable(metrics,fullfile(dest,loss+'_metrics.csv'));
    writetable(associations,fullfile(dest,loss+'_associations.csv'));
    fid=fopen(fullfile(dest,loss+'_summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));
end
